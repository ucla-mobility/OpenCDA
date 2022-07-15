import os
import copy
import logging
import time
import numpy as np
import carla
import math
import weakref
import shapely.geometry
from enum import Enum
from easydict import EasyDict
from collections import deque
from typing import Any, Dict, List, Optional, Tuple, Union

from ding.utils.default_helper import deep_merge_dicts

DEFAULT_CAMERA_CONFIG = {
    'size': [384, 160],
    'fov': 90,
    'position': [2.0, 0.0, 1.4],
    'rotation': [0, 0, 0],
}

DEFAULT_CAMERA_AUG_CONFIG = {'position_range': [0, 0, 0], 'rotation_range': [0, 0, 0]}

DEFAULT_LIDAR_CONFIG = {
    'channels': 1,
    'range': 2000,
    'points_per_second': 1000,
    'rotation_frequency': 10,
    'upper_fov': -3,
    'lower_fov': -3,
    'position': [0, 0.0, 1.4],
    'rotation': [0, -90, 0],
    'draw': False,
}

DEFAULT_GNSS_CONFIG = {
    'position': [0.0, 0.0, 1.4],
}


class TrafficLightState(Enum):
    RED = 0
    YELLOW = 1
    GREEN = 2
    OFF = 3


def get_random_sample(range_list):
    res = []
    for _range in range_list:
        num = np.random.random() * _range * 2 - _range
        res.append(num)
    return res

class SensorHelper(object):
    """
    Interfaces for sensors required for vehicles and data buffer for all sensor data in Carla. The updating for Carla
    sensors are not synchronous. In each tick, the newest sensor data is obtained from sensor data buffer and returned
    to the simulator. This class provides an interface that can easily create, receive data and destroy all
    kinds of sensors in Carla according to config, and apply the same sensor augmantation to all camera sensors.

    :Arguments:
        - obs_cfg (Dict): Config dict for sensor
        - aug_cfg (Dict, optional): Config dict for sensor augmentation. Defaults to None.

    :Interfaces: setup_sensors, get_sensors_data, clear_up
    """

    def __init__(
            self,
            obs_cfg: Dict,
            aug_cfg: Optional[Dict] = None,
    ) -> None:
        self._obs_cfg = obs_cfg
        self._obs_cfg_list = [self._obs_cfg]
        self._aug_cfg = aug_cfg
        self._sensors_dict = {}
        self._data_buffers = {}
        self._timestamps = {}
        self._random_aug_pos = None
        self._random_aug_rot = None

    def setup_sensors(self, world: carla.World, vehicle: carla.Actor) -> None:
        """
        Create the sensors defined in config and attach them to the hero vehicle

        :Arguments:
            - world (carla.World): Carla world
            - vehicle (carla.Actor): ego vehicle
        """
        bp_library = world.get_blueprint_library()
        if self._aug_cfg:
            self._aug_cfg = EasyDict(deep_merge_dicts(DEFAULT_CAMERA_AUG_CONFIG, self._aug_cfg))
            if min(self._aug_cfg.position_range) < 0 or min(self._aug_cfg.rotation_range) < 0:
                raise ValueError('Augmentation parameters must greater than 0!')
            self._random_aug_pos = get_random_sample(self._aug_cfg.position_range)
            self._random_aug_rot = get_random_sample(self._aug_cfg.rotation_range)
        else:
            self._random_aug_pos = [0, 0, 0]
            self._random_aug_rot = [0, 0, 0]
        for obs_item in self._obs_cfg_list:
            if obs_item.type in ['rgb', 'depth', 'segmentation']:
                obs_item = EasyDict(deep_merge_dicts(DEFAULT_CAMERA_CONFIG, obs_item))
                bp_name = {
                    'rgb': 'sensor.camera.rgb',
                    'depth': 'sensor.camera.depth',
                    'segmentation': 'sensor.camera.semantic_segmentation',
                }[obs_item.type]
                sensor_bp = bp_library.find(bp_name)
                sensor_bp.set_attribute('image_size_x', str(obs_item.size[0]))
                sensor_bp.set_attribute('image_size_y', str(obs_item.size[1]))
                sensor_bp.set_attribute('fov', str(obs_item.fov))
                sensor_location = carla.Location(
                    obs_item.position[0] + self._random_aug_pos[0], obs_item.position[1] + self._random_aug_pos[1],
                    obs_item.position[2] + self._random_aug_pos[2]
                )
                sensor_rotation = carla.Rotation(
                    obs_item.rotation[0] + self._random_aug_rot[0], obs_item.rotation[1] + self._random_aug_rot[1],
                    obs_item.rotation[2] + self._random_aug_rot[2]
                )

            elif obs_item.type == 'lidar':
                obs_item = EasyDict(deep_merge_dicts(DEFAULT_LIDAR_CONFIG, obs_item))
                sensor_bp = bp_library.find('sensor.lidar.ray_cast')
                sensor_bp.set_attribute('range', str(obs_item.range))
                sensor_bp.set_attribute('rotation_frequency', str(obs_item.rotation_frequency))
                sensor_bp.set_attribute('channels', str(obs_item.channels))
                sensor_bp.set_attribute('upper_fov', str(obs_item.upper_fov))
                sensor_bp.set_attribute('lower_fov', str(obs_item.lower_fov))
                sensor_bp.set_attribute('points_per_second', str(obs_item.points_per_second))
                sensor_location = carla.Location(obs_item.position[0], obs_item.position[1], obs_item.position[2])
                sensor_rotation = carla.Rotation(obs_item.rotation[0], obs_item.rotation[1], obs_item.rotation[2])

            elif obs_item.type == 'gnss':
                obs_item = EasyDict(deep_merge_dicts(DEFAULT_GNSS_CONFIG, obs_item))
                obs_item.update(obs_item)
                sensor_bp = bp_library.find('sensor.other.gnss')
                sensor_location = carla.Location(obs_item.position[0], obs_item.position[1], obs_item.position[2])
                sensor_rotation = carla.Rotation()
            else:
                continue

            sensor_transform = carla.Transform(sensor_location, sensor_rotation)
            sensor = world.spawn_actor(sensor_bp, sensor_transform, attach_to=vehicle)
            sensor.listen(CallBack(obs_item.name, obs_item.type, self))
            self.register_sensor(obs_item.name, sensor)

    def clean_up(self) -> None:
        """
        Remove and destroy all sensors
        """
        for key in self._sensors_dict:
            if self._sensors_dict[key] is not None:
                if self._sensors_dict[key].is_alive:
                    self._sensors_dict[key].stop()
                    self._sensors_dict[key].destroy()
                self._sensors_dict[key] = None
        time.sleep(0.1)
        self._sensors_dict.clear()
        self._data_buffers.clear()
        self._timestamps.clear()

    def register_sensor(self, tag: str, sensor: Any) -> None:
        """
        Registers the sensors
        """
        if tag in self._sensors_dict:
            raise ValueError("Duplicated sensor tag [{}]".format(tag))

        self._sensors_dict[tag] = sensor
        self._data_buffers[tag] = None
        self._timestamps[tag] = -1

    def update_sensor(self, tag: str, data: Any, timestamp: Any) -> None:
        """
        Updates the sensor
        """
        if tag not in self._sensors_dict:
            raise ValueError("The sensor with tag [{}] has not been created!".format(tag))
        self._data_buffers[tag] = data
        self._timestamps[tag] = timestamp

    def all_sensors_ready(self) -> bool:
        """
        Checks if all the sensors have sent data at least once
        """
        for key in self._sensors_dict:
            if self._data_buffers[key] is None:
                return False
        return True

    def get_sensors_data(self) -> Dict:
        """
        Get all registered sensor data from buffer

        :Returns:
            Dict: all newest sensor data
        """
        sensor_data = {}
        for obs_item in self._obs_cfg_list:
            if obs_item.type in ['rgb', 'segmentation', 'lidar', 'gnss']:
                key = obs_item.name
                img = self._data_buffers[key]
                sensor_data[key] = img
            elif obs_item.type == 'depth':
                key = obs_item.name
                raw = self._data_buffers[key]
                img = raw.astype(np.float64)
                R = img[..., 0]
                G = img[..., 1]
                B = img[..., 2]
                depth = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
                depth = 1000 * depth
                sensor_data[key] = depth
        if self._aug_cfg:
            sensor_data['aug'] = {
                'aug_pos': np.array(self._random_aug_pos),
                'aug_rot': np.array(self._random_aug_rot),
            }
        return sensor_data


class CallBack(object):
    """
    Class the sensors listen to in order to receive their data each frame
    """

    def __init__(self, tag: str, type: str, wrapper: Any) -> None:
        """
        Initializes the call back
        """
        self._tag = tag
        self._type = type
        self._data_wrapper = wrapper

    def __call__(self, data: Any) -> None:
        """
        call function
        """
        if isinstance(data, carla.Image):
            self._parse_image_cb(data, self._tag)
        elif isinstance(data, carla.LidarMeasurement):
            self._parse_lidar_cb(data, self._tag)
        elif isinstance(data, carla.GnssMeasurement):
            self._parse_gnss_cb(data, self._tag)
        else:
            logging.error('No callback method for this sensor.')

    # Parsing CARLA physical Sensors
    def _parse_image_cb(self, image: Any, tag: str) -> None:
        """
        parses cameras
        """
        if self._type == 'rgb':
            image.convert(carla.ColorConverter.Raw)
        if self._type == 'segmentation':
            image.convert(carla.ColorConverter.CityScapesPalette)
        img = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        img = np.reshape(img, (image.height, image.width, 4))
        img = img[:, :, :3]
        img = img[:, :, ::-1]
        img = copy.deepcopy(img)
        self._data_wrapper.update_sensor(tag, img, image.frame)

    def _parse_lidar_cb(self, lidar_data: Any, tag: str) -> None:
        """
        parses lidar sensors
        """
        points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
        points = copy.deepcopy(points)
        points = np.reshape(points, (int(points.shape[0] / 3), 3))
        self._data_wrapper.update_sensor(tag, points, lidar_data.frame)

    def _parse_gnss_cb(self, gnss_data: Any, tag: str) -> None:
        """
        parses gnss sensors
        """
        array = np.array([gnss_data.latitude, gnss_data.longitude, gnss_data.altitude], dtype=np.float64)
        self._data_wrapper.update_sensor(tag, array, gnss_data.frame)


class CollisionSensor(object):
    """
    Carla sensor interface used to detect collision info in simulator. Once created,
    it will automatically update every tick.

    :Arguments:
        - parent_actor (carla.Actor): Actor to detect collision
        - col_threshold (float): Threshold value of collided impulse

    :Interfaces: clear
    """

    def __init__(self, parent_actor: carla.Actor, col_threshold: float) -> None:
        self.sensor = None
        self._history = deque(maxlen=500)
        self._parent = parent_actor
        self._threshold = col_threshold
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        print('If parent alive: ' + str(parent_actor.is_alive))
        parent = world.get_actor(parent_actor.id)

        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=parent_actor)
        # self.sensor = world.try_spawn_actor(bp, carla.Transform(), attach_to=parent_actor)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

        self.collided = False
        self.collided_frame = -1

    @staticmethod
    def _on_collision(weak_self, event: Any) -> None:
        self = weak_self()
        if not self:
            return
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self._history.append((event.frame, intensity))
        if intensity > self._threshold:
            self.collided = True
            self.collided_frame = event.frame

    def clear(self) -> None:
        """
        Clear collision sensor in Carla world.
        """
        self._history.clear()
        if self.sensor.is_alive:
            self.sensor.stop()
            self.sensor.destroy()


class TrafficLightHelper(object):
    """
    Interface of traffic light detector and recorder. It detects next traffic light state,
    calculates distance from hero vehicle to the end of this road, and if hero vehicle crosses
    this line when correlated light is red, it will record running a red light

    :Arguments:
        - hero_vehicle (carla.Actor): Hero vehicle

    :Interfaces:
        - tick
    """

    def __init__(self, hero_vehicle: carla.Actor, map: carla.Map, debug: bool = False) -> None:
        self._hero_vehicle = hero_vehicle
        self._world = self._hero_vehicle.get_world()
        self._map = map

        self._light_dis_thresh = 20
        self._active_light = None
        self._last_light = None

        self.total_lights_ran = 0
        self.total_lights = 0
        self.ran_light = False
        self.active_light_state = TrafficLightState.OFF
        self.active_light_dis = 200

        self._debug = debug

    def tick(self) -> None:
        """
        Tick one step. It will check the next traffic light and its state, update the count number
        of traffic light if needed. It will check the running light event by getting the last waypoints
        in current road and check if the vehicle has crossed them.
        """
        self.ran_light = False
        vehicle_transform = self._hero_vehicle.get_transform()
        vehicle_location = vehicle_transform.location

        self._active_light, light_trigger_location = self._get_active_light()

        if self._active_light is not None:
            if self._debug:
                self._world.debug.draw_point(light_trigger_location + carla.Location(z=2), size=0.1)
            self.active_light_state = {
                carla.TrafficLightState.Green: TrafficLightState.GREEN,
                carla.TrafficLightState.Yellow: TrafficLightState.YELLOW,
                carla.TrafficLightState.Red: TrafficLightState.RED,
                carla.TrafficLightState.Off: TrafficLightState.OFF,
            }[self._active_light.state]
            delta = vehicle_location - light_trigger_location
            distance = np.sqrt(sum([delta.x ** 2, delta.y ** 2, delta.z ** 2]))
            self.active_light_dis = min(200, distance)
            if self.active_light_dis < self._light_dis_thresh:
                if self._last_light is None or self._active_light.id != self._last_light.id:
                    self.total_lights += 1
                    self._last_light = self._active_light

        else:
            self.active_light_state = TrafficLightState.OFF
            self.active_light_dis = 200

        if self._last_light is not None:
            if self._last_light.state != carla.TrafficLightState.Red:
                return

            veh_extent = self._hero_vehicle.bounding_box.extent.x

            tail_close_pt = self._rotate_point(
                carla.Vector3D(-0.8 * veh_extent, 0.0, vehicle_location.z), vehicle_transform.rotation.yaw
            )
            tail_close_pt = vehicle_location + carla.Location(tail_close_pt)

            tail_far_pt = self._rotate_point(
                carla.Vector3D(-veh_extent - 1, 0.0, vehicle_location.z), vehicle_transform.rotation.yaw
            )
            tail_far_pt = vehicle_location + carla.Location(tail_far_pt)

            trigger_waypoints = self._get_traffic_light_trigger_waypoints(self._last_light)

            if self._debug:
                z = 2.1
                if self._last_light.state == carla.TrafficLightState.Red:
                    color = carla.Color(155, 0, 0)
                elif self._last_light.state == carla.TrafficLightState.Green:
                    color = carla.Color(0, 155, 0)
                else:
                    color = carla.Color(155, 155, 0)
                for wp in trigger_waypoints:
                    text = "{}.{}".format(wp.road_id, wp.lane_id)
                    self._world.debug.draw_string(wp.transform.location + carla.Location(x=1, z=z), text, color=color)
                    self._world.debug.draw_point(wp.transform.location + carla.Location(z=z), size=0.1, color=color)

            for wp in trigger_waypoints:
                tail_wp = self._map.get_waypoint(tail_far_pt)

                # Calculate the dot product (Might be unscaled, as only its sign is important)
                ve_dir = vehicle_transform.get_forward_vector()
                wp_dir = wp.transform.get_forward_vector()
                dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

                # Check the lane until all the "tail" has passed
                if tail_wp.road_id == wp.road_id and tail_wp.lane_id == wp.lane_id and dot_ve_wp > 0:
                    # This light is red and is affecting our lane
                    yaw_wp = wp.transform.rotation.yaw
                    lane_width = wp.lane_width
                    location_wp = wp.transform.location

                    lft_lane_wp = self._rotate_point(carla.Vector3D(0.4 * lane_width, 0.0, location_wp.z), yaw_wp + 90)
                    lft_lane_wp = location_wp + carla.Location(lft_lane_wp)
                    rgt_lane_wp = self._rotate_point(carla.Vector3D(0.4 * lane_width, 0.0, location_wp.z), yaw_wp - 90)
                    rgt_lane_wp = location_wp + carla.Location(rgt_lane_wp)

                    # Is the vehicle traversing the stop line?
                    if self._is_vehicle_crossing_line((tail_close_pt, tail_far_pt), (lft_lane_wp, rgt_lane_wp)):
                        self.ran_light = True
                        self.total_lights_ran += 1
                        self._last_light = None

    def get_trafficlight_trigger_location(self, traffic_light: carla.Actor):
        """
        Calculates the yaw of the waypoint that represents the trigger volume of the traffic light
        """

        def rotate_point(point, angle):
            """
            rotate a given point by a given angle
            """
            x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
            y_ = math.sin(math.radians(angle)) * point.x - math.cos(math.radians(angle)) * point.y

            return carla.Vector3D(x_, y_, point.z)

        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(traffic_light.trigger_volume.location)
        area_ext = traffic_light.trigger_volume.extent

        point = rotate_point(carla.Vector3D(0, 0, area_ext.z), base_rot)
        point_location = area_loc + carla.Location(x=point.x, y=point.y)

        return carla.Location(point_location.x, point_location.y, point_location.z)

    def _get_active_light(self) -> Tuple[Optional[carla.Actor], Optional[carla.Vector3D]]:
        lights_list = self._world.get_actors().filter("*traffic_light*")

        vehicle_transform = self._hero_vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_waypoint = self._map.get_waypoint(vehicle_location)

        for traffic_light in lights_list:
            object_location = self.get_trafficlight_trigger_location(traffic_light)
            object_waypoint = self._map.get_waypoint(object_location)

            if object_waypoint.road_id != vehicle_waypoint.road_id:
                continue

            ve_dir = vehicle_waypoint.transform.get_forward_vector()
            wp_dir = object_waypoint.transform.get_forward_vector()
            dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

            if dot_ve_wp < 0:
                continue
            while not object_waypoint.is_intersection:
                next_waypoint = object_waypoint.next(0.5)[0]
                if next_waypoint and not next_waypoint.is_intersection:
                    object_waypoint = next_waypoint
                else:
                    break

            return traffic_light, object_waypoint.transform.location

        return None, None

    def _get_traffic_light_trigger_waypoints(self, traffic_light: carla.Actor) -> List[carla.Waypoint]:
        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(traffic_light.trigger_volume.location)

        # Discretize the trigger box into points
        area_ext = traffic_light.trigger_volume.extent
        x_values = np.arange(-0.9 * area_ext.x, 0.9 * area_ext.x, 1.0)  # 0.9 to avoid crossing to adjacent lanes

        area = []
        for x in x_values:
            point = self._rotate_point(carla.Vector3D(x, 0, area_ext.z), base_rot)
            point_location = area_loc + carla.Location(x=point.x, y=point.y)
            area.append(point_location)

        # Get the waypoints of these points, removing duplicates
        ini_wps = []
        for pt in area:
            wpx = self._map.get_waypoint(pt)
            # As x_values are arranged in order, only the last one has to be checked
            if not ini_wps or ini_wps[-1].road_id != wpx.road_id or ini_wps[-1].lane_id != wpx.lane_id:
                ini_wps.append(wpx)

        # Advance them until the intersection
        wps = []
        for wpx in ini_wps:
            while not wpx.is_intersection:
                next_wp = wpx.next(0.5)[0]
                if next_wp and not next_wp.is_intersection:
                    wpx = next_wp
                else:
                    break
            wps.append(wpx)

        return wps

    def _is_vehicle_crossing_line(self, seg1: List, seg2: List) -> bool:
        """
        check if vehicle crosses a line segment
        """
        line1 = shapely.geometry.LineString([(seg1[0].x, seg1[0].y), (seg1[1].x, seg1[1].y)])
        line2 = shapely.geometry.LineString([(seg2[0].x, seg2[0].y), (seg2[1].x, seg2[1].y)])
        inter = line1.intersection(line2)

        return not inter.is_empty

    def _rotate_point(self, point: carla.Vector3D, angle: float) -> carla.Vector3D:
        """
        rotate a given point by a given angle
        """
        x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
        y_ = math.sin(math.radians(angle)) * point.x + math.cos(math.radians(angle)) * point.y
        return carla.Vector3D(x_, y_, point.z)
