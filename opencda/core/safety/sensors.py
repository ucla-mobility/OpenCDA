"""
Sensors related to safety status check
"""
import math
import numpy as np
import carla
import weakref
import shapely
from collections import deque
from typing import List


class CollisionSensor(object):
    """
    Collision detection sensor.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle, this is for cav.
    params : dict
        The dictionary containing sensor configurations.

    Attributes
    ----------
    image : np.ndarray
        Current received rgb image.
    sensor : carla.sensor
        The carla sensor that mounts at the vehicle.
    """

    def __init__(self, vehicle, params):
        world = vehicle.get_world()

        blueprint = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(),
                                        attach_to=vehicle)

        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: CollisionSensor._on_collision(weak_self, event))

        self.collided = False
        self.collided_frame = -1
        self._history = deque(maxlen=params['history_size'])
        self._threshold = params['col_thresh']

    @staticmethod
    def _on_collision(weak_self, event) -> None:
        self = weak_self()
        if not self:
            return
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self._history.append((event.frame, intensity))
        if intensity > self._threshold:
            self.collided = True
            self.collided_frame = event.frame

    def return_status(self):
        return {'collision': self.collided}

    def tick(self, data_dict):
        pass

    def destroy(self) -> None:
        """
        Clear collision sensor in Carla world.
        """
        self._history.clear()
        if self.sensor.is_alive:
            self.sensor.stop()
            self.sensor.destroy()


class StuckDetector(object):
    """
    Stuck detector used to detect vehicle stuck in simulator.
    It takes speed as input in each tick.

    Parameters
    ----------
    params : dict
        The dictionary containing sensor configurations.
    """

    def __init__(self, params):
        self._speed_queue = deque(maxlen=params['len_thresh'])
        self._len_thresh = params['len_thresh']
        self._speed_thresh = params['speed_thresh']

        self.stuck = False

    def tick(self, data_dict) -> None:
        """
        Update one tick

        Parameters
        ----------
        data_dict : dict
            The data dictionary provided by the upsteam modules.
        """
        speed = data_dict['ego_speed']
        self._speed_queue.append(speed)
        if len(self._speed_queue) >= self._len_thresh:
            if np.average(self._speed_queue) < self._speed_thresh:
                self.stuck = True
                return
        self.stuck = False

    def return_status(self):
        return {'stuck': self.stuck}

    def destroy(self):
        """
        Clear speed history
        """
        self._speed_queue.clear()


class OffRoadDetector(object):
    """
    A detector to monitor whether

    Parameters
    ----------
    params : dict
        The dictionary containing sensor configurations.
    """

    def __init__(self, params):
        self.off_road = False

    def tick(self, data_dict) -> None:
        """
        Update one tick

        Parameters
        ----------
        data_dict : dict
            The data dictionary provided by the upsteam modules.
        """
        # static bev map that indicate where is the road
        static_map = data_dict['static_bev']
        if static_map is None:
            return
        h, w = static_map.shape[0], static_map.shape[1]
        # the ego is always at the center of the bev map. If the pixel is
        # black, that means the vehicle is off road.
        if np.mean(static_map[h // 2, w // 2]) == 255:
            self.off_road = True
        else:
            self.off_road = False

    def return_status(self):
        return {'offroad': self.off_road}

    def destroy(self):
        pass


class TrafficLightDector(object):
    """
    Interface of traffic light detector and recorder. It detects next traffic light state,
    calculates distance from hero vehicle to the end of this road, and if hero vehicle crosses
    this line when correlated light is red, it will record running a red light
    """

    def __init__(self, params, vehicle):
        self.ran_light = False
        self._map = None
        self.veh_extent = vehicle.bounding_box.extent.x

        self._light_dis_thresh = params['light_dist_thresh']
        self._active_light = None
        self._last_light = None

        self.total_lights_ran = 0
        self.total_lights = 0
        self.ran_light = False
        self.active_light_state = carla.TrafficLightState.Off
        self.active_light_dis = 200

    def tick(self, data_dict):
        # Reset the "ran light" flag
        self.ran_light = False

        # Extract the active traffic lights, vehicle transform, world, and map from data_dict
        active_lights = data_dict['objects']['traffic_lights']
        vehicle_transform = data_dict['ego_pos']
        world = data_dict['world']
        self._map = data_dict['carla_map']

        # Get the location of the first active traffic light
        self._active_light = active_lights[0] if len(active_lights) > 0 \
            else None
        vehicle_location = vehicle_transform.location

        # If there is an active traffic light,
        # compute the distance between the vehicle and the traffic light
        if self._active_light is not None:
            light_trigger_location = self._active_light.get_location()
            self.active_light_state = self._active_light.get_state()
            delta = vehicle_location - light_trigger_location
            distance = np.sqrt(sum([delta.x ** 2, delta.y ** 2, delta.z ** 2]))

            # Set the active light distance to the minimum of the
            # computed distance and a maximum threshold
            self.active_light_dis = min(200, distance)

            # If the vehicle is close enough to the traffic light,
            # and the traffic light has changed since the last tick,
            # increment the total number of traffic lights seen and set the
            # last light to the current light
            if self.active_light_dis < self._light_dis_thresh:
                if self._last_light is None or self._active_light.actor.id != self._last_light.id:
                    self.total_lights += 1
                    self._last_light = self._active_light.actor
        else:
            # If there is no active traffic light, set the active light state
            # to "Off" and set the active light distance to a default value
            self.active_light_state = carla.TrafficLightState.Off
            self.active_light_dis = 200

            # If there is a last light (i.e., a traffic light that was active
            # in the previous tick), check if it is currently red
        if self._last_light is not None:
            if self._last_light.state != carla.TrafficLightState.Red:
                return

            # Compute the endpoints of a line segment representing the
            # vehicle's position and direction
            veh_extent = self.veh_extent
            tail_close_pt = self._rotate_point(
                carla.Vector3D(-0.8 * veh_extent, 0.0, vehicle_location.z),
                vehicle_transform.rotation.yaw
            )
            tail_close_pt = vehicle_location + carla.Location(tail_close_pt)
            tail_far_pt = self._rotate_point(
                carla.Vector3D(-veh_extent - 1, 0.0, vehicle_location.z),
                vehicle_transform.rotation.yaw
            )
            tail_far_pt = vehicle_location + carla.Location(tail_far_pt)

            # Get the trigger waypoints for the last traffic light
            trigger_waypoints = self._get_traffic_light_trigger_waypoints(
                self._last_light)

            # For each trigger waypoint,
            # check if the vehicle has crossed the stop line
            for wp in trigger_waypoints:
                tail_wp = self._map.get_waypoint(tail_far_pt)

                # Calculate the dot product (Might be unscaled,
                # as only its sign is important)
                ve_dir = vehicle_transform.get_forward_vector()
                wp_dir = wp.transform.get_forward_vector()
                dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

                # Check the lane until all the "tail" has passed
                if tail_wp.road_id == wp.road_id and tail_wp.lane_id == wp.lane_id and dot_ve_wp > 0:
                    # This light is red and is affecting our lane
                    yaw_wp = wp.transform.rotation.yaw
                    lane_width = wp.lane_width
                    location_wp = wp.transform.location

                    lft_lane_wp = self._rotate_point(
                        carla.Vector3D(0.4 * lane_width, 0.0, location_wp.z),
                        yaw_wp + 90)
                    lft_lane_wp = location_wp + carla.Location(lft_lane_wp)
                    rgt_lane_wp = self._rotate_point(
                        carla.Vector3D(0.4 * lane_width, 0.0, location_wp.z),
                        yaw_wp - 90)
                    rgt_lane_wp = location_wp + carla.Location(rgt_lane_wp)

                    # Is the vehicle traversing the stop line?
                    if self._is_vehicle_crossing_line(
                            (tail_close_pt, tail_far_pt),
                            (lft_lane_wp, rgt_lane_wp)):
                        self.ran_light = True
                        self.total_lights_ran += 1
                        self._last_light = None

    def _is_vehicle_crossing_line(self, seg1: List, seg2: List) -> bool:
        """
        check if vehicle crosses a line segment
        """
        line1 = shapely.geometry.LineString(
            [(seg1[0].x, seg1[0].y), (seg1[1].x, seg1[1].y)])
        line2 = shapely.geometry.LineString(
            [(seg2[0].x, seg2[0].y), (seg2[1].x, seg2[1].y)])
        inter = line1.intersection(line2)

        return not inter.is_empty

    def _rotate_point(self, point: carla.Vector3D,
                      angle: float) -> carla.Vector3D:
        """
        rotate a given point by a given angle
        """
        x_ = math.cos(math.radians(angle)) * point.x - math.sin(
            math.radians(angle)) * point.y
        y_ = math.sin(math.radians(angle)) * point.x + math.cos(
            math.radians(angle)) * point.y
        return carla.Vector3D(x_, y_, point.z)

    def _get_traffic_light_trigger_waypoints(self,
                                             traffic_light: carla.Actor) -> \
            List[carla.Waypoint]:
        # Get the transform information for the traffic light
        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(
            traffic_light.trigger_volume.location)

        # Get the extent of the trigger volume
        area_ext = traffic_light.trigger_volume.extent
        # Discretize the trigger box into points along the x-axis
        x_values = np.arange(-0.9 * area_ext.x, 0.9 * area_ext.x,
                             1.0)  # 0.9 to avoid crossing to adjacent lanes

        # Create a list of discretized points
        area = []
        for x in x_values:
            point = self._rotate_point(carla.Vector3D(x, 0, area_ext.z),
                                       base_rot)
            point_location = area_loc + carla.Location(x=point.x, y=point.y)
            area.append(point_location)

        # Get the waypoints of these points, removing duplicates
        ini_wps = []
        for pt in area:
            wpx = self._map.get_waypoint(pt)
            # As x_values are arranged in order, only the last one has to be checked
            if not ini_wps or ini_wps[-1].road_id != wpx.road_id or ini_wps[
                -1].lane_id != wpx.lane_id:
                ini_wps.append(wpx)

        # Advance the waypoints until the intersection
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

    def return_status(self):
        return {'ran_light': self.ran_light}
