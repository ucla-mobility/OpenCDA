# -*- coding: utf-8 -*-
"""
Perception module
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import weakref
import sys
import time

import carla
import cv2
import numpy as np
import open3d as o3d

import opencda.core.sensing.perception.sensor_transformation as st
from opencda.core.common.misc import cal_distance_angle, get_speed
from opencda.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from opencda.core.sensing.perception.o3d_lidar_libs import o3d_visualizer_init, \
    o3d_pointcloud_encode, o3d_visualizer_show, o3d_camera_lidar_fusion


class CameraSensor(object):
    """
    Class for rgb camera.

    Parameters
    -vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn sensors.
    -position : string
        Indicates the sensor is a front or rear camera. option: front, left, right.

    Attributes
    -image : np.ndarray
        Current received image.
    -sensor : CARLA actor
        The current sensor actors that will be attach to the vehicles.
    """

    def __init__(self, vehicle, position='front'):
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('fov', '100')

        if position == 'front':
            spawn_point = carla.Transform(carla.Location(x=2.5, y=0.0, z=1.0),
                                          carla.Rotation(pitch=0, roll=0, yaw=0))
        elif position == 'right':
            spawn_point = carla.Transform(carla.Location(x=0.0, y=0.3, z=1.8),
                                          carla.Rotation(pitch=0, roll=0, yaw=100))
        else:
            spawn_point = carla.Transform(carla.Location(x=0.0, y=-0.3, z=1.8),
                                          carla.Rotation(pitch=0, roll=0, yaw=-100))

        self.sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)

        self.image = None
        self.timstamp = None
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CameraSensor._on_rgb_image_event(weak_self, event))

        # camera attributes
        self.image_width = int(self.sensor.attributes['image_size_x'])
        self.image_height = int(self.sensor.attributes['image_size_y'])

    @staticmethod
    def _on_rgb_image_event(weak_self, event):
        """CAMERA  method"""
        self = weak_self()
        if not self:
            return
        image = np.array(event.raw_data)
        image = image.reshape((self.image_height, self.image_width, 4))
        # we need to remove the alpha channel
        image = image[:, :, :3]

        self.image = image
        self.timestamp = event.timestamp


class LidarSensor(object):
    """
    Lidar sensor manager.

    Parameters
    -vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn sensors.
    -config_yaml : dict
        Configuration for lidar sensor.

    Attributes
    -o3d_pointcloud : o3d.PointCloud
        Recieved point cloud saved in o3d.PointCloud format.
    -sensor : CARLA actor
        The current sensor actors that will be attach to the vehicles.
    """

    def __init__(self, vehicle, config_yaml):
        """
        Construct class.
        Args:
            vehicle (carla.Vehicle): The attached vehicle.
            config_yaml (dict): Configuration for lidar.
        """
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast')

        # set attribute based on the configuration
        blueprint.set_attribute('upper_fov', str(config_yaml['upper_fov']))
        blueprint.set_attribute('lower_fov', str(config_yaml['lower_fov']))
        blueprint.set_attribute('channels', str(config_yaml['channels']))
        blueprint.set_attribute('range', str(config_yaml['range']))
        blueprint.set_attribute('points_per_second', str(config_yaml['points_per_second']))
        blueprint.set_attribute('rotation_frequency', str(config_yaml['rotation_frequency']))
        blueprint.set_attribute('dropoff_general_rate', str(config_yaml['dropoff_general_rate']))
        blueprint.set_attribute('dropoff_intensity_limit', str(config_yaml['dropoff_intensity_limit']))
        blueprint.set_attribute('dropoff_zero_intensity', str(config_yaml['dropoff_zero_intensity']))
        blueprint.set_attribute('noise_stddev', str(config_yaml['noise_stddev']))

        # spawn sensor on vehicle
        spawn_point = carla.Transform(carla.Location(x=-0.5, z=1.8))
        self.sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)

        # lidar data
        self.data = None
        self.timestamp = None
        # open3d point cloud object
        self.o3d_pointcloud = o3d.geometry.PointCloud()

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LidarSensor._on_data_event(weak_self, event))

    @staticmethod
    def _on_data_event(weak_self, event):
        """CAMERA  method"""
        self = weak_self()
        if not self:
            return

        # retrieve the raw lidar data and reshape to (N, 4)
        data = np.copy(np.frombuffer(event.raw_data, dtype=np.dtype('f4')))
        # (x, y, z, intensity)
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        self.data = data
        self.timestamp = event.timestamp


class PerceptionManager(object):
    """
    Default perception module. Currenly only used to detect vehicles.

    Parameters
    -vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn sensors.
    -config_yaml : dict
        Configuration for perception.
    - ml_manager: opencda object
        Ml manager includes all loaded trained perception models.

    Attributes
    -ml_manager : opencda object
        weak reference of the ML Manager.
    -activate : bool
        Whether perception algorithms are activated. If not, load object
        coordinates directly from the server.
    -lidar: opencda object
        Lidar sensor manager.
    -rgb_camera: opencda object
        Camera manager.
    """

    def __init__(self, vehicle, config_yaml, ml_manager):
        """
        Construct class.
        Args:
            vehicle (carla.Actor): The carla vehicle.
            config_yaml (dict):  The configuration yaml dictionary.
            ml_manager(MlManager): Machine learning manager from CAV World.
        """
        self.vehicle = vehicle

        self.activate = config_yaml['activate']
        self.camera_visualize = config_yaml['camera_visualize']
        self.camera_num = min(config_yaml['camera_num'], 3)
        self.lidar_visualize = config_yaml['lidar_visualize']

        if self.activate and not ml_manager:
            sys.exit('If you activate the perception module, then apply_ml must be set to true in'
                     'the argument parser to load the detection DL model.')
        self.ml_manager = ml_manager

        # we only spawn the camera when perception module is activated or camera visualization is needed
        if self.activate or self.camera_visualize:
            self.rgb_camera = []
            mount_position = ['front', 'right', 'left']
            for i in range(self.camera_num):
                self.rgb_camera.append(CameraSensor(vehicle, mount_position[i]))

        else:
            self.rgb_camera = None

        # we only spawn the camera when perception module is activated or lidar visualization is needed
        if self.activate or self.lidar_visualize:
            self.lidar = LidarSensor(vehicle, config_yaml['lidar'])
            self.o3d_vis = o3d_visualizer_init(vehicle.id)
        else:
            self.lidar = None
            self.o3d_vis = None

        # count how many steps have been passed
        self.count = 0

        self.ego_pos = None

    def dist(self, v):
        """
        A fast method to retrieve the obstable distance the ego vehicle from the server directly.
        Args:
            v (carla.vehicle):

        Returns:
            float: distance

        """
        return v.get_location().distance(self.ego_pos.location)

    def detect(self, ego_pos):
        """
        Detect surrounding objects. Currently only vehicle detection supported.
        Args:
            ego_pos (carla.Transform): Vehicle ego position

        Returns:
            List of carla.Vehicle or ObstacleVehicle
        """
        self.ego_pos = ego_pos

        objects = {'vehicles': []}

        if not self.activate:
            objects = self.deactivate_mode(objects)

        else:
            objects = self.activate_mode(objects)

        self.count += 1

        return objects

    def activate_mode(self, objects):
        """
        Use Yolov5 + Lidar fusion to detect objects.
        Args:
            objects(dict): object dictionary

        Returns:
            objects: dict
                The updated object dictionary.
        """
        # retrieve current cameras and lidar data
        rgb_images = []
        for rgb_camera in self.rgb_camera:
            time.sleep(0.001)
            rgb_images.append(cv2.cvtColor(np.array(rgb_camera.image), cv2.COLOR_BGR2RGB))

        # yolo detection
        yolo_detection = self.ml_manager.object_detector(rgb_images)
        # rgb_images for drawing
        rgb_draw_images = []

        for (i, rgb_camera) in enumerate(self.rgb_camera):
            # lidar projection
            rgb_image, projected_lidar = st.project_lidar_to_camera(self.lidar.sensor, rgb_camera.sensor,
                                                                    self.lidar.data, np.array(rgb_camera.image))
            rgb_draw_images.append(rgb_image)

            # camera lidar fusion
            objects = o3d_camera_lidar_fusion(objects, yolo_detection.xyxy[i],
                                              self.lidar.data, projected_lidar, self.lidar.sensor)

            # calculate the speed. current we retrieve from the server directly.
            self.speed_retrieve(objects)

        if self.camera_visualize:
            names = ['front', 'right', 'left']
            for (i, rgb_image) in enumerate(rgb_draw_images):
                if i > self.camera_num or i > self.camera_visualize - 1:
                    break
                rgb_image = self.ml_manager.draw_2d_box(yolo_detection, rgb_image, i)
                rgb_image = cv2.resize(rgb_image, (0, 0), fx=0.4, fy=0.4)
                cv2.imshow('%s camera of actor %d, perception activated' % (names[i], self.vehicle.id), rgb_image)
            cv2.waitKey(1)

        if self.lidar_visualize:
            o3d_pointcloud_encode(self.lidar.data, self.lidar.o3d_pointcloud)
            o3d_visualizer_show(self.o3d_vis, self.count, self.lidar.o3d_pointcloud, objects)

        return objects

    def deactivate_mode(self, objects):
        """
        Obstacle detection under perception deactivation mode.
        Args:
            objects(dict): object dictionary
        Returns:

        """
        world = self.vehicle.get_world()

        vehicle_list = world.get_actors().filter("*vehicle*")
        vehicle_list = [v for v in vehicle_list if self.dist(v) < 50 and
                        v.id != self.vehicle.id]

        # convert carla.Vehicle to opencda.ObstacleVehicle if lidar visualization is required.
        if self.lidar:
            vehicle_list = [ObstacleVehicle(None, None, v, self.lidar.sensor) for v in vehicle_list]
        objects.update({'vehicles': vehicle_list})

        if self.camera_visualize:
            time.sleep(0.001)
            # we only visualiz the frontal camera
            rgb_image = np.array(self.rgb_camera[0].image)
            # draw the ground truth bbx on the camera image
            rgb_image = self.visualize_3d_bbx_front_camera(objects, rgb_image)
            # resize to make it fittable to the screen
            rgb_image = cv2.resize(rgb_image, (0, 0), fx=0.4, fy=0.4)

            # show image using cv2
            cv2.imshow('front camera of actor %d, perception deactivated' % self.vehicle.id, rgb_image)
            cv2.waitKey(1)

        if self.lidar_visualize:
            o3d_pointcloud_encode(self.lidar.data, self.lidar.o3d_pointcloud)
            # render the raw lidar
            o3d_visualizer_show(self.o3d_vis, self.count, self.lidar.o3d_pointcloud, objects)

        return objects

    def visualize_3d_bbx_front_camera(self, objects, rgb_image):
        """
        Visualize the 3d bounding box on frontal camera image.
        Args:
            objects (dict): a dictionary containing all detected objects.
            rgb_image (np.ndarray):camera image.

        Returns:

        """
        for v in objects['vehicles']:
            # we only draw the bounding box in the fov of camera
            _, angle = cal_distance_angle(v.get_location(), self.ego_pos.location, self.ego_pos.rotation.yaw)
            if angle < 30:
                bbx_camera = st.get_2d_bb(v, self.rgb_camera[0].sensor, self.rgb_camera[0].sensor.get_transform())
                cv2.rectangle(rgb_image, (int(bbx_camera[0, 0]), int(bbx_camera[0, 1])),
                              (int(bbx_camera[1, 0]), int(bbx_camera[1, 1])), (255, 0, 0), 2)

        return rgb_image

    def speed_retrieve(self, objects):
        """
        We don't implement any obstacle speed calculation algorithm. The speed will be retrieved from
        the server directly.
        Args:
            objects(dict): The dictionary contains the objects.

        Returns:

        """
        if 'vehicles' not in objects:
            return

        world = self.vehicle.get_world()
        vehicle_list = world.get_actors().filter("*vehicle*")
        vehicle_list = [v for v in vehicle_list if self.dist(v) < 50 and
                        v.id != self.vehicle.id]

        for v in vehicle_list:
            loc = v.get_location()
            for obstacle_vehicle in objects['vehicles']:
                obstacle_speed = get_speed(obstacle_vehicle)
                if obstacle_speed > 0:
                    continue
                obstacle_loc = obstacle_vehicle.get_location()
                if abs(loc.x - obstacle_loc.x) <= 3.0 and abs(loc.y - obstacle_loc.y) <= 3.0:
                    obstacle_vehicle.set_velocity(v.get_velocity())

    def destroy(self):
        """
        Destroy sensors.
        Returns:

        """
        if self.rgb_camera:
            for rgb_camera in self.rgb_camera:
                rgb_camera.sensor.destroy()
        if self.lidar:
            self.lidar.sensor.destroy()
        if self.camera_visualize:
            cv2.destroyAllWindows()
        if self.lidar_visualize:
            self.o3d_vis.destroy_window()
