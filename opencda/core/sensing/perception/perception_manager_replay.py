# -*- coding: utf-8 -*-
"""
Perception module base.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import weakref
import sys
import time

import carla
import cv2
import numpy as np
import open3d as o3d
import math

import opencda.core.sensing.perception.sensor_transformation as st
from opencda.core.common.misc import \
    cal_distance_angle, get_speed, get_speed_sumo
from opencda.core.sensing.perception.obstacle_vehicle import \
    ObstacleVehicle
from opencda.core.sensing.perception.static_obstacle import TrafficLight
from opencda.core.sensing.perception.o3d_lidar_libs import \
    o3d_visualizer_init, o3d_pointcloud_encode, o3d_visualizer_show, \
    o3d_camera_lidar_fusion
from opencda.core.sensing.perception.perception_manager import CameraSensor, LidarSensor, SemanticLidarSensor, PerceptionManager

class CameraSensorReplay(CameraSensor):

    @staticmethod
    def spawn_point_estimation(relative_position, global_position):

        pitch = 0
        carla_location = carla.Location(x=0, y=0, z=0)
        x, y, z, yaw = relative_position

        # this is for rsu. It utilizes global position instead of relative
        # position to the vehicle
        if global_position is not None:
            carla_location = carla.Location(
                x=global_position[0],
                y=global_position[1],
                z=global_position[2])
            pitch = global_position[3]
            yaw = global_position[4]
            roll = global_position[5]

        carla_location = carla.Location(x=carla_location.x + x,
                                        y=carla_location.y + y,
                                        z=carla_location.z + z)

        carla_rotation = carla.Rotation(roll=0, yaw=yaw, pitch=pitch)
        spawn_point = carla.Transform(carla_location, carla_rotation)

        return spawn_point

class LidarSensorReplay(LidarSensor):

    def __init__(self, vehicle, world, config_yaml, global_position):

        blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast')

        if vehicle is not None:
            world = vehicle.get_world()
            blueprint.set_attribute('channels', str(config_yaml['channels']))
            blueprint.set_attribute('dropoff_general_rate', str(config_yaml['dropoff_general_rate']))
            blueprint.set_attribute('dropoff_intensity_limit', str(config_yaml['dropoff_intensity_limit']))
            blueprint.set_attribute('dropoff_zero_intensity', str(config_yaml['dropoff_zero_intensity']))
            blueprint.set_attribute('upper_fov', str(config_yaml['upper_fov']))
            blueprint.set_attribute('lower_fov', str(config_yaml['lower_fov']))
            blueprint.set_attribute('noise_stddev', str(config_yaml['noise_stddev']))
            blueprint.set_attribute('points_per_second', str(config_yaml['points_per_second']))
            blueprint.set_attribute('range', str(config_yaml['range']))
        else:
            # set attribute based on the configuration
            blueprint.set_attribute('lidar_type', str(config_yaml['lidar_type'])) ###
            blueprint.set_attribute('name', str(config_yaml['name']))
        
        blueprint.set_attribute('rotation_frequency', str(config_yaml['rotation_frequency']))

        # spawn sensor
        if global_position is None:
            spawn_point = carla.Transform(carla.Location(x=-0.5, z=1.9))
        else:
            if str(config_yaml['lidar_type']) == 'Risley_prism':
                rotation = carla.Rotation(
                                    pitch = global_position[3],
                                    yaw = global_position[4],
                                    roll = global_position[5]
                                    )
            else:
                rotation = carla.Rotation(
                                    pitch=0,
                                    yaw=0,
                                    roll=0
                                    )
            
            spawn_point = carla.Transform(carla.Location(x=global_position[0],
                                                         y=global_position[1],
                                                         z=global_position[2]),
                                          rotation)
        self.spawn_point = spawn_point
        self.sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)

        # lidar data
        self.data = None
        self.timestamp = None
        self.frame = 0
        # open3d point cloud object
        self.o3d_pointcloud = o3d.geometry.PointCloud()
        self.spawn_point = spawn_point

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: LidarSensor._on_data_event(
                weak_self, event))

class SemanticLidarSensorReplay(SemanticLidarSensor):

    def __init__(self, vehicle, world, config_yaml, global_position):
        if vehicle is not None:
            world = vehicle.get_world()

        blueprint = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')

        # set attribute based on the configuration
        if 'lidar_type' not in config_yaml:
            blueprint.set_attribute('lidar_type', 'default') ###
            blueprint.set_attribute('range', str(config_yaml['range']))
        else:
            blueprint.set_attribute('lidar_type', config_yaml['lidar_type']) ###
            blueprint.set_attribute('name', config_yaml['name'])

        blueprint.set_attribute('rotation_frequency', str(config_yaml['rotation_frequency']))

        # spawn sensor
        if global_position is None:
            spawn_point = carla.Transform(carla.Location(x=-0.5, z=1.9))
        else:
            if 'lidar_type' in config_yaml and str(config_yaml['lidar_type']) == 'Risley_prism':
                rotation = carla.Rotation(
                                    pitch = global_position[3],
                                    yaw = global_position[4],
                                    roll = global_position[5]
                                    )
            else:
                rotation = carla.Rotation(
                                    pitch=0,
                                    yaw=0,
                                    roll=0
                                    )
            
            spawn_point = carla.Transform(carla.Location(x=global_position[0],
                                                            y=global_position[1],
                                                            z=global_position[2]),
                                                rotation)

        if vehicle is not None:
            self.sensor = world.spawn_actor(
                blueprint, spawn_point, attach_to=vehicle)
        else:
            self.sensor = world.spawn_actor(blueprint, spawn_point)

        # lidar data
        self.points = None
        self.obj_idx = None
        self.obj_tag = None

        self.timestamp = None
        self.frame = 0
        # open3d point cloud object
        self.o3d_pointcloud = o3d.geometry.PointCloud()

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: self._on_data_event(
                weak_self, event))

    # @staticmethod
    # def _on_data_event(weak_self, event):
    #     """Semantic Lidar  method"""
    #     self = weak_self()

    #     time.sleep(5)
    #     if not self:
    #         return

    #     # shape:(n, 6)
    #     self.event = event
    #     data = np.frombuffer(event.raw_data, dtype=np.dtype([
    #         ('x', np.float32), ('y', np.float32), ('z', np.float32),
    #         ('CosAngle', np.float32), ('ObjIdx', np.uint32),
    #         ('ObjTag', np.uint32)]))

    #     # (x, y, z, intensity)
    #     self.points = np.array([data['x'], data['y'], data['z']]).T
    #     self.obj_tag = np.array(data['ObjTag'])
    #     self.obj_idx = np.array(data['ObjIdx'])

    #     self.data = data
    #     self.frame = event.frame
    #     self.timestamp = event.timestamp
        
class PerceptionManagerReplay(PerceptionManager):

    def __init__(self, vehicle, config_yaml, cav_world,
                 data_dump=False, carla_world=None, infra_id=None):
        self.vehicle = vehicle
        self.carla_world = carla_world if carla_world is not None \
            else self.vehicle.get_world()
        self._map = self.carla_world.get_map()
        self.id = infra_id if infra_id is not None else vehicle.id
        # self.id = infra_id if infra_id is not None else int(vehicle.attributes['role_name'])

        self.activate = config_yaml['activate']
        self.camera_visualize = config_yaml['camera']['visualize']
        self.camera_num = config_yaml['camera']['num']
        self.lidar_visualize = config_yaml['lidar']['visualize']
        self.global_position = config_yaml['global_position'] if 'global_position' in config_yaml else None

        self.cav_world = weakref.ref(cav_world)()
        ml_manager = cav_world.ml_manager

        if self.activate and data_dump:
            sys.exit("When you dump data, please deactivate the "
                     "detection function for precise label.")

        if self.activate and not ml_manager:
            sys.exit(
                'If you activate the perception module, '
                'then apply_ml must be set to true in'
                'the argument parser to load the detection DL model.')
        self.ml_manager = ml_manager

        # we only spawn the camera when perception module is activated or
        # camera visualization is needed
        if self.activate or self.camera_visualize:
            self.rgb_camera = []
            mount_position = config_yaml['camera']['positions']
            assert len(mount_position) == self.camera_num, \
                "The camera number has to be the same as the length of the" \
                "relative positions list"

            for i in range(self.camera_num):
                self.rgb_camera.append(
                    CameraSensorReplay(
                        vehicle, self.carla_world, mount_position[i],
                        self.global_position))

        else:
            self.rgb_camera = None

        # we only spawn the LiDAR when perception module is activated or lidar
        # visualization is needed
        if self.activate or self.lidar_visualize:
            self.lidar = LidarSensorReplay(vehicle,
                                    self.carla_world,
                                    config_yaml['lidar'],
                                    self.global_position)
            self.o3d_vis = o3d_visualizer_init(self.id)
        else:
            self.lidar = None
            self.o3d_vis = None

        # if data dump is true, semantic lidar is also spawned
        self.data_dump = data_dump
        if data_dump:
            self.semantic_lidar = SemanticLidarSensorReplay(vehicle,
                                                      self.carla_world,
                                                    #   config_yaml['lidar'],
                                                      config_yaml['semanticlidar'],###
                                                      self.global_position)

        # count how many steps have been passed
        self.count = 0
        # ego position
        self.ego_pos = None

        # the dictionary contains all objects
        self.objects = {}
        # traffic light detection related
        self.traffic_thresh = config_yaml['traffic_light_thresh'] \
            if 'traffic_light_thresh' in config_yaml else 50

    def deactivate_mode(self, objects):
        """
        Object detection using server information directly.

        Parameters
        ----------
        objects : dict
            The dictionary that contains all category of detected objects.
            The key is the object category name and value is its 3d coordinates
            and confidence.

        Returns
        -------
         objects: dict
            Updated object dictionary.
        """
        world = self.carla_world

        vehicle_list = world.get_actors().filter("*vehicle*")

        # !!! remarks: filter_vehicle_out_sensor() turned off for rsu lidars 
        #              with fixed locations otherwise recording replay simulation 
        #              will have missing vehicles and fail exact replication

        # use semantic lidar to filter out vehicles out of the range
        # if self.data_dump:
            # vehicle_list = self.filter_vehicle_out_sensor(vehicle_list)

        # convert carla.Vehicle to opencda.ObstacleVehicle if lidar
        # visualization is required.
        if self.lidar:
            vehicle_list = [
                ObstacleVehicle(
                    None,
                    None,
                    v,
                    self.lidar.sensor,
                    self.cav_world.sumo2carla_ids) for v in vehicle_list]
        else:
            vehicle_list = [
                ObstacleVehicle(
                    None,
                    None,
                    v,
                    None,
                    self.cav_world.sumo2carla_ids) for v in vehicle_list]

        objects.update({'vehicles': vehicle_list})

        if self.camera_visualize:
            while self.rgb_camera[0].image is None:
                continue

            # names = ['front', 'right', 'left', 'back']
            names = ['front', 'back']

            for (i, rgb_camera) in enumerate(self.rgb_camera):
                if i > self.camera_num - 1 or i > self.camera_visualize - 1:
                    break
                # we only visualiz the frontal camera
                rgb_image = np.array(rgb_camera.image)
                # draw the ground truth bbx on the camera image
                rgb_image = self.visualize_3d_bbx_front_camera(objects,
                                                               rgb_image,
                                                               i)
                # resize to make it fittable to the screen
                rgb_image = cv2.resize(rgb_image, (0, 0), fx=0.4, fy=0.4)

                # show image using cv2
                cv2.imshow(
                    '%s camera of actor %d, perception deactivated' %
                    (names[i], self.id), rgb_image)
                cv2.waitKey(1)

        if self.lidar_visualize:
            while self.lidar.data is None:
                continue
            o3d_pointcloud_encode(self.lidar.data, self.lidar.o3d_pointcloud)
            # render the raw lidar
            o3d_visualizer_show(
                self.o3d_vis,
                self.count,
                self.lidar.o3d_pointcloud,
                objects)

        # add traffic light
        objects = self.retrieve_traffic_lights(objects)
        self.objects = objects

        return objects

class PerceptionManagerReplayBEV(PerceptionManagerReplay):

    def __init__(self, vehicle, config_yaml, cav_world,
                 data_dump=False, carla_world=None, infra_id=None):
        self.vehicle = vehicle
        self.carla_world = carla_world if carla_world is not None \
            else self.vehicle.get_world()
        self._map = self.carla_world.get_map()
        self.id = infra_id if infra_id is not None else vehicle.id
        # self.id = infra_id if infra_id is not None else int(vehicle.attributes['role_name'])

        self.activate = config_yaml['activate']
        self.camera_visualize = config_yaml['camera']['visualize']
        self.camera_num = config_yaml['camera']['num']
        self.lidar_visualize = config_yaml['lidar']['visualize']
        self.global_position = config_yaml['global_position'] if 'global_position' in config_yaml else None

        self.cav_world = weakref.ref(cav_world)()

        if self.activate and data_dump:
            sys.exit("When you dump data, please deactivate the "
                     "detection function for precise label.")

        if self.activate or self.camera_visualize:
            self.rgb_camera = []
            mount_position = config_yaml['camera']['positions']
            assert len(mount_position) == self.camera_num, \
                "The camera number has to be the same as the length of the" \
                "relative positions list"

            for i in range(self.camera_num):
                self.rgb_camera.append(
                    CameraSensorReplayBEV(
                        vehicle, self.carla_world, mount_position[i],
                        self.global_position))

        else:
            self.rgb_camera = None

        self.lidar = None
        self.o3d_vis = None

        self.semantic_lidar = None
        # count how many steps have been passed
        self.count = 0
        # ego position
        self.ego_pos = None

        # the dictionary contains all objects
        self.objects = {}
        # traffic light detection related
        self.traffic_thresh = config_yaml['traffic_light_thresh'] \
            if 'traffic_light_thresh' in config_yaml else 50

class CameraSensorReplayBEV(CameraSensor):

    @staticmethod
    def spawn_point_estimation(relative_position, global_position):

        # pitch = 0
        carla_location = carla.Location(x=0, y=0, z=0)
        # x, y, z, pitch, yaw, roll = relative_position

        carla_location = carla.Location(x=carla_location.x,
                                        y=carla_location.y,
                                        z=carla_location.z+40)

        carla_rotation = carla.Rotation(roll=0, yaw=0, pitch=-90)
        spawn_point = carla.Transform(carla_location, carla_rotation)

        return spawn_point
