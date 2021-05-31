# -*- coding: utf-8 -*-
"""
Perception module
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import weakref
import sys

import carla
import cv2
import numpy as np

from core.sensing.perception.obstacle_vehicle import ObstacleVehicle


class CameraSensor(object):
    """
    Class for rgb camera.
    """

    def __init__(self, vehicle):
        """
        Construct class.
        Args:
            vehicle (carla.Vehicle): Carla actor.
        """
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.camera.rgb')

        spawn_point = carla.Transform(carla.Location(x=2.5, z=1.0))
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


class PerceptionManager(object):
    """
    Perception manager mainly for object detection
    """

    def __init__(self, vehicle, config_yaml):
        """
        Construct class.
        Args:
            vehicle (carla.Actor): The carla vehicle.
            config_yaml (dict):  The configuration yaml dictionary
        """
        self.vehicle = vehicle

        self.activate = config_yaml['activate']
        self.camera_visualize = config_yaml['camera_visualize']

        self.rgb_camera = CameraSensor(vehicle)

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
        world = self.vehicle.get_world()
        self.ego_pos = ego_pos

        objects = {}

        if not self.activate:
            vehicle_list = world.get_actors().filter("*vehicle*")
            vehicle_list = [v for v in vehicle_list if self.dist(v) < 50 and
                            v.id != self.vehicle.id]
            objects.update({'vehicles': vehicle_list})

            if self.camera_visualize:
                rgb_image = self.rgb_camera.image

                # show image using cv2
                cv2.imshow('rgb image of actor %d' % self.vehicle.id, rgb_image)
                cv2.waitKey(1)

        else:
            sys.exit('Current version does not implement any perception algorithm')

        return objects

    def destroy(self):
        """
        Destroy sensors.
        Returns:

        """
        self.rgb_camera.sensor.destroy()
        if self.camera_visualize:
            cv2.destroyAllWindows()