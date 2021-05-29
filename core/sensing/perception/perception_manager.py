# -*- coding: utf-8 -*-
"""
Perception module
"""


# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import sys

import carla

from core.sensing.perception.obstacle_vehicle import ObstacleVehicle


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

        # todo: currently only this parameter exist
        self.activate = config_yaml['activate']
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
            vehicle_list = [v for v in vehicle_list if self.dist(v) < 60 and
                            v.id != self.vehicle.id]
            objects.update({'vehicles': vehicle_list})
        else:
            sys.exit('Current version does not implement any perception algorithm')

        return objects

