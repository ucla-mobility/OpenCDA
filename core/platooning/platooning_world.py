# -*- coding: utf-8 -*-

"""Platooning World Object to save all platooning-related object
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from core.platooning.platooning_manager import PlatooningManager
from core.vehicle.vehicle_manager import VehicleManager


class PlatooningWorld(object):
    """
    A customized world object to save all platooning and CDA vehicle information
    :return:
    """

    def __init__(self):
        """
        Consturct class
        :param self:
        :return:
        """
        self.vehicle_id_set = set()
        self._vehicle_manager_dict = {}
        self._platooning_dict = {}
        # for co-simulation only, save the carla-sumo dictionary
        self.sumo2carla_ids = {}

    def update_vehicle_manager(self, vehicle_manager):
        """
        Update created vehicle manager to the world
        :param vehicle_manager:
        :return:
        """
        self.vehicle_id_set.add(vehicle_manager.vehicle.id)
        self._vehicle_manager_dict.update({vehicle_manager.vid: vehicle_manager})

    def update_platooning(self, platooning_manger):
        """
        Add created platooning
        :param platooning_manger:
        :return:
        """
        self._platooning_dict.update({platooning_manger.pmid: platooning_manger})

    def update_sumo_vehicles(self, sumo2carla_ids):
        """
        Update the sumo-carla id hash map for co-simulation
        :param sumo2carla_ids: {'sumo_id': 'carla_id'}
        :return:
        """
        self.sumo2carla_ids = sumo2carla_ids


    def get_vehicle_managers(self):
        """
        Return vehicle manager dictionary
        :return:
        """
        return self._vehicle_manager_dict