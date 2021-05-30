# -*- coding: utf-8 -*-

"""Platooning World Object to save all platooning-related object
"""


# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT


class CavWorld(object):
    """
    A customized world object to save all CDA vehicle information
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

    def update_vehicle_manager(self, vehicle_manager):
        """
        Update created CAV manager to the world
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

    def get_vehicle_managers(self):
        """
        Return vehicle manager dictionary
        :return:
        """
        return self._vehicle_manager_dict

    def get_platoon_dict(self):
        """
        Return existing platoons
        :return:
        """
        return self._platooning_dict

    def locate_vehicle_manager(self, loc):
        """
        Locate the vehicle manager based on the given location.
        Args:
            loc (carla.Location): vehicle location.

        Returns:
            (VehicleManager): The vehicle manager at the give location.
        """

        target_vm = None
        for key, vm in self._vehicle_manager_dict.items():
            x = vm.localizer.get_ego_pos().location.x
            y = vm.localizer.get_ego_pos().location.y

            if loc.x == x and loc.y == y:
                target_vm = vm
                break

        return target_vm