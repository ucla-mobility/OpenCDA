# -*- coding: utf-8 -*-

"""Platooning plugin for communication and track FSM
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from core.agents.tools.misc import compute_distance
from core.platooning.fsm import FSM


class PlatooningPlugin(object):
    """
    Platooning Plugin
    """

    def __init__(self, cda_enabled=True, in_platooning=False,
                 platooning_id=None, leader=False, status=FSM.SEARCHING,
                 search_range=200):
        """
        Construct class
        :param cda_enabled: whether cda enabled
        :param in_platooning:  whether the vehicle is in platooning
        :param platooning_id:  the platooning id that the vehicle joined
        :param leader: whether this vehicle is a leader in platooning
        :param status: the FSM status
        :param search_range: searching range for communication
        """

        self.cda_enabled = cda_enabled
        self.in_platooning = in_platooning
        self.platooning_id = platooning_id
        self.platooning_object = None
        self.leader = leader
        self.id_in_team = None
        self.status = status
        self._range = search_range

        # used for cut-in joining
        self.front_vehicle = None
        self.rear_vechile = None

    def dissolve(self):
        """
        Vehicle dissolve from the platooning
        :return:
        """
        self.in_platooning = False
        self.platooning_id = None
        self.leader = False

    def take_charge(self):
        """
        Vehicle becomes the leader
        :return:
        """
        self.leader = True
        self.status = FSM.MAINTINING

    def communication_searching(self, ego_id, platooning_world, cur_loc):
        """
        Search platooning nearby
        :param ego_id:
        :param platooning_world:
        :param cur_loc:
        :return:
        """
        vehicle_manager_dict = platooning_world.get_vehicle_managers()
        for uuid, vm in vehicle_manager_dict.items():
            if uuid == ego_id:
                continue
            distance = compute_distance(cur_loc, vm.vehicle.get_location())
            if distance < self._range:
                return uuid, vm
        return None, None

    def platooning_search(self, ego_id, platooning_world, cur_loc):
        """
        Search platooning nearby and if any existed communicate with them to prepare for joinning
        :param ego_id:
        :param platooning_world:
        :param cur_loc:
        :return:
        """
        uuid, vm = self.communication_searching(ego_id, platooning_world, cur_loc)
        if not uuid:
            return False
        else:
            _, _, platooning_object = vm.get_platooning_status()
            min_distance = float('inf')
            min_index = -1

            # if the platooning is not open to joining
            if not platooning_object.response_joining_request():
                return False

            for (i, vehicle_manager) in enumerate(platooning_object.vehicle_manager_list):
                distance = compute_distance(cur_loc, vehicle_manager.vehicle.get_location())
                if distance < min_distance:
                    min_distance = distance
                    min_index = i

            self.front_vehicle = platooning_object.vehicle_manager_list[min_index]

            if min_index < len(platooning_object.vehicle_manager_list) - 1:
                self.rear_vechile = platooning_object.vehicle_manager_list[min_index + 1]

            return True, min_distance, min_index
