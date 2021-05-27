# -*- coding: utf-8 -*-

"""Platooning plugin for communication and track FSM
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import warnings
import sys

from core.common.misc import compute_distance, cal_distance_angle
from core.application.platooning.fsm import FSM


class PlatooningPlugin(object):
    """
    Platooning Plugin
    """

    def __init__(self, search_range, cda_enabled):
        """
        Construct class
        :param search_range:
        :param cda_enabled:
        """
        self.search_range = search_range
        self.cda_enabled = cda_enabled

        # whether leader in a platoon
        self.leader = False
        self.platooning_object = None
        self.platooning_id = None
        self.in_id = None
        self.status = None

        # ego speed and position
        self.ego_pos = None
        self.ego_spd = None

        # the platoon in the black list won't be considered again
        self.platooning_blacklist = []

        # used to label the front and rear vehicle position
        self.front_vehicle = None
        self.rear_vechile = None

    def update_info(self, ego_pos, ego_spd):
        """
        Update the ego position and speed
        :param ego_pos: ego position, carla.Transform
        :param ego_spd: ego speed, km/h
        :return:
        """
        self.ego_pos = ego_pos
        self.ego_spd = ego_spd

    def reset(self):
        """
        Reset to the origin status
        :return:
        """
        self.front_vehicle = None
        self.rear_vechile = None

        self.leader = False
        self.platooning_object = None
        self.platooning_id = None
        self.in_id = None

    def set_platoon(self, in_id, platooning_object=None, platooning_id=None, leader=False):
        """
        Set platooning status
        :param platooning_object: platooning manager todo: remove this later
        :param platooning_id: platoon id the cav belongs to
        :param in_id: the position in the platoon, etc. 0 represents leader and 1 represents the second position
        :param leader: indicate whether this cav is a leader in platoon
        :return:
        """
        if in_id is None:
            if not self.cda_enabled:
                self.set_status(FSM.DISABLE)
                warnings.warn("CDA feature is disabled, can not activate platooning application ")
            else:
                self.set_status(FSM.SEARCHING)
            return

        if platooning_object:
            self.platooning_object = platooning_object
        if platooning_id:
            self.platooning_id = platooning_id

        self.in_id = in_id
        if leader:
            self.leader = leader
            self.set_status(FSM.LEADING_MODE)
        else:
            self.set_status(FSM.MAINTINING)

    def set_status(self, status):
        """
        Set FSM status
        :param status:
        :return:
        """
        self.status = status

    def search_platoon(self, ego_pos, platooning_world):
        """
        Search platoon candidate in the range
        :param ego_pos:
        :param platooning_world:
        :return: the uuid of platoon member, platoon object
        """
        platoon_manager_dict = platooning_world.get_platoon_dict()
        for pmid, pm in platoon_manager_dict.items():
            for vm in pm.vehicle_manager_list:
                distance = compute_distance(ego_pos, vm.localizer.get_ego_pos().location)
                if distance < self.search_range:
                    return pmid, pm
        return None, None

    def match_platoon(self, platooning_world):
        """
        A naive way to find the best position to join a platoon
        :param platooning_world: an object containing all existing platoons
        :return: platoon found or not, closest platoon member team id
        """
        # make sure the previous status won't influence current one
        self.reset()

        cur_loc = self.ego_pos.location
        cur_yaw = self.ego_pos.rotation.yaw

        pmid, pm = self.search_platoon(cur_loc, platooning_world)

        if not pmid or pmid in self.platooning_blacklist:
            return False, -1

        # used to search the closest platoon member in the searched platoon
        min_distance = float('inf')
        min_index = -1
        min_angle = 0

        # if the platooning is not open to joining
        if not pm.response_joining_request():
            return False, -1

        for (i, vehicle_manager) in enumerate(pm.vehicle_manager_list):
            distance, angle = cal_distance_angle(vehicle_manager.vehicle.get_location(),
                                                 cur_loc, cur_yaw)
            if distance < min_distance:
                min_distance = distance
                min_index = i
                min_angle = angle

        # if the ego is in front of the platooning
        if min_index == 0 and min_angle > 90:
            self.front_vehicle = None
            self.rear_vechile = pm.vehicle_manager_list[0]
            return True, min_index

        self.front_vehicle = pm.vehicle_manager_list[min_index]

        if min_index < len(pm.vehicle_manager_list) - 1:
            self.rear_vechile = pm.vehicle_manager_list[min_index + 1]

        return True, min_index
