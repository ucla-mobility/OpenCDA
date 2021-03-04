# -*- coding: utf-8 -*-

"""Platooning plugin for communication and track FSM
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import pickle
import os
import sys

import model.GFS.FISmodule as FISmodule
import model.GFS.FISmoduleGFSBestMergePoint as FISmoduleGFSBestMergePoint

from core.agents.tools.misc import compute_distance, cal_distance_angle
from core.platooning.fsm import FSM
from model.GFS.GFS_controller import GFSController

sys.modules['FISmodule'] = FISmodule
sys.modules['FISmoduleGFSBestMergePoint'] = FISmoduleGFSBestMergePoint


class PlatooningPlugin(object):
    """
    Platooning Plugin
    """

    def __init__(self, cda_enabled=True, in_platooning=False,
                 platooning_id=None, leader=False, status=FSM.SEARCHING,
                 search_range=35):
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

        # the platooning list that not consider to join anymore
        self.platooning_black_list = []

        # gfs model path
        absolute_path = os.path.abspath(__file__)
        diretory_path = os.path.dirname(absolute_path)
        model_path = os.path.join(diretory_path, '../../model/GFS/')

        # load 3 different fuzzy model for platooning joining position, speed and merging speed
        with open(os.path.join(model_path, 'BestFIS-theBest37-Safe.pickle'), 'rb') as f:
            gfs_m_speed = pickle.load(f)

        with open(os.path.join(model_path, 'BestFIS-pl-score.pickle'), 'rb') as g:
            gfs_pl_score = pickle.load(g)

        with open(os.path.join(model_path, 'BestGFS_PL_speed.pickle'), 'rb') as h:
            gfs_pl_speed = pickle.load(h)

        self.gfs_controller = GFSController(gfs_pl_score, gfs_pl_speed, gfs_m_speed, search_range + 20, 0.05)

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

    def platooning_search_gfs(self, ego_id, platooning_world, ego_vehicle):
        """
        Search Platooning and find the best joining position and corresponding speed with gfs model
        :param ego_id:
        :param platooning_world:
        :param ego_vehicle:
        :return:
        """

        cur_loc = ego_vehicle.get_location()
        cur_yaw = ego_vehicle.get_transform().rotation.yaw

        uuid, vm = self.communication_searching(ego_id, platooning_world, cur_loc)
        if not uuid:
            return False, 0, 0, None

        else:
            _, _, platooning_object = vm.get_platooning_status()
            if platooning_object.pmid in self.platooning_black_list:
                return False, 0, 0, None
            front_vehicle, rear_vehicle, best_idx = self.gfs_controller.getBestMergePosition(ego_vehicle,
                                                                                             platooning_object)
            self.front_vehicle = front_vehicle
            self.rear_vechile = rear_vehicle

            return True, 0, best_idx, platooning_object

    def platooning_search(self, ego_id, platooning_world, ego_vehicle):
        """
        Search platooning nearby and if any existed communicate with them to prepare for joinning
        :param ego_id:
        :param platooning_world:
        :param ego_vehicle:
        :return:
        """
        cur_loc = ego_vehicle.get_location()
        cur_yaw = ego_vehicle.get_transform().rotation.yaw

        uuid, vm = self.communication_searching(ego_id, platooning_world, cur_loc)
        if not uuid:
            return False, 0, 0, None
        else:
            _, _, platooning_object = vm.get_platooning_status()
            if platooning_object.pmid in self.platooning_black_list:
                return False, 0, 0, None

            min_distance = float('inf')
            min_index = -1
            min_angle = 0

            # if the platooning is not open to joining
            if not platooning_object.response_joining_request():
                return False, 0, 0, None

            for (i, vehicle_manager) in enumerate(platooning_object.vehicle_manager_list):
                distance, angle = cal_distance_angle(vehicle_manager.vehicle.get_location(),
                                                     cur_loc, cur_yaw)
                if distance < min_distance:
                    min_distance = distance
                    min_index = i
                    min_angle = angle

            # if the ego is in front of the platooning
            if min_index == 0 and min_angle > 90:
                self.front_vehicle = None
                self.rear_vechile = platooning_object.vehicle_manager_list[0]
                return True, min_distance, min_index, platooning_object

            self.front_vehicle = platooning_object.vehicle_manager_list[min_index]

            if min_index < len(platooning_object.vehicle_manager_list) - 1:
                self.rear_vechile = platooning_object.vehicle_manager_list[min_index + 1]

            return True, min_distance, min_index, platooning_object
