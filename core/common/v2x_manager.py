# -*- coding: utf-8 -*-

"""Communication manager for cooperation
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla

from core.application.platooning.platooning_plugin2 import PlatooningPlugin


class V2XManager(object):
    """
    V2X Manager for platooning, cooperative perception and so on
    """

    def __init__(self, config_yaml):
        """
        Construct class
        :param config_yaml: configuration yaml file
        """
        # if disabled, no cooperation will be operated
        self.cda_enabled = config_yaml['enabled']
        self.communication_range = config_yaml['communication_range']

        # used for cooperative perception.
        self._recieved_buffer = {}

        # used for platooning communication
        self.platooning_plugin = PlatooningPlugin(self.communication_range, self.cda_enabled)

    def update_info(self, ego_pos, ego_spd):
        """
        Update all communication plugins with current localization info
        """
        self.platooning_plugin.update_info(ego_pos, ego_spd)

    """
    Followings are platooning-specific functions
    """

    def set_platoon(self, in_id, platooning_object=None, platooning_id=None, leader=False):
        """
        Set platooning status
        :param platooning_object: platooning world that contains all platoon information todo: remove this later
        :param platooning_id: platoon id the cav belongs to
        :param in_id: the position in the platoon, etc. 0 represents leader and 1 represents the second position
        :param leader: indicate whether this cav is a leader in platoon
        :return:
        """
        self.platooning_plugin.set_platoon(in_id, platooning_object, platooning_id, leader)

    def set_platoon_status(self, status):
        """
        Set the cav to a different fsm status
        :param status: fsm status
        :return:
        """
        self.platooning_plugin.set_status(status)

    def set_platoon_front(self, vm):
        """
        Set the frontal vehicle to another vehicle
        :param vm: vehicle manager
        :return:
        """
        self.platooning_plugin.front_vehicle = vm

    def set_platoon_rear(self, vm):
        """
        Set the rear vehicle to another vehicle
        :param vm:
        :return:
        """
        self.platooning_plugin.rear_vechile = vm

    def add_platoon_blacklist(self, pmid):
        """
        Add an existing platoon to current blacklist
        :param pmid: platoon id
        :return:
        """
        self.platooning_plugin.platooning_blacklist.append(pmid)

    def match_platoon(self, platoon_world):
        """
        A naive way to find the best position to join a platoon
        :param platoon_world:
        :return:
        """
        return self.platooning_plugin.match_platoon(platoon_world)

    def in_platoon(self):
        """
        Check whether the vehicle is inside the platoon
        :return: bool, flag indication whether in a platoon
        """
        return False if self.platooning_plugin.in_id is None else True

    def get_platoon_manager(self):
        """
        Retrieve the platoon manager the cav belongs to and the corresponding id
        :return:
        """
        return self.platooning_plugin.platooning_object, self.platooning_plugin.in_id

    def get_platoon_status(self):
        """
        Retrive the FSM status for platooning application
        :return:
        """
        return self.platooning_plugin.status

    def get_platoon_front_rear(self):
        """
        Get the ego vehicle's front and rear cav in the platoon
        :return:
        """
        return self.platooning_plugin.front_vehicle, self.platooning_plugin.rear_vechile