# -*- coding: utf-8 -*-

"""Communication manager for cooperation
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import weakref

from opencda.core.application.platooning.platooning_plugin import PlatooningPlugin


class V2XManager(object):
    """
    V2X Manager for platooning, cooperative perception and so on.

    Parameters
    -cav_world : opencda object
        CAV world.
    -config_yaml : dict
        The configuration dictionary of the v2x module.
    
    Attributes
    -_recieved_buffer : dict
        A buffer for receive data.
    -platooning_plugin : opencda object
        The platooning plugin for communication during platooning.
    """

    def __init__(self, cav_world, config_yaml):
        
        # if disabled, no cooperation will be operated
        self.cda_enabled = config_yaml['enabled']
        self.communication_range = config_yaml['communication_range']

        # used for cooperative perception.
        self._recieved_buffer = {}

        # used for platooning communication
        self.platooning_plugin = PlatooningPlugin(self.communication_range, self.cda_enabled)

        self.cav_world = weakref.ref(cav_world)()

    def update_info(self, ego_pos, ego_spd):
        """
        Update all communication plugins with current localization info.
        """
        self.platooning_plugin.update_info(ego_pos, ego_spd)

    def set_platoon(self, in_id, platooning_object=None, platooning_id=None, leader=False):
        """
        Set platooning status

        Args
        -platooning_object (platoon object): platooning world that contains all platoon information todo: remove this later.
        -platooning_id (int): platoon id the cav belongs to.
        -in_id (int): the position in the platoon, etc. 0 represents leader and 1 represents the second position.
        -leader (boolean): indicate whether this cav is a leader in platoon.
        
        """
        self.platooning_plugin.set_platoon(in_id, platooning_object, platooning_id, leader)

    def set_platoon_status(self, status):
        """
        Set the cav to a different fsm status.
        
        Args
            -status (string): fsm status.

        """
        self.platooning_plugin.set_status(status)

    def set_platoon_front(self, vm):
        """
        Set the frontal vehicle to another vehicle

        Args
           - vm (vehicle manager):The target vehicle manager.

        """
        self.platooning_plugin.front_vehicle = vm

    def set_platoon_rear(self, vm):
        """
        Set the rear vehicle to another vehicle

        Args:
            - vm (vehicle manager):The target vehicle manager.
        """
        self.platooning_plugin.rear_vechile = vm

    def add_platoon_blacklist(self, pmid):
        """
        Add an existing platoon to current blacklist.
        
        Args:
            - pmid (int):The target platoon manager ID.
        """
        self.platooning_plugin.platooning_blacklist.append(pmid)

    def match_platoon(self):
        """
        A naive way to find the best position to join a platoon.
        """
        return self.platooning_plugin.match_platoon(self.cav_world)

    def in_platoon(self):
        """
        Check whether the vehicle is inside the platoon.

        Args:
            - detection result (bool): Flag indication whether in a platoon.
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
        """
        return self.platooning_plugin.status

    def get_platoon_front_rear(self):
        """
        Get the ego vehicle's front and rear cav in the platoon
        """
        return self.platooning_plugin.front_vehicle, self.platooning_plugin.rear_vechile