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
        self.cda_enabled = config_yaml['cda_enabled']
        self.communication_range = config_yaml['communication_range']

        # used for cooperative perception.
        self._recieved_buffer = {}

        # used for platooning
        self.platooning_plugin = PlatooningPlugin(self.communication_range, self.cda_enabled)

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

    def get_platoon_status(self):
        """
        Retrive the FSM status for platooning application
        :return:
        """
        return self.platooning_plugin.status
