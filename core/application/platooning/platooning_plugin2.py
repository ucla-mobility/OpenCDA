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

    def set_platoon(self, in_id, platooning_object=None, platooning_id=None, leader=False):
        """
        Set platooning status
        :param platooning_object: platooning manager todo: remove this later
        :param platooning_id: platoon id the cav belongs to
        :param in_id: the position in the platoon, etc. 0 represents leader and 1 represents the second position
        :param leader: indicate whether this cav is a leader in platoon
        :return:
        """
        if not in_id:
            if not self.cda_enabled:
                self.set_status(FSM.DISABLE)
                warnings.warn("CDA feature is disabled, can not activate platooning application ")
            else:
                self.set_status(FSM.SEARCHING)

        self.platooning_object = platooning_object
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
