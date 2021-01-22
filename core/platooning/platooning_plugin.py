# -*- coding: utf-8 -*-

"""Platooning plugin
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from core.platooning.fsm import FSM


class PlatooningPlugin(object):
    """
    Platooning Plugin
    """

    def __init__(self, cda_enabled=True, in_platooning=False,
                 platooning_id=None, leader=False, status=FSM.SEARCHING):
        """
        Construct class
        :param cda_enabled: whether cda enabled
        :param in_platooning:  whether the vehicle is in platooning
        :param platooning_id:  the platooning id that the vehicle joined
        """

        self.cda_enabled = cda_enabled
        self.in_platooning = in_platooning
        self.platooning_id = platooning_id
        self.leader = leader
        self.status = status

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
