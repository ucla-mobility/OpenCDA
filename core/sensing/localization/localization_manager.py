# -*- coding: utf-8 -*-
"""
Localization module TODO: Will add more content next version
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import sys

from core.common.misc import get_speed


class LocalizationManager(object):
    """
    The core class that manages localization estimation. todo: remove vehicle actor
    """
    def __init__(self, vehicle, config_yaml):
        """
        Construction class
        :param vehicle: carla actor
        :param config_yaml: configuration related to localization
        """
        self.vehicle = vehicle
        self.activate = config_yaml['activate']
        self._ego_pos = None
        self._speed = 0
        # we need to retrieve an initialized position. todo: create a specific initializer later
        self.localize()

    def localize(self):
        """
        Currently implemented in a naive way. todo: will add more contents next version
        :return:
        """
        if not self.activate:
            self._ego_pos = self.vehicle.get_transform()
            self._speed = get_speed(self.vehicle)
        else:
            sys.exit('Localization currently not implemented, please wait for next version.')

    def get_ego_pos(self):
        """
        Retrieve ego vehicle position
        :return: vehicle position
        """
        return self._ego_pos

    def get_ego_spd(self):
        """
        Retrieve ego vehicle speed
        :return:
        """
        return self._speed