# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import uuid
import weakref
import statistics

from core.application.platooning.platoon_behavior_agent import PlatooningBehaviorAgent


class VehicleManager(object):
    """
    A class manager to embed different modules with vehicle together
    """
    def __init__(self, vehicle, config_yaml, application):
        """
        Construction class
        :param vehicle: carla actor
        :param config_yaml: a dictionary that contains the parameters of the vehicle
        :param application: application category, support:['single','platoon'] currently
        """
        # an unique uuid for this vehicle
        self.vid = str(uuid.uuid1())
        self.vehicle = vehicle

        # retrieve the configure for different modules
        sensing_config = config_yaml['sensing']
        behavior_config = config_yaml['behavior']
        control_config = config_yaml['controller']
        v2x_config = config_yaml['v2x']

        if application=='single':


