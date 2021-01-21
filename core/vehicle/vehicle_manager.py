# -*- coding: utf-8 -*-

"""A class manager to embed different plugins with vehicle
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla

from core.agents.navigation.behavior_agent import BehaviorAgent
from core.communication.vehicle_communication_manager import VehicleCommunicationManager


class VehicleManager(object):
    """
    A class manager to embed different modules with vehicle together
    """

    def __init__(self, vehicle, behavior='normal', communication_range=10,
                 buffer_size=5, sample_resolution=4.5, cda_enabled=True):
        """
        Construct class
        :param vehicle: carla Actor
        :param behavior: driving style.
        :param communication_range:
        :param buffer_size: queue size for behavior planning
        :param sample_resolution: the minimum distance between any waypoint in the routing
        :param cda_enabled:  whether the vehicle equipped with cda feature
        """
        self.vehicle = vehicle
        self.agent = BehaviorAgent(vehicle, behavior=behavior,
                                   buffer_size=buffer_size, sampling_resolution=sample_resolution)

        self._communication_manager = VehicleCommunicationManager(communication_range)
