# -*- coding: utf-8 -*-

"""A class manager to embed different plugins with vehicle
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import uuid

from core.agents.navigation.platoon_behavior_agent import PlatooningBehaviorAgent
from core.communication.vehicle_communication_manager import VehicleCommunicationManager
from core.platooning.platooning_plugin import PlatooningPlugin


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
        self.vid = str(uuid.uuid1())
        self.destination = None

        self.vehicle = vehicle
        self.agent = PlatooningBehaviorAgent(vehicle, behavior=behavior,
                                             buffer_size=buffer_size, sampling_resolution=sample_resolution)

        self._communication_manager = VehicleCommunicationManager(communication_range)
        self._platooning_plugin = PlatooningPlugin(cda_enabled)

    def set_platooning(self, platooning_id, lead=False):
        """
        Called when vehicle joined/formed a platooning
        :param lead:
        :param platooning_id:
        :return:
        """
        self._platooning_plugin.platooning_id = platooning_id
        self._platooning_plugin.in_platooning = True
        if lead:
            self._platooning_plugin.take_charge()

    def get_platooning_status(self):
        """
        Check whether the vehicle in the platooning
        :return:
        """
        return self._platooning_plugin.in_platooning, self._platooning_plugin.platooning_id

    def update_info(self, world, frontal_vehicle=None):
        """
        Update the world and platooning information
        :param world:
        :param frontal_vehicle:
        :return:
        """
        self.agent.update_information(world, frontal_vehicle)

    def run_step(self):
        """
        Execute one step of navigation based on platooning status
        :return:
        """
        # TODO: Add Cooperative Merge and Dissolove
        if not self._platooning_plugin.in_platooning or self._platooning_plugin.leader:
            control = self.agent.run_step()
        else:
            control = self.agent.run_step_following()

        return control
