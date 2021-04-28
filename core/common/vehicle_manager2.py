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
from core.plan.behavior_agent import BehaviorAgent


class VehicleManager(object):
    """
    A class manager to embed different modules with vehicle together
    """

    def __init__(self, vehicle, config_yaml, application, world):
        """
        Construction class
        :param vehicle: carla actor
        :param config_yaml: a dictionary that contains the parameters of the vehicle
        :param application: application category, support:['single','platoon'] currently
        :param world: TODO: Temprory, remove it step by step
        """
        # an unique uuid for this vehicle
        self.vid = str(uuid.uuid1())
        self.vehicle = vehicle

        # retrieve the configure for different modules
        sensing_config = config_yaml['sensing']
        behavior_config = config_yaml['behavior']
        control_config = config_yaml['controller']
        v2x_config = config_yaml['v2x']

        # behavior agent
        self.agent = None
        if application == 'single':
            self.agent = BehaviorAgent(vehicle, behavior_config)

        # TODO: remove this later
        world.update_vehicle_manager(self)
        self.world = weakref.ref(world)()

    def update_info(self, world, frontal_vehicle=None):
        """
        Update the world and platooning information
        :param world:
        :param frontal_vehicle:
        :return:
        """
        self.agent.update_information(world, frontal_vehicle)

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation based on platooning status
        :return:
        """
        # TODO: use a safer way to pass target speed
        control = self.agent.run_step(target_speed)
        return control

    def destroy(self):
        """
        Destroy the actor vehicle
        :return:
        """
        self.vehicle.destroy()
