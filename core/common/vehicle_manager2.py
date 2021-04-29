# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import uuid
import weakref

from core.plan.behavior_agent import BehaviorAgent
from core.actuation.pid_controller import VehiclePIDController


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

        # controller TODO: Add a wrapper class for all controller types
        self.controller = VehiclePIDController(control_config['args'])

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
        # TODO: should give output from localization module
        self.controller.update_info(self.vehicle)

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation based on platooning status
        :return:
        """
        # TODO: use a safer way to pass target speed
        target_speed, target_pos = self.agent.run_step(target_speed)
        control = self.controller.run_step(target_speed, target_pos)
        return control

    def destroy(self):
        """
        Destroy the actor vehicle
        :return:
        """
        self.vehicle.destroy()
