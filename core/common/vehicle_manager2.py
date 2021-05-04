# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import uuid
import weakref

from core.actuation.pid_controller import VehiclePIDController
from core.common.v2x_manager import V2XManager
from core.sensing.localization.localization_manager import LocalizationManager
from core.plan.behavior_agent import BehaviorAgent

class VehicleManager(object):
    """
    A class manager to embed different modules with vehicle together
    """

    def __init__(self, vehicle, config_yaml, application, world):
        """
        Construction class todo: multiple application can be activated at the same time
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

        # v2x module
        self.v2x_manager = V2XManager(v2x_config)
        # localization module
        self.localizer = LocalizationManager(vehicle, sensing_config['localization'])
        # behavior agent
        self.agent = None

        if application == 'single':
            # todo: remove the vehicle
            self.agent = BehaviorAgent(vehicle, behavior_config)

        # controller TODO: Add a wrapper class for all controller types
        self.controller = VehiclePIDController(control_config['args'])

        # TODO: remove this later. This is a wrong implmentation, cda disabled shouldn't be added to it
        world.update_vehicle_manager(self)
        self.world = weakref.ref(world)()

    def update_info(self, world, frontal_vehicle=None):
        """
        Call perception and localization module to retrieve surrounding info an ego position.
        :param world:
        :param frontal_vehicle:
        :return:
        """
        # localization
        self.localizer.localize()
        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()

        self.agent.update_information(world, frontal_vehicle)
        # pass position and speed info to controller
        self.controller.update_info(ego_pos, ego_spd)

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation.
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
