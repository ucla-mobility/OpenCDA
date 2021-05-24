# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import uuid
import weakref

from core.actuation.pid_controller import VehiclePIDController
from core.application.platooning.platoon_behavior_agent import PlatooningBehaviorAgent
from core.common.v2x_manager import V2XManager
from core.sensing.localization.localization_manager import LocalizationManager
from core.plan.behavior_agent import BehaviorAgent


class VehicleManager(object):
    """
    A class manager to embed different modules with vehicle together
    """

    def __init__(self, vehicle, config_yaml, application, carla_map, world=None):
        """
        Construction class todo: multiple application can be activated at the same time
        :param vehicle: carla actor
        :param config_yaml: a dictionary that contains the parameters of the vehicle
        :param application: application category, support:['single','platoon'] currently
        :param carla_map: Carla HD Map
        :param world: TODO: Temprory, remove it step by step
        """
        # an unique uuid for this vehicle
        self.vid = str(uuid.uuid1())
        self.vehicle = vehicle
        self.carla_map = carla_map

        # retrieve the configure for different modules
        sensing_config = config_yaml['sensing']
        behavior_config = config_yaml['behavior']
        control_config = config_yaml['controller']
        v2x_config = config_yaml['v2x']

        # v2x module
        self.v2x_manager = V2XManager(v2x_config)
        # localization module
        self.localizer = LocalizationManager(vehicle, sensing_config['localization'], carla_map)
        # behavior agent
        self.agent = None

        if 'platooning' in application:
            platoon_config = config_yaml['platoon']
            self.agent = PlatooningBehaviorAgent(vehicle, self, self.v2x_manager,
                                                 behavior_config, platoon_config,
                                                 world, carla_map)
        else:
            # todo: remove the vehicle
            self.agent = BehaviorAgent(vehicle, carla_map, behavior_config)

        # controller TODO: Add a wrapper class for all controller types
        self.controller = VehiclePIDController(control_config['args'])

        # TODO: remove this later. This is a wrong implmentation, cda disabled shouldn't be added to it
        world.update_vehicle_manager(self)
        self.world = weakref.ref(world)()

    def set_destination(self, start_location, end_location, clean=False, end_reset=True):
        """
        Wrapper function to set global route
        :param start_location:
        :param end_location:
        :param clean:
        :param end_reset:
        :return:
        """
        self.agent.set_destination(start_location, end_location, clean, end_reset)

    def update_info(self, world):
        """
        Call perception and localization module to retrieve surrounding info an ego position.
        :param world: # todo: only vm with platoon application should have this
        :return:
        """
        # localization
        self.localizer.localize()
        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()

        self.v2x_manager.update_info(ego_pos, ego_spd)
        self.agent.update_information(world, ego_pos, ego_spd)
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
        self.localizer.destroy()
        self.vehicle.destroy()
