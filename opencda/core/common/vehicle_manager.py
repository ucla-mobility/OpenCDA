# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import uuid

from opencda.core.actuation.control_manager import ControlManager
from opencda.core.application.platooning.platoon_behavior_agent import PlatooningBehaviorAgent
from opencda.core.common.v2x_manager import V2XManager
from opencda.core.sensing.localization.localization_manager import LocalizationManager
from opencda.core.sensing.perception.perception_manager import PerceptionManager
from opencda.core.plan.behavior_agent import BehaviorAgent


class VehicleManager(object):
    """
    A class manager to embed different modules with vehicle together.

    Parameters
    -vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.
    -config : dict
        The configuration dictionary of the localization module.
    -application : list
        The application category, currently support:['single','platoon'].
    -carla_map : carla.Map
        The CARLA simulation map.
    -cav_world : opencda object
        CAV World.
    
    Attributes
    -v2x_manager : opencda object
        The current V2X manageer. 
    -localizer : opencda object
        The current localization manageer. 
    -perception_manager : opencda object
        The current V2X perception manageer. 
    -agent : opencda object
        The current carla agent that handles the basic control of ego vehicle.
    -controller : opencda object
        The current control manager.
    """

    def __init__(self, vehicle, config_yaml, application, carla_map, cav_world):
        
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
        self.v2x_manager = V2XManager(cav_world, v2x_config)
        # localization module
        self.localizer = LocalizationManager(vehicle, sensing_config['localization'], carla_map)
        # perception module
        self.perception_manager = PerceptionManager(vehicle, sensing_config['perception'], cav_world.ml_manager)
        # behavior agent
        self.agent = None

        if 'platooning' in application:
            platoon_config = config_yaml['platoon']
            self.agent = PlatooningBehaviorAgent(vehicle, self, self.v2x_manager,
                                                 behavior_config, platoon_config, carla_map)
        else:
            self.agent = BehaviorAgent(vehicle, carla_map, behavior_config)

        self.controller = ControlManager(control_config)

        cav_world.update_vehicle_manager(self)

    def set_destination(self, start_location, end_location, clean=False, end_reset=True):
        """
        Wrapper function to set global route

        Args:
            -start_location (carla.location): The start location of the current task.
            -end_location (carla.location): The destination location of the current task.
            -clean (boolean): Indicator of whether clean waypoint queue.
            -end_reset (boolean): Indicator of whether reset the end location.
        """
        self.agent.set_destination(start_location, end_location, clean, end_reset)

    def update_info(self):
        """
        Call perception and localization module to retrieve surrounding info an ego position.
        """
        # localization
        self.localizer.localize()
        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()

        # object detection
        objects = self.perception_manager.detect(ego_pos)

        self.v2x_manager.update_info(ego_pos, ego_spd)
        self.agent.update_information(ego_pos, ego_spd, objects)
        # pass position and speed info to controller
        self.controller.update_info(ego_pos, ego_spd)

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation.
        """
        target_speed, target_pos = self.agent.run_step(target_speed)
        control = self.controller.run_step(target_speed, target_pos)
        return control

    def destroy(self):
        """
        Destroy the actor vehicle
        """
        self.perception_manager.destroy()
        self.localizer.destroy()
        self.vehicle.destroy()
