# -*- coding: utf-8 -*-
"""
Utilize scenario manager to manage CARLA simulation construction. This script
is used for carla simulation only, and if you want to manage the Co-simulation,
please use cosim_api.py.
"""
# Author: Xu Han
# License: TDG-Attribution-NonCommercial-NoDistrib
import os
import numpy as np
import random
import copy
import sys
import pkg_resources
from typing import Any, Union, Optional, Dict, List
from distutils.version import LooseVersion
from collections import defaultdict
from abc import ABC, abstractmethod
from easydict import EasyDict

import carla
from carla import WeatherParameters

from ding.utils.default_helper import deep_merge_dicts
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.utils.sim_api import ScenarioManager
from opencda.core.ml_libs.rl.utils.simulator_utils.sensor_utils \
    import SensorHelper, CollisionSensor, TrafficLightHelper
from opencda.core.ml_libs.rl.utils.simulator_utils.map_utils \
    import BeVWrapper
from opencda.core.ml_libs.rl.utils.simulator_utils.carla_utils \
    import control_to_signal, get_birdview
from opencda.core.ml_libs.rl.utils.others.tcp_helper \
    import find_traffic_manager_port
from opencda.core.ml_libs.rl.utils.simulator_utils.map_utils import BeVWrapper


VEHICLE_NAME = 'vehicle.lincoln.mkz'
ROLE_NAME = 'hero'
OBS_TYPE_LIST = ['state', 'depth', 'rgb', 'segmentation', 'bev', 'lidar', 'gnss']

class RLScenarioManager(ScenarioManager):
    """
    A scenario manager class with special RL functionalities.
    The RL scenario manager creates a client to Carla server, and is able to get observation, send
    control signals to the hero vehicle and record essential data from the simulated world. The
    RL scenario manager may change the environment parameters including maps and weathers and can
    add actors.The RL scenario manager also retrieve running state and information of the hero vehicle.
    Note: The xodr is defined as none as RL is based on CARLA. The apply_ml is set to False as RL is used.

    Parameters
    ----------
    scenario_params : dict
        The dictionary contains all simulation configurations.

    cfg : dict
        The RL configuration file.

    town : str
        Town name if not using customized map, eg. 'Town06'.

    cav_world : opencda object
        CAV World that contains the information of all CAVs.

    Attributes
    ----------
    client : carla.client
        The client that connects to carla server.

    world : carla.world
        Carla simulation server.

    origin_settings : dict
        The origin setting of the simulation server.

    carla_map : carla.map
        Carla HD Map.
    """
    def __init__(self,
                 scenario_params,
                 cfg,
                 town,
                 cav_world,
                 version='0.9.11') -> None:
        """
        Init RL_Scenario_manager.
        """
        super().__init__(scenario_params=scenario_params,
                         apply_ml=False,
                         carla_version=version,
                         xodr_path=None,
                         town=town,
                         cav_world=cav_world)
        self._cfg = cfg
        self._cav_world = cav_world
        self._no_rendering = cfg.no_rendering

        self._simulation_config = scenario_params['world']
        self._town_name = cfg['town']
        self._sync_mode = self._simulation_config['sync_mode']

        self._col_threshold = self._cfg.col_threshold
        self._waypoint_num = self._cfg.waypoint_num
        self._obs_cfg = self._cfg.obs
        # self._planner_cfg = self._cfg.planner
        self._camera_aug_cfg = self._cfg.camera_aug
        self._verbose = self._cfg.verbose
        self._debug = self._cfg.debug

        self._tick = 0
        self._timestamp = 0
        self._collided = False
        self._ran_light = False
        self._off_road = False

        self._vehicle = None
        self._sensor_helper = None
        self._bev_wrapper = None
        self._collision_sensor = None
        self._traffic_light_helper = None

    def prepare_observations(self, hero_vehicle) -> None:
        """
        Prepare the initial observation for RL model when initiate world. Initiate collision sensor
        and traffic light helper.

        Parameters
        ----------
        hero_vehicle:carla.vehicle
            The host vehicle that the rl agent is controlling.
        """
        self._vehicle = hero_vehicle
        self._sensor_helper = SensorHelper(self._obs_cfg, self._camera_aug_cfg)
        self._sensor_helper.setup_sensors(self.world, hero_vehicle)
        while not self._sensor_helper.all_sensors_ready():
            self.world.tick()

        if self._obs_cfg.type == 'bev':
            self._bev_wrapper = BeVWrapper(self._obs_cfg)
            self._bev_wrapper.init(self.client, self.world, self.carla_map, hero_vehicle)

        self._collision_sensor = CollisionSensor(hero_vehicle, self._col_threshold)
        self._traffic_light_helper = TrafficLightHelper(hero_vehicle, self.carla_map)

    def update_bev_waypoints(self, waypoint_list):
        """
        Update bev wrapper with waypoint lists.
        Parameters
        ----------
        waypoint_list:list
            The current waypoint list.
        """
        self._bev_wrapper.update_waypoints(waypoint_list)

    def get_state(self):
        """
        Get the current hero vehicle state (vehicle dynamic and navigation info), and organize them as an
        observation dictionary which will be used by carla ENV to generate observation for RL model.

        Returns
        -------
        state:dict
            The organized state data (vehicle dynamic and navigation info) in a dictionary.
        """
        # check traffic light state
        light_state = self._traffic_light_helper.active_light_state.value
        # read location
        location = self._vehicle.get_location()
        # map info
        # Note: Only use scenario manager attribute to access the carla map. Retrieve map is expensive.
        drive_waypoint = self.carla_map.get_waypoint(
            location,
            project_to_road=False)
        is_junction = False

        # check if off-road
        if drive_waypoint is not None:
            is_junction = drive_waypoint.is_junction
            self._off_road = False
        else:
            self._off_road = True

        # Note: Only use scenario manager attribute to access the carla map. Retrieve map is expensive.
        lane_waypoint = self.carla_map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
        lane_location = lane_waypoint.transform.location
        lane_forward_vector = lane_waypoint.transform.rotation.get_forward_vector()

        state = {
            'is_junction': is_junction,
            'lane_location': np.array([lane_location.x, lane_location.y]),
            'lane_forward': np.array([lane_forward_vector.x, lane_forward_vector.y]),
            'tl_state': light_state,
            'tl_dis': self._traffic_light_helper.active_light_dis
        }
        if lane_waypoint is None:
            state['lane_forward'] = None
        else:
            lane_forward_vector = lane_waypoint.transform.get_forward_vector()
            state['lane_forward'] = np.array([lane_forward_vector.x, lane_forward_vector.y])

        return state

    def get_sensor_data(self) -> Dict:
        """
        Get all sensor data and bird-eye view data if exist in current world. BEV will be
        converted to a multichannel 2D grid.
        """
        sensor_data = self._sensor_helper.get_sensors_data()

        if self._obs_cfg.type not in OBS_TYPE_LIST:
            raise NotImplementedError("observation type %s not implemented" % obs_item.type)
        elif self._obs_cfg.type == 'bev':
            key = self._obs_cfg.name
            sensor_data.update({key: get_birdview(self._bev_wrapper.get_bev_data())})

        return sensor_data

    def get_information(self):
        """
        Get current simulation information (tick, timestamp, traffic lights passed and ran) in a dictionary.

        Returns
        -------
        information: dict
            Current simulation information summarized in a dict.
        """
        information = {
            'tick': self._tick,
            'timestamp': self._timestamp,
            'total_lights': self._traffic_light_helper.total_lights,
            'total_lights_ran': self._traffic_light_helper.total_lights_ran
        }
        return information

    def run_step(self):
        """
        Run one simulation step.
        """
        # tick world
        self.world.tick()
        self._tick += 1
        world_snapshot = self.world.get_snapshot()
        self._timestamp = world_snapshot.timestamp.elapsed_seconds

        # update vehicle status
        self._collided = self._collision_sensor.collided
        self._traffic_light_helper.tick()
        self._ran_light = self._traffic_light_helper.ran_light

        # prepare BEV map
        if self._bev_wrapper is not None:
            self._bev_wrapper.tick()

    def clean_up(self) -> None:
        """
        Destroy all actors and sensors in current world. Clear all messages saved in simulator.
        This will NOT destroy the Carla client, so simulator can use same carla client to start next episode.
        """
        # destroy all actors (vehicles, sensors, BEVs)
        self.destroyActors()
        # check for helpers
        if self._sensor_helper is not None:
            self._sensor_helper.clean_up()
        if self._bev_wrapper is not None:
            self._bev_wrapper.clear()
        if self._collision_sensor is not None:
            self._collision_sensor.clear()

        # reset params
        self._tick = 0
        self._timestamp = 0
        self._collided = False
        self._ran_light = False
        self._off_road = False

    def is_vehicle_off_road(self):
        return self._off_road

    def is_vehicle_ran_light(self):
        return self._ran_light

    def is_vehicle_collided(self):
        return self._collided
