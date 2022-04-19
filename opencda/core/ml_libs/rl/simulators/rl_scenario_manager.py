'''
Copyright 2021 OpenDILab. All Rights Reserved:
Description: Carla simulator.
'''
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
from opencda.core.ml_libs.rl.simulators.carla_data_provider \
    import CarlaDataProvider
from opencda.core.ml_libs.rl.utils.simulator_utils.carla_utils \
    import control_to_signal, get_birdview
from opencda.core.ml_libs.rl.utils.planner \
    import BasicPlanner, BehaviorPlanner # , LBCPlannerNew
from opencda.core.ml_libs.rl.utils.others.tcp_helper \
    import find_traffic_manager_port


VEHICLE_NAME = 'vehicle.lincoln.mkz'
ROLE_NAME = 'hero'
OBS_TYPE_LIST = ['state', 'depth', 'rgb', 'segmentation', 'bev', 'lidar', 'gnss']

PLANNER_DICT = {
    'basic': BasicPlanner,
    'behavior': BehaviorPlanner
}


# class CarlaSimulator(BaseSimulator):
# class CarlaSimulator(ABC):

class RLScenarioManager(ScenarioManager):
    """
    A scenario manager class with special RL functionalities. 
    The RL scenario manager creates a client to Carla server, and is able to get observation, send
    control signals to the hero vehicle and record essential data from the simulated world. The 
    RL scenario manager may change the environment parameters including maps and weathers and can 
    add actors.The RL scenario manager also retrieve running state and information of the hero vehicle.

    :Arguments:
        - cfg (Dict): Config Dict.
        - client (carla.Client, optional): Already established Carla client. Defaults to None.
        - host (str, optional): TCP host Carla client link to. Defaults to 'localhost'.
        - port (int, optional): TCP port Carla client link to. Defaults to 9000.
        - tm_port (int, optional): Traffic manager port Carla client link to. Defaults to None.
        - timeout (float, optional): Carla client link timeout. Defaults to 60.0.

    :Interfaces:
        init, get_state, get_sensor_data, get_navigation, get_information, apply_planner,
        apply_control, run_step, clean_up

    :Properties:
        - town_name (str): Current town name.
        - hero_player (carla.Actor): hero actor in simulation.
        - collided (bool): Whether collided in current episode.
        - ran_light (bool): Whether ran light in current frame.
        - off_road (bool): Whether ran off road in current frame.

    """

    def __init__(self,
                 scenario_params,
                 cfg,
                 town,
                 cav_world) -> None:
        """
        Init RL_Scenario_manager.
        """
        super().__init__(scenario_params=scenario_params,
                         apply_ml=False,
                         version='0.9.11',
                         xodr_path=None,
                         town=town,
                         cav_world=cav_world)

        self._cfg = cfg
        self._cav_world = cav_world

        # todo: redering
        self._no_rendering = self._cfg.no_rendering

        self._simulation_config = scenario_params['world']
        self._carla_port = port
        self._town_name = cfg['town']
        self._sync_mode = self._simulation_config['world']['sync_mode']

        # todo Note: use parent class varaibles instead
        self._client = None  # --> parent class has self.client
        self._world = None   # --> parent class has self.world
        self._map = None     # --> parent class has self.carla_map

        # todo: vheicle manager functions
        self._hero_actor = None
        self._single_cav_list = None
        self._start_location = None
        self._end_location = None

        #
        #
        self._col_threshold = self._cfg.col_threshold
        self._waypoint_num = self._cfg.waypoint_num
        self._obs_cfg = self._cfg.obs
        self._planner_cfg = self._cfg.planner
        self._camera_aug_cfg = self._cfg.camera_aug
        self._verbose = self._cfg.verbose

        self._tick = 0
        self._timestamp = 0
        self._collided = False
        self._ran_light = False
        self._off_road = False
        self._wrong_direction = False
        self._sensor_helper = None
        self._bev_wrapper = None
        self._collision_sensor = None
        self._traffic_light_helper = None

        self._debug = self._cfg.debug

    def prepare_observations(self, hero_vehicle) -> None:
        """
        Prepare the initial observation for RL model when initiate world. Initiate collision sensor
        and traffic light helper.

        Parameters
        ----------
        hero_vehicle:carla.vehicle
            The host vehicle that the rl agent is controlling.
        """
        self._sensor_helper = SensorHelper(self._obs_cfg, self._camera_aug_cfg)
        self._sensor_helper.setup_sensors(self.world, hero_vehicle)
        while not self._sensor_helper.all_sensors_ready():
            self.world.tick()

        for obs_item in self._obs_cfg:
            if obs_item.type == 'bev':
                self._bev_wrapper = BeVWrapper(obs_item)
                self._bev_wrapper.init(self.client, self.world, self.carla_map, hero_vehicle)

        self._collision_sensor = CollisionSensor(hero_vehicle, self._col_threshold)
        self._traffic_light_helper = TrafficLightHelper(hero_vehicle)

    def get_state(self, hero_vehicle_manager):
        """
        Get the current hero vehicle state (vehicle dynamic and navigation info), and organize them as an
        observation dictionary which will be used by carla ENV to generate observation for RL model.

        Parameters
        ----------
        hero_vehicle_manager: opencda.vechile_manager
            The current vehicle manager of the hero vehicle. It was used to read all simulation data.

        Returns
        -------
        state:dict
            The organized state data (vehicle dynamic and navigation info) in a dictionary.
        """
        # vehicle state info
        speed = hero_vehicle_manager.get_speed() * 3.6
        transform = hero_vehicle_manager.get_transform()
        location = hero_vehicle_manager.get_location()
        forward_vector = hero_vehicle_manager.get_forward_vector()
        acceleration = hero_vehicle_manager.get_acceleration()
        angular_velocity = hero_vehicle_manager.get_angular_velocity()
        velocity = hero_vehicle_manager.get_speed_vector()

        # todo: check light state helper
        light_state = self._traffic_light_helper.active_light_state.value

        # navigation info
        drive_waypoint = self.map.get_waypoint(
            location,
            project_to_road=False
        )
        is_junction = False
        if drive_waypoint is not None:
            is_junction = drive_waypoint.is_junction
            self._off_road = False
        else:
            self._off_road = True

        lane_waypoint = self.map.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
        lane_location = lane_waypoint.transform.location
        lane_forward_vector = lane_waypoint.transform.rotation.get_forward_vector()

        state = {
            'speed': speed,
            'location': np.array([location.x, location.y, location.z]),
            'forward_vector': np.array([forward_vector.x, forward_vector.y]),
            'acceleration': np.array([acceleration.x, acceleration.y, acceleration.z]),
            'velocity': np.array([velocity.x, velocity.y, velocity.z]),
            'angular_velocity': np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z]),
            'rotation': np.array([transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll]),
            'is_junction': is_junction,
            'lane_location': np.array([lane_location.x, lane_location.y]),
            'lane_forward': np.array([lane_forward_vector.x, lane_forward_vector.y]),
            # todo: check light state helper
            'tl_state': light_state,
            'tl_dis': self._traffic_light_helper.active_light_dis,
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

        for obs_item in self._obs_cfg:
            if obs_item.type not in OBS_TYPE_LIST:
                raise NotImplementedError("observation type %s not implemented" % obs_item.type)
            elif obs_item.type == 'bev':
                key = obs_item.name
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
            if CarlaDataProvider._hero_vehicle_route is not None:
                self._bev_wrapper.tick()

    def clean_up(self) -> None:
        """
        Destroy all actors and sensors in current world. Clear all messages saved in simulator and data provider.
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
        self._wrong_direction = False






    # !!! todo: List of todo items.... !!!
    """
    1. need-to-implement function list:          
        a. Navigation modules (planner class vs. openCDA BehaviorAgent) 
            self._rl_scenario_manager.get_navigation() --> Involves DI_drive planner:
                                                           Rewrite the planner class using opencda planner.
                                                           Preserve the same function to calculate distance/orientation from 
                                                           current host position to navigation target.
            related params:  (parmas that will be used by carla_env)
                    * self._off_road                             --> use next waypoint to determine if on road
                    * self._wrong_direction                      --> use planner to determine orientation 
                    * self._rl_scenario_manager.end_distance     --> current distance to navigation goal  (dist(current,end))
                    * self._rl_scenario_manager.total_distance   --> total route distance to navigation goal (dist(start,end))
                    * self._rl_scenario_manager.end_timeout      --> timeout for entire route provided by planner.
                    ** carla_env interface 
                    self._stuck = self._stuck_detector.stuck   --> if speed buffer has a low avg value then the vehicle 
                                                                    is determined to be stucked. This shold also be implemented
                                                                    in planner/agent class, so carla_env can directly use it.
                                                       
    2. Implementation Notes:      
        self._collided = self._simulator.collided --> use collide sensor to determine colllision
        self._ran_light = self._simulator.ran_light --> use traffic light helper to determine if ran light
    
    3. Next note:   
       Double check function in planner and carla simulator. Implement all functions that involves vehicle
       state. Use new class (vehicle manager and behavioral agent) to get reward related info (state, traffic, 
       direction, stuck detec etc).
    
    """