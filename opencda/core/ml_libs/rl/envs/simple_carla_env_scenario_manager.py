# -*- coding: utf-8 -*-
"""
Utilize scenario manager to manage CARLA simulation construction. This script
is used for carla simulation only, and if you want to manage the Co-simulation,
please use cosim_api.py.
"""
# Author: Xu Han
# License: TDG-Attribution-NonCommercial-NoDistrib

import os
import time
import math
import copy
from abc import ABC, abstractmethod
from collections import defaultdict
from typing import Any, Dict, Optional, Tuple

import gym
import carla
import torch
import numpy as np
from gym import spaces, utils
from easydict import EasyDict

from ding.utils.default_helper import deep_merge_dicts
from ding.envs.env.base_env import BaseEnvTimestep, BaseEnvInfo
from ding.envs.common.env_element import EnvElementInfo
from ding.torch_utils.data_helper import to_ndarray

from opencda.core.common.cav_world import CavWorld
from opencda.core.common.rl_state_manager import RLStateManager
from opencda.core.ml_libs.rl.simulators.rl_scenario_manager import RLScenarioManager
from opencda.core.ml_libs.rl.utils.env_utils.stuck_detector import StuckDetector
from opencda.core.ml_libs.rl.utils.simulator_utils.carla_utils import lane_mid_distance
from opencda.core.ml_libs.rl.utils.others.visualizer import Visualizer
from opencda.core.ml_libs.rl.utils.simulator_utils.carla_utils import visualize_birdview

metadata = {'render.modes': ['rgb_array']}
action_space = spaces.Dict({})
observation_space = spaces.Dict({})


class CarlaRLEnv(gym.Env, utils.EzPickle):
    """
        A simple deployment of Carla Environment with single hero vehicle. It use ``CarlaSimulator`` to interact with
        Carla server and gets running status. The observation is obtained from simulator's state, information and
        sensor data, along with reward which can be calculated and retrived.

        When created, it will initialize environment with config and Carla TCP host & port. This method will NOT create
        simulator instance. It only creates some data structures to store information when running env.

        Parameters
        ----------
        cfg : Dict
            Env config dict.

        host : str, optional
            Carla server IP host. Defaults to 'localhost'.

        port : int, optional
            Carla server IP port. Defaults to 9000.

        tm_port : int, optional
            Carla Traffic Manager port. Defaults to None.
        """

    def __init__(
            self,
            cfg: Dict,
            host: str = 'localhost',
            port: int = 9000,
            tm_port: Optional[int] = None,
            carla_timeout: Optional[int] = 60.0
    ) -> None:
        """
        Initialize environment with config and Carla TCP host & port.
        """
        # load config
        self._cfg = EasyDict(cfg)
        self._rl_config = self._cfg.rl_config
        self._is_v2v = self._rl_config.env.activate_v2v
        # simulation parameters
        self._world_config = self._cfg['world']
        self._simulator_cfg = self._cfg['rl_config']['env']['simulator']
        self._carla_host = host
        self._carla_port = port
        self._carla_tm_port = tm_port
        self._carla_timeout = carla_timeout

        self._use_local_carla = False
        if self._carla_host != 'localhost':
            self._use_local_carla = True

        # OpenCDA Simulator interfaces
        self._cav_world = None
        self._rl_scenario_manager = None
        self._rl_state_manager = None
        self._rsu_list = None
        self._tm = None
        self._bg_list = None
        self._single_cav_list = None
        self._hero_vehicle = None
        self._hero_vehicle_manager = None
        self._v2v = self._cfg['rl_config']['env']['v2v']
        # visualization configs
        self._visualize_cfg = self._cfg['rl_config']['env']['visualize']
        self._simulator_databuffer = dict()
        self._visualizer = None
        self._last_canvas = None

        # Scenario Configs
        # rl related configurations
        self._rl_action_dt = self._cfg['rl_config']['action_delta_t']   # temporary value == 1
        self._carla_town_name = None
        self._start_location = None
        self._end_location = None
        self._bev_wrapper = None
        # Failure Config
        self._col_is_failure = self._rl_config.env.col_is_failure  # whether to consider collision as failure
        self._stuck_is_failure = self._rl_config.env.stuck_is_failure  # whether to consider getting stucked on road as failure
        self._ignore_light = self._rl_config.env.ignore_light  # whether to ignore traffic light
        self._ran_light_is_failure = self._rl_config.env.ran_light_is_failure  # whether to consider running red light as failure
        self._off_road_is_failure = self._rl_config.env.off_road_is_failure  # whether to consider running off road as failure
        self._wrong_direction_is_failure = self._rl_config.env.wrong_direction_is_failure  # whether to consider driving in wrong direction as failure
        self._off_route_is_failure = self._rl_config.env.off_route_is_failure  # whether to consider driving off planned route as failure
        self._off_route_distance = self._rl_config.env.off_route_distance  # distance to route threshold to mark current episode as failure
        # reward config
        self._reward = 0
        self._last_steer = 0
        self._last_distance = None
        self._reward_type = self._rl_config.env.reward_type
        assert set(self._reward_type).issubset(self._reward_type), \
            set(self._reward_type)
        self._success_distance = self._rl_config.env.success_distance  # distance to navigation target threshold to determine success
        self._success_reward = self._rl_config.env.success_reward  # numerical reward for a successful navigation
        self._max_speed = self._rl_config.env.max_speed  # maximum allowed speed
        self._collided = False  # take penalty for collision
        self._stuck = False  # take penalty for stuck in traffic
        self._ran_light = False  # take penalty for running red light
        self._off_road = False  # take penalty for running off-road
        self._wrong_direction = False  # take penalty for driving in wrong direction
        self._off_route = False  # take penalty for driving off planned route
        self._stuck_detector = StuckDetector(self._rl_config.env.stuck_len)
        # accumulated final reward
        self._final_eval_reward = 0.0
        # time configs
        self._tick = 0
        self._timeout = float('inf')
        self._launched_simulator = False

        # Action configs
        # import the action list
        self._acc_list = self._rl_config.env.ACC_LIST
        self._steer_list = self._rl_config.env.STEER_LIST
        # configurate discrete or continuous env
        self._env_is_discrete = True if (
                self._rl_config.env.env_action_space == 'discrete') else False
        self._env_is_continuous = not self._env_is_discrete

    def _init_carla_simulator(self) -> None:
        """
        Initialize CARLA simulator, CAV world and vehicle manager. Spawn all vheicles in simulation world
        including CAVs and background vehicles.
        """
        # init CAV world
        cav_world = CavWorld()

        # init rl scenario manager
        # overwrite port info in scenario params so the CARLA client is assign to the correct port
        scenario_params = self._cfg
        scenario_params['world']['client_port'] = self._carla_port
        self._carla_town_name = self._simulator_cfg['town']
        self._rl_scenario_manager = RLScenarioManager(scenario_params,
                                                      cfg=self._simulator_cfg,
                                                      town=self._carla_town_name,
                                                      cav_world=cav_world)
        # Display status
        print("------ Run Carla on Port: %d, GPU: %d ------" % (self._carla_port, 0))
        print("------ Initiate Scenario Manager at Map: %s ------" % self._carla_town_name)
        # set simulation launch indicator
        self._launched_simulator = True
        self._cav_world = self._rl_scenario_manager.world

    def action_to_waypoint(self, np_action):
        """
        A function to map RL action to a discrete target waypoint.
        Parameters. There are 9 actions total, we devide the target waypoint
        to left, same and right area and each area contains 9 candidate points.
        ----------
        np_action:int
            The output of the RL model.

        Returns
        -------
        waypoint:Carla.Waypoint
            The corresponding target waypoint.
        """
        #  find ego waypoint
        ego_waypoint = self._hero_vehicle_manager.agent.get_current_waypoint()
        egp_speed = self._hero_vehicle_manager.agent.get_ego_speed()
        # action_delta_t default value is 1 (if vehicle standstill, use 3m/s as default)
        ds = max(egp_speed, 3) / 3.6 * self._rl_action_dt

        def get_next(waypoint, dist):
            return waypoint.next(dist)[0] if waypoint.next(dist) else None

        next_waypoint_mid = get_next(ego_waypoint, ds)
        # debug visualization
        # start
        self._cav_world.debug.draw_string(ego_waypoint.transform.location, 'x',
                                          color=carla.Color(r=0, g=255, b=255),
                                          life_time=1.0,
                                          persistent_lines=True)
        # target
        self._cav_world.debug.draw_string(next_waypoint_mid.transform.location, 'x',
                                          color=carla.Color(r=0, g=255, b=0),
                                          life_time=1.0,
                                          persistent_lines=True)
        # testing draw point at next watpoint
        # todo: testing mid only
        return next_waypoint_mid

        # todo: revert whole function after testing ------------------------------------------
        # if next_waypoint_mid:
        #     next_waypoint_left = next_waypoint_mid.get_left_lane() if next_waypoint_mid.get_left_lane() else None
        #     next_waypoint_right = next_waypoint_mid.get_right_lane() if next_waypoint_mid.get_right_lane() else None
        # else:
        #     next_waypoint_left = None
        #     next_waypoint_right = None
        #
        # # convert np_action (action_shape = 9) to waypoint
        # candidate_waypoints = np.array([
        #     [next_waypoint_left, get_next(next_waypoint_left, ds), get_next(next_waypoint_left, 2*ds)],
        #     [next_waypoint_mid, get_next(next_waypoint_mid, ds), get_next(next_waypoint_mid, 2*ds)],
        #     [next_waypoint_right, get_next(next_waypoint_right, ds), get_next(next_waypoint_right, 2*ds)] ])
        #
        # target_waypoint = candidate_waypoints[np_action // 3, np_action % 3]
        #
        # return target_waypoint
        # todo: revert whole function after testing ------------------------------------------
    # print step
    def print_step(self, info):
        """
        An  utility function to print step information.
        """
        done_tick = info['tick']
        done_reward = info['final_eval_reward']
        if info['success']:
            done_state = 'Success'
        elif info['collided']:
            done_state = "Collided"
        elif info['wrong_direction']:
            done_state = "Wrong Direction"
        elif info['off_road']:
            done_state = "Off road"
        elif info['off_route']:
            done_state = "Off route"
        elif info['stuck']:
            done_state = "Stuck"
        elif info['timeout']:
            done_state = "Timeout"
        elif info['ran_light']:
            done_state = "ran red light"
        else:
            done_state = 'None'
        print(
            "[ENV] {} done with tick: {}, finish state: {}, episodic reward: {}".format(
                repr(self), done_tick, done_state, done_reward
            )
        )

        print('Add this line for debug...')

    def get_observations(self):
        '''
        Get the observations from simulator. The sensor data,
        navigation, state and information in simulator are examined.

        Returns
        -------
        obs:dict
            The current observation of the simulation step.
        '''
        obs = dict()
        # 1. update vehicle manager
        self._hero_vehicle_manager.update_info()
        # 2. read vehicle state
        # vehicle_state = self._hero_vehicle_manager.rl_manager.get_vehicle_state()
        vehicle_state = self._rl_state_manager.get_vehicle_state()
        # 3. read scenario state
        scenario_state = self._rl_scenario_manager.get_state()
        # 4. update off road status
        self._off_road = self._rl_scenario_manager.is_vehicle_off_road()
        # self._hero_vehicle_manager.rl_manager.set_off_road(self._off_road)
        self._rl_state_manager.set_off_road(self._off_road)
        # 5. read navigation state
        # navigation, waypoint_list = self._hero_vehicle_manager.rl_manager.get_navigation_state()
        navigation, waypoint_list = self._rl_state_manager.get_navigation_state()
        # 6. update bev wrapper
        self._rl_scenario_manager.update_bev_waypoints(waypoint_list)
        sensor_data = self._rl_scenario_manager.get_sensor_data()
        information = self._rl_scenario_manager.get_information()
        # 7. update data buffer
        state = {**vehicle_state, **scenario_state}  # merge the two states from scenario/rl manager
        self._simulator_databuffer['state'] = state
        self._simulator_databuffer['navigation'] = navigation
        self._simulator_databuffer['information'] = information

        if 'action' not in self._simulator_databuffer:
            self._simulator_databuffer['action'] = dict()
        if not navigation['agent_state'] == 4 or self._ignore_light:
            self._stuck_detector.tick(state['speed'])

        obs.update(sensor_data)
        obs.update(
            {
                'tick': information['tick'],
                'timestamp': np.float32(information['timestamp']),
                'agent_state': navigation['agent_state'],
                'node': navigation['node'],
                'node_forward': navigation['node_forward'],
                'target': np.float32(navigation['target']),
                'target_forward': np.float32(navigation['target_forward']),
                'command': navigation['command'],
                'speed': np.float32(vehicle_state['speed']),
                'speed_limit': np.float32(navigation['speed_limit']),
                'location': np.float32(vehicle_state['location']),
                'forward_vector': np.float32(vehicle_state['forward_vector']),
                'acceleration': np.float32(vehicle_state['acceleration']),
                'velocity': np.float32(vehicle_state['velocity']),
                'angular_velocity': np.float32(vehicle_state['angular_velocity']),
                'rotation': np.float32(vehicle_state['rotation']),
                'is_junction': np.float32(scenario_state['is_junction']),
                'tl_state': scenario_state['tl_state'],
                'tl_dis': np.float32(scenario_state['tl_dis']),
                'waypoint_list': navigation['waypoint_list'],
                'direction_list': navigation['direction_list'],
            }
        )

        if self._visualizer is not None:
            if self._visualize_cfg.type not in sensor_data:
                raise ValueError("visualize type {} not in sensor data!".format(self._visualize_cfg.type))
            self._render_buffer = sensor_data[self._visualize_cfg.type].copy()
            if self._visualize_cfg.type == 'birdview':
                self._render_buffer = visualize_birdview(self._render_buffer)

        return obs

    # DI-engine required function
    def reset(self, **kwargs) -> Dict:
        """
        Reset environment to start a new episode, with provided reset params. If there is no simulator, this method will
        create a new simulator instance. The reset param is sent to simulator's ``init`` method to reset simulator,
        then reset all statues recording running states, and create a visualizer if needed. It returns the first frame
        observation.

        Returns
        -------
        obs:ndarray
            The initial observation of the simulation world.

        """

        if not self._launched_simulator:
            self._init_carla_simulator()

        # kill all actors in world
        live_actors = self._cav_world.get_actors()
        if self._single_cav_list is not None:
            self._rl_scenario_manager.destroyActors()
            self._single_cav_list = None
            self._bg_list = None
            for actor in live_actors:
                actor.destroy()

        # 1. init managers
        is_data_dump = self._rl_config.env.v2v
        self._single_cav_list = self._rl_scenario_manager.create_vehicle_manager(application=['rl'],
                                                                                 data_dump=is_data_dump)
        # RSU module
        if 'rsu_base' in self._cfg.keys():
            self._rsu_list = \
                self._rl_scenario_manager.create_rsu_manager(data_dump=is_data_dump)

        # background traffic
        if 'carla_traffic_manager' in self._cfg.keys():
            traffic_manager, bg_veh_list = self._rl_scenario_manager.create_traffic_carla()

        # RL module
        self._rl_state_manager = RLStateManager(self._single_cav_list,
                                                self._rl_config)
        # update attributes
        # self._single_cav_list = single_cav_list
        # always set the first single CAV as ego vehicle
        self._hero_vehicle_manager = self._single_cav_list[0]
        self._hero_vehicle = self._single_cav_list[0].vehicle

        # 2. prepare sensors (# init sensor helper, collision sensor and traffic light sensor)
        self._rl_scenario_manager.prepare_observations(self._hero_vehicle)

        # 3. set destination and update total distance
        self._rl_state_manager.set_total_distance()

        # warmup simulation for 10 ticks
        for _ in range(10):
            # 1. vehicle manager run step
            for i, single_cav in enumerate(self._single_cav_list):
                single_cav.update_info()
                # control = single_cav.run_step()
                # single_cav.vehicle.apply_control(control)
            # 2.scenario run step
            self._rl_scenario_manager.run_step()
            # 3. update RSU
            if 'rsu_base' in self._cfg.keys():
                for rsu in self._rsu_list:
                    rsu.update_info()
                    rsu.run_step()
            # 4. update states
            # self._hero_vehicle_manager.rl_manager.get_vehicle_state()
            self._rl_state_manager.get_vehicle_state()
            self._rl_scenario_manager.get_state()
            self._off_road = self._rl_scenario_manager.is_vehicle_off_road()
            #  self._hero_vehicle_manager.rl_manager.set_off_road(self._off_road)
            self._rl_state_manager.set_off_road(self._off_road)
            # self._hero_vehicle_manager.rl_manager.get_navigation_state()
            self._rl_state_manager.get_navigation_state()
            # 5. increase one tick
            self._tick += 1
            # 6. tick carla
            self._cav_world.tick()

        # organize initial observation
        if self._visualize_cfg is not None:
            if self._visualizer is not None:
                self._visualizer.done()
            else:
                self._visualizer = Visualizer(self._visualize_cfg)

            if 'name' in kwargs:
                vis_name = kwargs['name']
            else:
                vis_name = "{}_{}".format(
                    self._carla_town_name, time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
                )

            self._visualizer.init(vis_name)

        if 'col_is_failure' in kwargs:
            self._col_is_failure = kwargs['col_is_failure']
        if 'stuck_is_failure' in kwargs:
            self._stuck_is_failure = kwargs['stuck_is_failure']
        self._simulator_databuffer.clear()
        self._collided = False
        self._stuck = False
        self._ran_light = False
        self._off_road = False
        self._wrong_direction = False
        self._off_route = False
        self._stuck_detector.clear()
        self._tick = 0
        self._reward = 0
        self._last_steer = 0
        self._last_distance = None
        # self._timeout = self._hero_vehicle_manager.rl_manager.get_sim_end_timeout()
        self._timeout = self._rl_state_manager.get_ego_end_timeout()
        # reset episodic reward
        self._final_eval_reward = 0.0

        # reshape obs_out for reset (discrete/continuous wrapper)
        obs = self.get_observations()
        # regulate obs shape
        obs_out = {
            'birdview': obs['birdview'][..., [0, 1, 5, 6, 8]],
            'speed': (obs['speed'] / 25).astype(np.float32),
        }
        # read lidar data
        if self._v2v:
            lidar_data = self._rl_state_manager.get_lidar_frames()
            obs_out.update(lidar_data)

        # reshape obs_out (benchmark wrapper)
        obs_out = to_ndarray(obs_out, dtype=np.float32)
        if isinstance(obs_out, np.ndarray) and len(obs.shape) == 3:
            obs_out = obs_out.transpose((2, 0, 1))

        return obs_out

    # DI-engine required function
    def step(self, action) -> Tuple[Any, float, bool, Dict]:
        """
        Run one step of RL environment, get observation from simulator and calculate reward. The environment will
        be set to 'done' upon success or failure, in which case all visualizers will end. The interfaces follow
        the standard ``gym.Env``.

        Parameters
        ----------
        action:dict
            The action provided by the policy.

        Returns
        -------
        :tuple
            A tuple contains observation, reward, done and information.
        """

        # clip the action values
        if action is not None:
            # update buffer
            self._simulator_databuffer['action'] = action.item()

        else:
            self._simulator_databuffer['action'] = dict()

        # convert action to target waypoint
        one_step_target_waypoint = self.action_to_waypoint(action.item())
        ego_vehicle_loc = self._hero_vehicle.get_location()
        # find target speed (note: temporarly using speed limit as a target speed, in m/s)
        target_speed = max(self._hero_vehicle.get_speed_limit(), 20)


        if one_step_target_waypoint is not None:
            # apply RL action (update target location)
            self._hero_vehicle_manager.agent.apply_rl_action(ego_vehicle_loc,
                                                             one_step_target_waypoint,
                                                             target_speed,
                                                             self._rl_action_dt)
            # run both reward step and action step
            control = self._hero_vehicle_manager.run_step()
            # apply ed and control
            self._hero_vehicle.apply_control(control)
        else:
            # agent selected out-of-range location, use current location as target and take penalty
            self._hero_vehicle_manager.agent.apply_rl_action(ego_vehicle_loc,
                                                             ego_vehicle_loc,
                                                             target_speed,
                                                             self._rl_action_dt)
            control = self._hero_vehicle_manager.runstep()
            # apply speed and control
            self._hero_vehicle.apply_control(control)


        # Run one simulation step for all interfaces
        # 1. vehicle manager update run step

        for i, single_cav in enumerate(self._single_cav_list):
            single_cav.update_info()
            # only run reward step
            single_cav.agent.run_rl_reward_step()
        # 2.scenario run step
        self._rl_scenario_manager.run_step()
        # 3. update RSU
        if 'rsu_base' in self._cfg.keys():
            for rsu in self._rsu_list:
                rsu.update_info()
                rsu.run_step()
        # 4. increase one tick
        self._tick += 1
        # 5. tick carla
        self._cav_world.tick()

        # get observation
        obs = self.get_observations()
        # determine success/failure
        self._collided = self._rl_scenario_manager.is_vehicle_collided()
        self._stuck = self._stuck_detector.stuck
        self._ran_light = self._rl_scenario_manager.is_vehicle_ran_light()
        self._off_road = self._rl_scenario_manager.is_vehicle_off_road()
        self._wrong_direction = self._rl_state_manager.is_vehicle_wrong_direction()

        location = self._simulator_databuffer['state']['location'][:2]
        target = self._simulator_databuffer['navigation']['target']

        # add calculation step for debug purpose
        veh_target_dist = np.linalg.norm(location - target)

        self._off_route = veh_target_dist >= self._off_route_distance

        self._reward, reward_info = self.compute_reward(one_step_target_waypoint)
        info = self._rl_scenario_manager.get_information()
        info.update(reward_info)
        info.update(
            {
                'collided': self._collided,
                'stuck': self._stuck,
                'ran_light': self._ran_light,
                'off_road': self._off_road,
                'wrong_direction': self._wrong_direction,
                'off_route': self._off_route,
                'timeout': self._tick > self._timeout,
                'success': self.is_success(),
            }
        )
        # determine if done
        done = self.is_success() or self.is_failure()
        if done:
            # reset scenario manager
            self._rl_scenario_manager.clean_up()
            if self._visualizer is not None:
                self._visualizer.done()
                self._visualizer = None

        # regulate observation
        obs_out = {
            'birdview': obs['birdview'][..., [0, 1, 5, 6, 8]],
            'speed': (obs['speed'] / 25).astype(np.float32),
        }
        # read and update lidar data
        if self._v2v:
            lidar_data = self._rl_state_manager.get_lidar_frames()
            obs_out.update(lidar_data)

        # update final episode reward
        self._final_eval_reward += self._reward
        # reshape obs
        obs_out = to_ndarray(obs_out, dtype=np.float32)
        if isinstance(obs_out, np.ndarray) and len(obs_out.shape) == 3:
            obs_out = obs_out.transpose((2, 0, 1))
        # reshape reward
        reward_out = to_ndarray([self._reward], dtype=np.float32)
        if done:
            info['final_eval_reward'] = self._final_eval_reward
            self.print_step(info)

        return BaseEnvTimestep(obs_out, reward_out, done, info)

    # DI-Engine required function
    def info(self) -> BaseEnvInfo:
        """
        Interface of ``info`` method to suit DI-engine requirement.
        It returns a named tuple ``BaseEnvInfo`` defined in DI-engine
        which contains information about observation, action and reward space.

        Returns
        -------
        :BaseEnvInfo
            Env information instance defined in DI-engine.
        """

        obs_space = EnvElementInfo(shape=self.env.observation_space, value={'min': 0., 'max': 1., 'dtype': np.float32})
        act_space = EnvElementInfo(
            shape=self.env.action_space,
            value={
                'min': np.float32("-inf"),
                'max': np.float32("inf"),
                'dtype': np.float32
            },
        )
        rew_space = EnvElementInfo(
            shape=1,
            value={
                'min': np.float32("-inf"),
                'max': np.float32("inf")
            },
        )
        return BaseEnvInfo(
            agent_num=1, obs_space=obs_space, act_space=act_space, rew_space=rew_space, use_wrappers=None
        )

    def close(self) -> None:
        """
        Stop the simulator and visualizer instances and close the environment.
        """
        if self._launched_simulator:
            self._rl_scenario_manager.clean_up()
            del self._rl_scenario_manager
            self._launched_simulator = False
        if self._visualizer is not None:
            self._visualizer.done()
            self._visualizer = None

    def is_success(self) -> bool:
        """
        Check if the task has succeeded. It is defined as when the host vehicle is close
        to the target location.

        Returns
        -------
        : bool
        Whether the episode is successful.
        """

        # if self._hero_vehicle_manager.rl_manager.get_end_distance() < self._success_distance:
        if self._rl_state_manager.get_end_distance() < self._success_distance:
            return True
        return False

    def is_failure(self) -> bool:
        """
        Check if the task has failed.The failure definition is listed in the config file.

        Returns
        -------
        : bool
        Whether the episode has failed.
        """
        # if self._stuck_is_failure and self._stuck:
        #     return True
        # if self._col_is_failure and self._collided:
        #     return True
        # if self._ran_light_is_failure and self._ran_light:
        #     return True
        # if self._off_road_is_failure and self._off_road:
        #     return True
        # if self._wrong_direction_is_failure and self._wrong_direction:
        #     return True
        # if self._off_route_is_failure and self._off_route:
        #     return True
        # if self._tick > self._timeout:
        #     return True

        return False

    def compute_reward(self, target_waypoint) -> Tuple[float, Dict]:
        """
        Compute reward for current step, whose details will be summarized in a dict.

        Returns
        -------
        :tuple[float, Dict]
            Total reward value and detail for each value.

        """
        """
        Compute reward for current frame, with details returned in a dict. In short, in contains goal reward,
        route following reward calculated by route length in current and last frame, some navigation attitude reward
        with respective to target waypoint, and failure reward by checking each failure event.

        :Returns:
            Tuple[float, Dict]: Total reward value and detail for each value.
        """

        def dist(loc1, loc2):
            return ((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2) ** 0.5

        def angle(vec1, vec2):
            cos = vec1.dot(vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2) + 1e-10)
            cos = np.clip(cos, -1, 1)
            angle = np.arccos(cos)
            angle = np.clip(angle, 0, np.pi)
            return angle

        # goal reward
        goal_reward = 0
        plan_distance = self._rl_state_manager.get_end_distance()
        if self.is_success():
            goal_reward += self._success_reward
        elif self.is_failure():
            goal_reward -= 1

        # distance reward
        location = self._simulator_databuffer['state']['location']
        target = self._simulator_databuffer['navigation']['target']
        target_distance = dist(target, location)
        cur_distance = plan_distance + target_distance
        if self._last_distance is None:
            distance_reward = 0
        else:
            distance_reward = np.clip((self._last_distance - cur_distance) * 2, 0, 1)
        self._last_distance = cur_distance

        # state reward: speed, angle, steer, mid lane
        speed = self._simulator_databuffer['state']['speed'] / 3.6
        speed_limit = self._simulator_databuffer['navigation']['speed_limit'] / 3.6
        speed_limit = min(self._max_speed, speed_limit)
        agent_state = self._simulator_databuffer['navigation']['agent_state']
        target_speed = speed_limit
        if agent_state == 2 or agent_state == 3:
            target_speed = 0
        elif agent_state == 4 and not self._ignore_light:
            target_speed = 0
        # speed_reward = 1 - abs(speed - target_speed) / speed_limit
        speed_reward = 0
        if speed < target_speed / 5:
            speed_reward -= 1
        if speed > target_speed:
            speed_reward -= 1
        # consider reward for action selection
        action_reward = 0
        if target_waypoint is None:
            action_reward -= 1

        forward_vector = self._simulator_databuffer['state']['forward_vector']
        target_forward = self._simulator_databuffer['navigation']['target_forward']
        angle_reward = 3 * (0.1 - angle(forward_vector, target_forward) / np.pi)

        steer = self._hero_vehicle.get_control().steer
        steer_reward = 0.5
        if abs(steer - self._last_steer) > 0.5:
            steer_reward -= 0.2
        self._last_steer = steer

        waypoint_list = self._simulator_databuffer['navigation']['waypoint_list']
        lane_mid_dis = lane_mid_distance(waypoint_list, location)
        lane_reward = -0.5 * lane_mid_dis

        failure_reward = 0
        if self._col_is_failure and self._collided:
            failure_reward -= 5
        elif self._stuck_is_failure and self._stuck:
            failure_reward -= 5
        elif self._off_road_is_failure and self._off_road:
            failure_reward -= 5
        elif self._ran_light_is_failure and not self._ignore_light and self._ran_light:
            failure_reward -= 5
        elif self._wrong_direction_is_failure and self._wrong_direction:
            failure_reward -= 5

        reward_info = {}
        total_reward = 0
        reward_info['goal_reward'] = goal_reward
        reward_info['distance_reward'] = distance_reward
        reward_info['speed_reward'] = speed_reward
        # add action reward
        reward_info['action_reward'] = action_reward
        reward_info['angle_reward'] = angle_reward
        reward_info['steer_reward'] = steer_reward
        reward_info['lane_reward'] = lane_reward
        reward_info['failure_reward'] = failure_reward

        reward_dict = {
            'goal': goal_reward,
            'distance': distance_reward,
            'speed': speed_reward,
            # add action reward
            'actin': action_reward,
            'angle': angle_reward,
            'steer': steer_reward,
            'lane': lane_reward,
            'failure': failure_reward
        }
        for rt in self._reward_type:
            total_reward += reward_dict[rt]

        return total_reward, reward_info

    def render(self, mode='rgb_array') -> Any:
        """
        Render a runtime visualization on screen, save a gif or video according to visualizer config.
        The main canvas is from a specific sensor data. It only works when 'visualize' is set in config dict.

        Parameters
        ----------
        mode:str
            The rendering mode of the visualizer.

        Returns
        -------
        :visualizer.canvas
            One frame of the visualozation.

        """
        if self._visualizer is None:
            return self._last_canvas

        render_info = {
            'collided': self._collided,
            'off_road': self._off_road,
            'wrong_direction': self._wrong_direction,
            'off_route': self._off_route,
            'reward': self._reward,
            'tick': self._tick,
            'end_timeout': self._rl_state_manager.get_ego_end_timeout(),
            'end_distance': self._rl_state_manager.get_end_distance(),
            'total_distance': self._rl_state_manager.get_total_distance(),
        }
        render_info.update(self._simulator_databuffer['state'])
        render_info.update(self._simulator_databuffer['navigation'])
        render_info.update(self._simulator_databuffer['information'])
        render_info.update(self._simulator_databuffer['action'])

        self._visualizer.paint(self._render_buffer, render_info)
        self._visualizer.run_visualize()
        self._last_canvas = self._visualizer.canvas
        return self._visualizer.canvas

    def seed(self, seed: int, dynamic_seed: bool = True) -> None:
        """
        Set random seed for environment.
        :Arguments:
            - seed (int): Random seed value.
        """
        if 'seed' in self._world_config:
            seed = self._world_config['seed']
            self._seed = seed
            self._dynamic_seed = dynamic_seed
            print('[ENV] Setting seed:', seed)
        else:
            print('No random seed provided, use ' + str(seed) + ' as random seed and update config...')
            self._world_config['seed'] = seed

    def __repr__(self) -> str:
        return "SimpleCarlaEnv - host %s : port %s" % (self._carla_host, self._carla_port)

    @property
    def hero_player(self) -> carla.Actor:
        return self._hero_vehicle
