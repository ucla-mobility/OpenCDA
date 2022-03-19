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

from opencda.core.ml_libs.rl.simulators import CarlaSimulator
from opencda.core.ml_libs.rl.utils.env_utils.stuck_detector import StuckDetector
from opencda.core.ml_libs.rl.utils.simulator_utils.carla_utils import lane_mid_distance

# -------------- simple carla env default config ----------------
metadata = {'render.modes': ['rgb_array']}
action_space = spaces.Dict({})
observation_space = spaces.Dict({})
reward_type = ['goal', 'distance', 'speed', 'angle', 'steer', 'lane', 'failure']
config = dict(
    simulator=dict(),
    # reward types total reward take into account
    reward_type=['goal', 'distance', 'speed', 'angle', 'failure'],
    # reward value if success
    success_reward=10,
    # failure judgement
    col_is_failure=False,
    stuck_is_failure=False,
    ignore_light=False,
    ran_light_is_failure=False,
    off_road_is_failure=False,
    wrong_direction_is_failure=False,
    off_route_is_failure=False,
    # failure judgement hyper-parameters
    off_route_distance=6,
    success_distance=5,
    stuck_len=300,
    max_speed=5,
    # whether open visualize
    visualize=None,
)
# ---------------------------------------------------------------

class SimpleCarlaEnv(gym.Env, utils.EzPickle):
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
            carla_timeout: Optional[int] = 60.0,
            **kwargs,
    ) -> None:

        # ------------ copy from base class -------------
        if 'cfg_type' not in cfg:
            self._cfg = self.__class__.default_config()
            self._cfg = deep_merge_dicts(self._cfg, cfg)
        else:
            self._cfg = cfg
        utils.EzPickle.__init__(self)
        # ----------------------------------------------
        """
        Initialize environment with config and Carla TCP host & port.
        """
        # simulation parameters
        self._simulator_cfg = self._cfg.simulator
        self._carla_host = host
        self._carla_port = port
        self._carla_tm_port = tm_port
        self._carla_timeout = carla_timeout

        self._use_local_carla = False
        if self._carla_host != 'localhost':
            self._use_local_carla = True
        self._simulator = None

        # rl related configurations
        # Failure Config
        self._col_is_failure = self._cfg.col_is_failure                             # whether to consider collision as failure
        self._stuck_is_failure = self._cfg.stuck_is_failure                         # whether to consider getting stucked on road as failure
        self._ignore_light = self._cfg.ignore_light                                 # whether to ignore traffic light
        self._ran_light_is_failure = self._cfg.ran_light_is_failure                 # whether to consider running red light as failure
        self._off_road_is_failure = self._cfg.off_road_is_failure                   # whether to consider running off road as failure
        self._wrong_direction_is_failure = self._cfg.wrong_direction_is_failure     # whether to consider driving in wrong direction as failure
        self._off_route_is_failure = self._cfg.off_route_is_failure                 # whether to consider driving off planned route as failure
        self._off_route_distance = self._cfg.off_route_distance                     # distance to route threshold to mark current episode as failure
        # reward config
        self._reward_type = self._cfg.reward_type
        assert set(self._reward_type).issubset(self._reward_type), \
            set(self._reward_type)
        self._success_distance = self._cfg.success_distance                         # distance to navigation target threshold to determine success
        self._success_reward = self._cfg.success_reward                             # numerical reward for a successful navigation
        self._max_speed = self._cfg.max_speed                                       # maximum allowed speed
        self._collided = False                                                      # take penalty for collision
        self._stuck = False                                                         # take penalty for stucking in traffic
        self._ran_light = False                                                     # take penalty for running red light
        self._off_road = False                                                      # take penalty for running off road
        self._wrong_direction = False                                               # take penalty for driving in wrong direction
        self._off_route = False                                                     # take penalty for driving off planned route
        self._stuck_detector = StuckDetector(self._cfg.stuck_len)
        # time configs
        self._tick = 0
        self._timeout = float('inf')
        self._launched_simulator = False
        # visualization configs
        self._visualize_cfg = self._cfg.visualize
        self._simulator_databuffer = dict()
        self._visualizer = None
        self._last_canvas = None

        # import the action list
        self._acc_list = self._cfg.ACC_LIST
        self._steer_list = self._cfg.STEER_LIST

        # configurate discrete or continuous env
        self._env_is_discrete = False
        self._env_is_continuous = False

        # accumulated final reward
        self._final_eval_reward = 0.0

        # make sure the two settings are exclusive
        if 'env_action_space' in self._cfg:
            self._env_is_discrete = True if (
                    self._cfg['env_action_space'] == 'discrete' and not self._env_is_continuous) else False
            self._env_is_continuous = True if (
                    self._cfg['env_action_space'] == 'continuous' and not self._env_is_discrete) else False

    def _init_carla_simulator(self) -> None:
        if not self._use_local_carla:
            print("------ Run Carla on Port: %d, GPU: %d ------" % (self._carla_port, 0))
            #self.carla_process = subprocess.Popen()
            self._simulator = CarlaSimulator(
                cfg=self._simulator_cfg,
                client=None,
                host=self._carla_host,
                port=self._carla_port,
                tm_port=self._carla_tm_port,
                timeout=self._carla_timeout,
            )
        else:
            print('------ Using Remote carla @ {}:{} ------'.format(self._carla_host, self._carla_port))
            self._simulator = CarlaSimulator(
                cfg=self._simulator_cfg,
                client=None,
                host=self._carla_host,
                port=self._carla_port,
                tm_port=self._carla_tm_port
            )
        self._launched_simulator = True

    def discrete_action_from_num(self, np_action):
        #  convert action from numpy to number
        if isinstance(np_action, torch.Tensor):
            np_action = np_action.item()
        action_index = np.squeeze(np_action)
        assert action_index < len(self._acc_list) * len(self._steer_list), (action_index, len(self._acc_list) * len(self._steer_list))

        # find corresponding discrete action combination
        mod_value = len(self._acc_list)
        acc = self._acc_list[action_index % mod_value]
        steer = self._steer_list[action_index // mod_value]
        action = {
            'steer': steer,
            'throttle': acc[0],
            'brake': acc[1],
        }
        return action

    def continuous_action_from_num(self, action_ids):
        action_ids = to_ndarray(action_ids, dtype=int)
        action_ids = np.squeeze(action_ids)
        acc_id = action_ids[0]
        steer_id = action_ids[1]
        assert acc_id < len(self._acc_list), (acc_id, len(self._acc_list))
        assert steer_id < len(self._steer_list), (steer_id, len(self._steer_list))
        acc = self._acc_list[acc_id]
        steer = self._steer_list[steer_id]
        action = {
            'steer': steer,
            'throttle': acc[0],
            'brake': acc[1],
        }
        return action

    # print step
    def print_step(self, info):
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
        elif info['stuck']:
            done_state = "Stuck"
        elif info['timeout']:
            done_state = "Timeout"
        else:
            done_state = 'None'
        print(
            "[ENV] {} done with tick: {}, state: {}, reward: {}".format(
                repr(self), done_tick, done_state, done_reward
            )
        )

    # DI-engine required function
    def reset(self, **kwargs) -> Dict:
        """
        Reset environment to start a new episode, with provided reset params. If there is no simulator, this method will
        create a new simulator instance. The reset param is sent to simulator's ``init`` method to reset simulator,
        then reset all statues recording running states, and create a visualizer if needed. It returns the first frame
        observation.

        :Returns:
            Dict: The initial observation.
        """
        if not self._launched_simulator:
            self._init_carla_simulator()

        self._simulator.init(**kwargs)

        # if self._visualize_cfg is not None:
        #     if self._visualizer is not None:
        #         self._visualizer.done()
        #     else:
        #         self._visualizer = Visualizer(self._visualize_cfg)
        #
        #     if 'name' in kwargs:
        #         vis_name = kwargs['name']
        #     else:
        #         vis_name = "{}_{}".format(
        #             self._simulator.town_name, time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        #         )
        #
        #     self._visualizer.init(vis_name)

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
        self._timeout = self._simulator.end_timeout
        # reset episodic reward
        self._final_eval_reward = 0.0

        # # add configuration for discrete and continous
        # self._env_is_discrete = False
        # self._env_is_continuous = False

        # reshape obs_out for reset (discrete/continuous wrapper)
        obs = self.get_observations()
        obs_out = {
            'birdview': obs['birdview'][..., [0, 1, 5, 6, 8]],
            'speed': (obs['speed'] / 25).astype(np.float32),
        }

        # reshape obs_out (benchmark wrapper)
        obs_out = to_ndarray(obs_out, dtype=np.float32)
        if isinstance(obs_out, np.ndarray) and len(obs.shape) == 3:
            obs_out = obs_out.transpose((2, 0, 1))

        return obs_out

    # DI-engine required function
    def step(self, action) -> Tuple[Any, float, bool, Dict]:
        """
        Run one time step of environment, get observation from simulator and calculate reward. The environment will
        be set to 'done' only at success or failure. And if so, all visualizers will end. Its interfaces follow
        the standard definition of ``gym.Env``.

        :Arguments:
            - action (Dict): Action provided by policy.

        :Returns:
            Tuple[Any, float, bool, Dict]: A tuple contains observation, reward, done and information.
        """
        # cast action to np array
        action = to_ndarray(action)
        # reshape action
        if self._env_is_discrete:
            action_dict = self.discrete_action_from_num(action)
            action = action_dict
        if self._env_is_continuous:
            action_dict = self.continuous_action_from_num(action)
            action = action_dict

        # clip the action values
        if action is not None:
            for key in ['throttle', 'brake']:
                if key in action:
                    np.clip(action[key], 0, 1)
            if 'steer' in action:
                np.clip(action['steer'], -1, 1)
            self._simulator.apply_control(action)
            self._simulator_databuffer['action'] = action
        else:
            self._simulator_databuffer['action'] = dict()

        self._simulator.run_step()
        self._tick += 1

        obs = self.get_observations()

        self._collided = self._simulator.collided
        self._stuck = self._stuck_detector.stuck
        self._ran_light = self._simulator.ran_light
        self._off_road = self._simulator.off_road
        self._wrong_direction = self._simulator.wrong_direction

        location = self._simulator_databuffer['state']['location'][:2]
        target = self._simulator_databuffer['navigation']['target']
        self._off_route = np.linalg.norm(location - target) >= self._off_route_distance

        self._reward, reward_info = self.compute_reward()
        info = self._simulator.get_information()
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

        done = self.is_success() or self.is_failure()
        # if done:
        #     self._simulator.clean_up()
        #     if self._visualizer is not None:
        #         self._visualizer.done()
        #         self._visualizer = None

        # regulate observation (discrete/continuous wrapper)
        obs_out = {
            'birdview': obs['birdview'][..., [0, 1, 5, 6, 8]],
            'speed': (obs['speed'] / 25).astype(np.float32),
        }

        # original return
        # return obs_out, self._reward, done, info

        # prepare Base Env step (benchmark wrapper)
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
        Interface of ``info`` method to suit DI-engine format env.
        It returns a named tuple ``BaseEnvInfo`` defined in DI-engine
        which contains information about observation, action and reward space.

        :Returns:
            BaseEnvInfo: Env information instance defined in DI-engine.
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
        Delete simulator & visualizer instances and close the environment.
        """
        if self._launched_simulator:
            self._simulator.clean_up()
            self._simulator._set_sync_mode(False)
            del self._simulator
            self._launched_simulator = False
        # if self._visualizer is not None:
        #     self._visualizer.done()
        #     self._visualizer = None

    def is_success(self) -> bool:
        """
        Check if the task succeed. It only happens when hero vehicle is close to target waypoint.

        :Returns:
            bool: Whether success.
        """
        if self._simulator.end_distance < self._success_distance:
            return True
        return False

    def is_failure(self) -> bool:
        """
        Check if env fails. colliding, being stuck, running light, running off road, running in
        wrong direction according to config. It will certainly happen when time is out.

        :Returns:
            bool: Whether failure.
        """
        if self._stuck_is_failure and self._stuck:
            return True
        if self._col_is_failure and self._collided:
            return True
        if self._ran_light_is_failure and self._ran_light:
            return True
        if self._off_road_is_failure and self._off_road:
            return True
        if self._wrong_direction_is_failure and self._wrong_direction:
            return True
        if self._off_route_is_failure and self._off_route:
            return True
        if self._tick > self._timeout:
            return True

        return False

    def get_observations(self) -> Dict:
        """
        Get observations from simulator. The sensor data, navigation, state and information in simulator
        are used, while not all these are added into observation dict.

        :Returns:
            Dict: Observation dict.
        """
        obs = dict()
        state = self._simulator.get_state()
        navigation = self._simulator.get_navigation()
        sensor_data = self._simulator.get_sensor_data()
        information = self._simulator.get_information()

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
                'speed': np.float32(state['speed']),
                'speed_limit': np.float32(navigation['speed_limit']),
                'location': np.float32(state['location']),
                'forward_vector': np.float32(state['forward_vector']),
                'acceleration': np.float32(state['acceleration']),
                'velocity': np.float32(state['velocity']),
                'angular_velocity': np.float32(state['angular_velocity']),
                'rotation': np.float32(state['rotation']),
                'is_junction': np.float32(state['is_junction']),
                'tl_state': state['tl_state'],
                'tl_dis': np.float32(state['tl_dis']),
                'waypoint_list': navigation['waypoint_list'],
                'direction_list': navigation['direction_list'],
            }
        )

        # if self._visualizer is not None:
        #     if self._visualize_cfg.type not in sensor_data:
        #         raise ValueError("visualize type {} not in sensor data!".format(self._visualize_cfg.type))
        #     self._render_buffer = sensor_data[self._visualize_cfg.type].copy()
        #     if self._visualize_cfg.type == 'birdview':
        #         self._render_buffer = visualize_birdview(self._render_buffer)

        return obs

    def compute_reward(self) -> Tuple[float, Dict]:
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
        plan_distance = self._simulator.end_distance
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

        forward_vector = self._simulator_databuffer['state']['forward_vector']
        target_forward = self._simulator_databuffer['navigation']['target_forward']
        angle_reward = 3 * (0.1 - angle(forward_vector, target_forward) / np.pi)

        steer = self._simulator_databuffer['action'].get('steer', 0)
        command = self._simulator_databuffer['navigation']['command']
        steer_reward = 0.5
        if abs(steer - self._last_steer) > 0.5:
            steer_reward -= 0.2
        if command == 1 and steer > 0.1:
            steer_reward = 0
        elif command == 2 and steer < -0.1:
            steer_reward = 0
        elif (command == 3 or command == 4) and abs(steer) > 0.3:
            steer_reward = 0
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
        reward_info['angle_reward'] = angle_reward
        reward_info['steer_reward'] = steer_reward
        reward_info['lane_reward'] = lane_reward
        reward_info['failure_reward'] = failure_reward

        reward_dict = {
            'goal': goal_reward,
            'distance': distance_reward,
            'speed': speed_reward,
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

        :Returns:
            Any: visualized canvas, mainly used by tensorboard and gym monitor wrapper
        """
        # TODO: Disablr visualizer for the current version. Refer to DI-drive when adding visualization function.
        self._visualizer = None
        return self._visualizer.canvas

    def seed(self, seed: int, dynamic_seed: bool = True) -> None:
        """
        Set random seed for environment.
        :Arguments:
            - seed (int): Random seed value.
        """
        self._seed = seed
        self._dynamic_seed = dynamic_seed
        np.random.seed(self._seed)
        print('[ENV] Setting seed:', seed)
        # np.random.seed(seed)

    def __repr__(self) -> str:
        return "SimpleCarlaEnv - host %s : port %s" % (self._carla_host, self._carla_port)

    @property
    def hero_player(self) -> carla.Actor:
        return self._simulator.hero_player

    @classmethod
    def default_config(cls: type) -> EasyDict:
        # cfg = EasyDict(cls.config)
        cfg = EasyDict(config)
        cfg.cfg_type = cls.__name__ + 'Config'
        return copy.deepcopy(cfg)