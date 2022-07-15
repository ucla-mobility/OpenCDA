# -*- coding: utf-8 -*-
"""
Scenario testing: single CAV navigation RL task
with V2V lidar, RSU and onboard sensor capability 
"""
# Author: Xu Han 
# License: TDG-Attribution-NonCommercial-NoDistrib
import os
import argparse
import copy

import carla
import numpy as np
import torch
from functools import partial
from easydict import EasyDict
from tensorboardX import SummaryWriter

from ding.envs import SyncSubprocessEnvManager, BaseEnvManager
from ding.policy import DQNPolicy, TD3Policy
from ding.worker import BaseLearner, SampleSerialCollector, AdvancedReplayBuffer, NaiveReplayBuffer
from ding.utils import set_pkg_seed
from ding.rl_utils import get_epsilon_greedy_fn

import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml, save_yaml
from opencda.core.ml_libs.rl.envs.simple_carla_env_scenario_manager import CarlaRLEnv
from opencda.core.ml_libs.rl.utils.others.tcp_helper import parse_carla_tcp
from opencda.core.ml_libs.rl.rl_models import DQNRLModel, TD3RLModel
from opencda.core.ml_libs.rl.utils.others.ding_utils import compile_config
from opencda.core.ml_libs.rl.utils.others.ding_utils import read_ding_config
from opencda.core.ml_libs.rl.eval.single_carla_evaluator import SingleCarlaEvaluator
from opencda.core.ml_libs.rl.eval.carla_benchmark_evaluator import CarlaBenchmarkEvaluator


def init_carla_env(env_cfg, host, port, tm_port=None):
    """
    Initiate a Carla environment for rl agent.
    Parameters
    ----------
    env_cfg:dict
        The parsed configuration file.
    host:str
        The local host name.
    port:int
        The port for CARLA server.
    tm_port:int (optional)
        The tm port to run CARLA traffic manager. Default value is none as CARLA is able
        to generate new TM port automatically.

    Returns
    -------

    """
    env = CarlaRLEnv(env_cfg, host, port, tm_port)
    return env


def get_rl_policy(spec):
    """
    Get the desired RL policy and model based on configuration file.
    Parameters
    ----------
    spec:str
        The specific name of the desired policy, should be parsed from config file.
    Returns
    -------
    (policy_cls, model_cls): DI engine interfaces
        The DI-engine RL policy and RL model interface.

    """
    policy_cls, model_cls = {
        'dqn': (DQNPolicy, DQNRLModel),
        'td': (TD3Policy, TD3RLModel)
        # todo: add more models here
    }[spec]
    return policy_cls, model_cls


def get_rl_cfg(opt, default_train_config):
    """
    Get the structured configurations for DI-engine's environment manager.
    Parameters
    ----------
    opt:str
        The terminal argument passed in from opencda.py.
    default_train_config:dict
        The Default configuration file.

    Returns
    -------
    cfg:dict
        The structured configuration file, matches the format requirement of environment manager.
    """
    # read config
    default_train_config.exp_name = 'simple-rl'
    policy_name = default_train_config.policy.type
    opt_policy, _ = get_rl_policy(policy_name)
    # compile cfg
    cfg = compile_config(
        cfg=default_train_config,
        env_manager=SyncSubprocessEnvManager,
        policy=opt_policy,
        learner=BaseLearner,
        collector=SampleSerialCollector,
        buffer=AdvancedReplayBuffer,
    )
    return cfg


def rl_train(opt, config_yaml, seed=0):
    """
    Start the training loop for RL agent.
    Parameters
    ----------
    opt:dict
        The command line option parsed from opencda.py script.
    config_yaml:dict
        The configuration file for the current scenario.
    seed:int
        The current random seed.

    """
    # read configs 
    scenario_params = load_yaml(config_yaml)
    default_train_config = EasyDict(scenario_params['rl_config'])
    rl_cfg = get_rl_cfg(opt, default_train_config)
    policy_type = default_train_config.policy.type
    # regulate server
    tcp_list = parse_carla_tcp(rl_cfg.server)
    collector_env_num, evaluator_env_num = rl_cfg.env.collector_env_num, rl_cfg.env.evaluator_env_num
    assert len(tcp_list) >= collector_env_num + evaluator_env_num, \
        "Carla server not enough! Need {} servers but only found {}.".format(
            collector_env_num + evaluator_env_num, len(tcp_list)
        )

    # init env wrapper
    if policy_type == 'dqn':
        wrapped_env = init_carla_env

    # init rl environment with complete config_yaml
    collector_env = BaseEnvManager(
        env_fn=[partial(wrapped_env, scenario_params, *tcp_list[i]) for i in range(collector_env_num)],
        cfg=rl_cfg.env.manager.collect,
    )

    collector_env.seed(seed)
    print('Init carla rl env !')
    # evaluate_env.seed(seed)
    set_pkg_seed(seed)

    policy_cls, model_cls = get_rl_policy(policy_type)
    model = model_cls(**rl_cfg.policy.model)
    policy = policy_cls(rl_cfg.policy, model=model)

    # Switch to SummaryWriter to log training process.
    tb_logger = SummaryWriter('./opencda/scenario_testing/rl_log/{}/'.format(rl_cfg.exp_name))

    # initiate learner and collector
    learner = BaseLearner(rl_cfg.policy.learn.learner, policy.learn_mode, tb_logger, exp_name=rl_cfg.exp_name)
    collector = SampleSerialCollector(rl_cfg.policy.collect.collector, collector_env, policy.collect_mode, tb_logger,
                                      exp_name=rl_cfg.exp_name)

    print('Init leaner and collector!')
    # initiate replay buffer
    replay_buffer = NaiveReplayBuffer(rl_cfg.policy.other.replay_buffer, tb_logger, exp_name=rl_cfg.exp_name)

    # initiate epsilon greedy
    if policy_type == 'dqn':
        eps_cfg = rl_cfg.policy.other.eps
        epsilon_greedy = get_epsilon_greedy_fn(eps_cfg.start, eps_cfg.end, eps_cfg.decay, eps_cfg.type)

    learner.call_hook('before_run')

    # initiate replay buffer and push the first step
    if policy_type != 'ppo':
        if policy_type == 'dqn':
            eps = epsilon_greedy(collector.envstep)
            new_data = collector.collect(n_sample=10000, train_iter=learner.train_iter, policy_kwargs={'eps': eps})
            # print('---Current step is: ---')
            # print(new_data)
            # print('-----------------------')
        else:
            new_data = collector.collect(n_sample=10000, train_iter=learner.train_iter)

    replay_buffer.push(new_data, cur_collector_envstep=collector.envstep)

    learner.call_hook('after_run')

    collector.close()
    # evaluator.close()
    learner.close()
    if policy_type != 'ppo':
        replay_buffer.close()


def rl_eval(opt, config_yaml, seed=0):
    """
    Start the evaluating loop for RL agent.
    Parameters
    ----------
    opt:dict
        The command line option parsed from opencda.py script.
    config_yaml:dict
        The configuration file for the current scenario.
    seed:int
        The current random seed.

    """
    # read configs
    scenario_params = load_yaml(config_yaml)
    default_train_config = EasyDict(scenario_params['rl_config'])
    rl_cfg = get_rl_cfg(opt, default_train_config)
    policy_type = default_train_config.policy.type

    # regulate server
    tcp_list = parse_carla_tcp(rl_cfg.server)
    collector_env_num, evaluator_env_num = rl_cfg.env.collector_env_num, rl_cfg.env.evaluator_env_num
    assert len(tcp_list) >= collector_env_num + evaluator_env_num, \
        "Carla server not enough! Need {} servers but only found {}.".format(
            collector_env_num + evaluator_env_num, len(tcp_list)
        )

    # init env wrapper
    if policy_type == 'dqn':
        wrapped_env = init_carla_env

    # init carla env for evaluation
    eval_env = BaseEnvManager(
        env_fn=[partial(wrapped_env, scenario_params, *tcp_list[i]) for i in range(collector_env_num)],
        cfg=rl_cfg.env.manager.collect,
    )
    eval_env.seed(seed)
    print('Init carla eval env !')
    set_pkg_seed(seed)

    policy_cls, model_cls = get_rl_policy(policy_type)
    model = model_cls(**rl_cfg.policy.model)
    policy = policy_cls(rl_cfg.policy, model=model, enable_field=['eval'])

    # load Pre-train model
    if opt.rl_model_dir:
        pre_trained_model_dir = opt.rl_model_dir
        state_dict = torch.load(pre_trained_model_dir, map_location='cpu')
        policy.eval_mode.load_state_dict(state_dict)

    # init evaluator
    # todo: check evaluator and rewrite with opencda
    evaluator = CarlaBenchmarkEvaluator(rl_cfg.policy.eval.evaluator, eval_env, policy.eval_mode)
    success_rate = evaluator.eval()
    evaluator.close()


def rl_test(opt, config_yaml, seed=0):
    """
    Start the testing  loop for RL agent.
    Parameters
    ----------
    opt:dict
        The command line option parsed from opencda.py script.
    config_yaml:dict
        The configuration file for the current scenario.
    seed:int
        The current random seed.

    """
    # read configs
    scenario_params = load_yaml(config_yaml)
    default_train_config = EasyDict(scenario_params['rl_config'])
    rl_cfg = get_rl_cfg(opt, default_train_config)
    policy_type = default_train_config.policy.type

    # regulate server
    tcp_list = parse_carla_tcp(rl_cfg.server)
    collector_env_num, evaluator_env_num = rl_cfg.env.collector_env_num, rl_cfg.env.evaluator_env_num
    assert len(tcp_list) > 0, "No Carla server found!"
    # only test one single server
    host, port = tcp_list[0]
    # init test env
    test_env = CarlaRLEnv(env_cfg, host, port, tm_port)
    test_env.seed(seed)
    print('Init carla test env !')
    set_pkg_seed(seed)

    # init model and policy
    policy_cls, model_cls = get_rl_policy(policy_type)
    model = model_cls(**rl_cfg.policy.model)
    policy = policy_cls(rl_cfg.policy, model=model, enable_field=['eval'])

    # load Pre-train model
    if opt.rl_model_dir:
        pre_trained_model_dir = opt.rl_model_dir
        state_dict = torch.load(pre_trained_model_dir, map_location='cpu')
        policy.eval_mode.load_state_dict(state_dict)

    # init evaluator
    # todo: check evaluator and rewrite with opencda
    evaluator = SingleCarlaEvaluator(cfg.policy.eval.evaluator, eval_env, policy.eval_mode)
    success_rate = evaluator.eval()
    evaluator.close()
