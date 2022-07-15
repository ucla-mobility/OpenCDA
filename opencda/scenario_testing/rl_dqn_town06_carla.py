# -*- coding: utf-8 -*-
"""
Scenario testing: single CAV navigation RL task
with V2V lidar, RSU and onboard sensor capability 
"""
# Author: Xu Han 
# License: TDG-Attribution-NonCommercial-NoDistrib

from opencda.core.ml_libs.rl.rl_api import rl_train, rl_eval, rl_test


def run_scenario(opt, config_yaml):
    if opt.rl_func == 'train':
        # start rl training
        rl_train(opt, config_yaml)
        print('RL train function complete...')
    elif opt.rl_func == 'test':
        # start rl training
        rl_eval(opt, config_yaml)
        print('RL evaluation complete...')
    elif opt.rl_func == 'eval':
        # start rl training
        rl_train(opt, config_yaml)
        print('RL testing complete...')
