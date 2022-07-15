# -*- coding: utf-8 -*-
"""
Scenario testing: single CAV navigation RL task
with V2V lidar, RSU and onboard sensor capability 
"""
# Author: Xu Han 
# License: TDG-Attribution-NonCommercial-NoDistrib
import sys

from opencda.core.ml_libs.rl.rl_api import rl_train, rl_eval, rl_test


def run_scenario(opt, config_yaml):
    if opt.rl_func == 'train':
        # start rl training
        rl_train(opt, config_yaml)
        print('RL train function complete...')
    else:
        sys.exit("Test and evaluation not implemented yet.")
