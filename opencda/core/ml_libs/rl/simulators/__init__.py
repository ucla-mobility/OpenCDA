'''
Copyright 2021 OpenDILab. All Rights Reserved:
Description:
'''
from opencda.core.ml_libs.rl import SIMULATORS

if 'carla' in SIMULATORS:
    from opencda.core.ml_libs.rl.simulators.carla_simulator import CarlaSimulator

