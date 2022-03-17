'''
Copyright 2021 OpenDILab. All Rights Reserved:
Description:
'''
from opencda.core.ml_libs.reinforcement_learning import SIMULATORS

if 'carla' in SIMULATORS:
    from .carla_simulator import CarlaSimulator

