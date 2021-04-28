# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway sorely with carla
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import argparse
import os
import sys

import carla

from scenerio_testing.utils.yaml_utils import load_yaml
from scenerio_testing.utils.load_customized_world import load_customized_world


def arg_parse():
    parser = argparse.ArgumentParser(description="Platooning Joining Settings")
    parser.add_argument("--yaml", required=True, type=str, help='corresponding yaml file of the testing')

    opt = parser.parse_args()
    return opt


def main():
    try:
        opt = argparse()
        scenario_params = load_yaml(opt.yaml)
        # set simulation
        simulation_config = scenario_params['world']

        client = carla.Client('localhost', simulation_config['client_port'])
        client.set_timeout(2.0)

        # Retrieve the customized map
        current_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(current_path,
                                 '../../../assets/2lane_freeway_simplified/map_v7.4_smooth_curve.xodr')
        world = load_customized_world(xodr_path)
        if not world:
            sys.exit()

        # used to recover the world back to async mode when the testing is done
        origin_settings = world.get_settings()
        new_settings = world.get_settings()

        if simulation_config['sync_mode']:
            new_settings.synchronous_mode = True
            new_settings.fixed_delta_seconds = simulation_config['simulation_config']
        world.apply_settings(new_settings)

        cav_vehicle_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz2017')


    finally:
        pass
