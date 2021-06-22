# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway simplified map sorely with carla
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import argparse
import os

import carla

import opencda.scenario_testing.utils.sim_api as sim_api

from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml


def run_scenario(opt, config_yaml):
    try:
        scenario_params = load_yaml(config_yaml)

        # create simulation world
        simulation_config = scenario_params['world']
        client, world, carla_map, origin_settings = sim_api.createSimulationWorld(simulation_config, town='Town06')

        if opt.record:
            client.start_recorder("single_town06_carla.log", True)

        # create background traffic in carla
        traffic_manager, bg_veh_list = sim_api.createTrafficManager(client, world,
                                                                    scenario_params['carla_traffic_manager'])

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)
        single_cav_list = sim_api.createVehicleManager(world, scenario_params, ['single'], cav_world, carla_map)

        # create evaluation manager
        eval_manager = EvaluationManager(cav_world)

        spectator = world.get_spectator()
        # run steps
        while True:
            world.tick()
            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)

    finally:
        eval_manager.evaluate()
        if opt.record:
            client.stop_recorder()

        world.apply_settings(origin_settings)

        for v in single_cav_list:
            v.destroy()
        for v in bg_veh_list:
            v.destroy()
