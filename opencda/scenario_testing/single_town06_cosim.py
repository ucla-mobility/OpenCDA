# -*- coding: utf-8 -*-
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os

import carla

import opencda.scenario_testing.utils.cosim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time


def run_scenario(opt, scenario_params):
    try:
        scenario_params = add_current_time(scenario_params)

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)

        # sumo conifg file path
        current_path = os.path.dirname(os.path.realpath(__file__))
        sumo_cfg = os.path.join(current_path,
                                '../assets/Town06')

        # create co-simulation scenario manager
        scenario_manager = \
            sim_api.CoScenarioManager(scenario_params,
                                      opt.apply_ml,
                                      opt.version,
                                      town='Town06',
                                      cav_world=cav_world,
                                      sumo_file_parent_path=sumo_cfg)
        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'],
                                                    map_helper=map_api.
                                                    spawn_helper_2lanefree)

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='single_2lanefree_cosim',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()

        while True:
            # simulation tick
            scenario_manager.tick()

            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location +
                                                    carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)

    finally:
        eval_manager.evaluate()
        scenario_manager.close()
        for v in single_cav_list:
            v.destroy()
