# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the
customized 2-lane freeway simplified map sorely with co-sim
"""
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
        xodr_path = os.path.join(current_path,
                                 '../assets/2lane_freeway_simplified/'
                                 '2lane_freeway_simplified.xodr')
        sumo_cfg = os.path.join(current_path,
                                '../assets/2lane_freeway_simplified')

        # create co-simulation scenario manager
        scenario_manager = \
            sim_api.CoScenarioManager(scenario_params,
                                      opt.apply_ml,
                                      opt.version,
                                      xodr_path=xodr_path,
                                      cav_world=cav_world,
                                      sumo_file_parent_path=sumo_cfg)

        # create platoon members
        platoon_list = \
            scenario_manager.create_platoon_manager(
                map_helper=map_api.spawn_helper_2lanefree,
                data_dump=False)

        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['platooning'],
                                                    map_helper=map_api.
                                                    spawn_helper_2lanefree)

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='single_2lanefree_cosim',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = platoon_list[0].vehicle_manager_list[1].vehicle

        while True:
            # simulation tick
            scenario_manager.tick()

            transform = spectator_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location +
                                                    carla.Location(z=80),
                                                    carla.Rotation(pitch=-90)))

            for platoon in platoon_list:
                platoon.update_information()
                platoon.run_step()

            for i, single_cav in enumerate(single_cav_list):
                if single_cav.v2x_manager.in_platoon():
                    single_cav_list.pop(i)

                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)

    finally:
        eval_manager.evaluate()
        scenario_manager.close()

        for platoon in platoon_list:
            platoon.destroy()

        for v in single_cav_list:
            v.destroy()
