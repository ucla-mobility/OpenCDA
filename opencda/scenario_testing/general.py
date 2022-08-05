# -*- coding: utf-8 -*-
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
import os

import carla

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml


def run_scenario(opt, config_yaml):
    try:
        scenario_params = load_yaml(config_yaml)

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)

        # basic_config
        basic_config = scenario_params['basic_config']

        if basic_config['xodr_path'] != 'None':
            current_path = os.path.dirname(os.path.realpath(__file__))
            xodr_path = os.path.join(
                current_path,
                basic_config['xodr_path'])
            map_helper = map_api.spawn_helper_2lanefree
        else:
            xodr_path = None
            map_helper = None

        if basic_config['Town'] != 'None':
            town = basic_config['Town']
        else:
            town = None

        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   xodr_path=xodr_path,
                                                   town=town,
                                                   cav_world=cav_world)

        if opt.record:
            scenario_manager.client. \
                start_recorder(basic_config['log_name'], True)

        spectator = scenario_manager.world.get_spectator()
        spectator_config = basic_config['spectator_config']

        if basic_config['type'] == 'single':
            single_cav_list = \
                scenario_manager.create_vehicle_manager(application=['single'])
            spectator_vehicle = single_cav_list[0].vehicle

        elif basic_config['type'] == 'platooning':
            # create platoon members
            platoon_list = \
                scenario_manager.create_platoon_manager(
                    map_helper=map_helper,
                    data_dump=False)

            # create single cavs
            single_cav_list = \
                scenario_manager.create_vehicle_manager(['platooning'],
                                                        map_helper=map_helper)

            spectator_vehicle = platoon_list[0].vehicle_manager_list[1].vehicle

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name=basic_config['script_name'],
                              current_time=scenario_params['current_time'])

        # run steps
        while True:
            scenario_manager.tick()
            transform = spectator_vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=spectator_config['z']),
                carla.Rotation(
                    pitch=-
                    spectator_config['pitch'])))

            if basic_config['type'] == 'single':
                for i, single_cav in enumerate(single_cav_list):
                    single_cav.update_info()
                    control = single_cav.run_step()
                    single_cav.vehicle.apply_control(control)

            elif basic_config['type'] == 'platooning':

                for platoon in platoon_list:
                    platoon.update_information()
                    platoon.run_step()

                for i, single_cav in enumerate(single_cav_list):
                    # this function should be added in wrapper
                    if single_cav.v2x_manager.in_platoon():
                        single_cav_list.pop(i)
                    else:
                        single_cav.update_info()
                        control = single_cav.run_step()
                        single_cav.vehicle.apply_control(control)

    finally:
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        if basic_config['type'] == 'platooning':
            for platoon in platoon_list:
                platoon.destroy()
        for cav in single_cav_list:
            cav.destroy()
        for v in bg_veh_list:
            v.destroy()


