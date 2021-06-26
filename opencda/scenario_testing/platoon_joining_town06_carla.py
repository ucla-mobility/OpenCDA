# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway sorely with carla
Warning: You have to load the 2lanefreecomplete map into your ue4 editor before running this
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT


import carla

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
from opencda.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml


def run_scenario(opt, config_yaml):
    try:
        scenario_params = load_yaml(config_yaml)

        # create simulation world
        simulation_config = scenario_params['world']
        client, world, carla_map, origin_settings = sim_api.createSimulationWorld(simulation_config, town='Town06')

        if opt.record:
            client.start_recorder("platoon_joining_town06_carla.log", True)
        # create background traffic in carla
        traffic_manager, bg_veh_list = sim_api.createTrafficManager(client, world,
                                                                    scenario_params['carla_traffic_manager'])

        # create platoon members
        platoon_list, cav_world = sim_api.createPlatoonManagers(world, carla_map, scenario_params,
                                                                apply_ml=opt.apply_ml)
        # create single cavs
        single_cav_list = sim_api.createVehicleManager(world, scenario_params, ['platooning'], cav_world,
                                                       carla_map)

        # create evaluation manager
        eval_manager = EvaluationManager(cav_world)

        spectator = world.get_spectator()
        # fix the spectator on a certain car
        spectator_vehicle = single_cav_list[0].vehicle

        while True:
            world.tick()
            transform = spectator_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=60),
                                                    carla.Rotation(pitch=-90)))
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
            client.stop_recorder()

        world.apply_settings(origin_settings)

        for v in bg_veh_list:
            v.destroy()
        for cav in single_cav_list:
            cav.destroy()
        for platoon in platoon_list:
            platoon.destroy()

