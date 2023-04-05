# -*- coding: utf-8 -*-
"""
Scenario testing: single vehicle behavior in intersection
"""
# Author: XH
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla

import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time


def run_scenario(opt, scenario_params):
    try:

        scenario_params = add_current_time(scenario_params)

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)

        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town='Town04',
                                                   cav_world=cav_world)

        if opt.record:
            scenario_manager.client. \
                start_recorder("single_town04_carla_no_traffic.log", True)

        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'])

        # create background traffic in carla
        # traffic_manager, bg_veh_list = \
        #     scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='single_intersection_town04_carla',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        
        # transform = single_cav_list[0].vehicle.get_transform()
        # spectator.set_transform(carla.Transform(
        #     transform.location +
        #     carla.Location(
        #         x=-7,
        #         z=2.5),
        #     carla.Rotation(
        #         pitch=-
        #         8))
        # )
        # for i, single_cav in enumerate(single_cav_list):
        #     single_cav.update_info()
        #     control = single_cav.run_step()
        #     single_cav.vehicle.apply_control(control)
        # # pause until press enter
        # scenario_manager.tick()
        counter = 0
        # run steps
        while True:
            if counter == 30:
                input("Press Enter to continue...")
            scenario_manager.tick()
            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
            transform.location +
            carla.Location(
                x=-7,
                z=2.5),
            carla.Rotation(
                pitch=-
                8)))
            print("counter: ", counter)
            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)
            counter += 1

    finally:
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in single_cav_list:
            v.destroy()
        # for v in bg_veh_list:
        #     v.destroy()

