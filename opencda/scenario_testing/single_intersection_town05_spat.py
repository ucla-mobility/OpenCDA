# -*- coding: utf-8 -*-
"""
Scenario testing: single vehicle behavior in intersection with SPaT control
"""
# Author: Xu Han <hanxu417@ucla>
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla

import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time
from opencda.core.common.traffic_signal_manager import \
    TrafficSignalManager


def run_scenario(opt, scenario_params):
    try:
        scenario_params = add_current_time(scenario_params)

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)

        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town='Town05',
                                                   cav_world=cav_world)

        # create traffic signal manager
        traffic_signal_config = scenario_params['scenario']['traffic_signal']

        # traffic signal manager 
        traffic_signal_manager = TrafficSignalManager(
            world=scenario_manager.world,
            northbound_id=traffic_signal_config['northbound_id'],
            eastbound_id=traffic_signal_config['eastbound_id'],
            southbound_id=traffic_signal_config['southbound_id'],
            westbound_id=traffic_signal_config['westbound_id'],
            adaptive_cycle=traffic_signal_config['adaptive_cycle'])

        # record option 
        if opt.record:
            scenario_manager.client. \
                start_recorder("single_town06_carla.log", True)

        # single CAV         
        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'])

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='single_intersection_town06_carla',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        # run steps
        while True:
            scenario_manager.tick()

            # Traffic signal control
            traffic_signal_manager.update_info()
            traffic_signal_manager.run_step()

            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=50),
                carla.Rotation(
                    pitch=-
                    90)))

            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)

    finally:
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in single_cav_list:
            v.destroy()
        for v in bg_veh_list:
            v.destroy()

