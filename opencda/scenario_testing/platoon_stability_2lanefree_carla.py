# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway simplified map sorely with carla
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import sys
import os

import carla

import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.misc import get_speed
from opencda.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import load_yaml


def run_scenario(opt, config_yaml):
    try:
        scenario_params = load_yaml(config_yaml)
        current_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(current_path,
                                 '../assets/2lane_freeway_simplified/map_v7.6_12ft_lane.xodr')

        # create simulation world
        simulation_config = scenario_params['world']
        client, world, carla_map, origin_settings = sim_api.createSimulationWorld(simulation_config, xodr_path)

        if opt.record:
            client.start_recorder("platoon_joining_2lanefree_carla.log", True)

        # create background traffic in carla
        traffic_manager, bg_veh_list = sim_api.createTrafficManager(client, world,
                                                                    scenario_params['carla_traffic_manager'])

        # create platoon members
        platoon_list, cav_world = sim_api.createPlatoonManagers(world, carla_map, scenario_params,
                                                                apply_ml=opt.apply_ml)

        if len(platoon_list) > 1:
            sys.exit("In this scenario testing, only single platoon is allowed.")

        spectator = world.get_spectator()
        spectator_vehicle = platoon_list[0].vehicle_manager_list[0].vehicle

        eval_manager = EvaluationManager(cav_world)

        # adjusting leader speed 
        leader_speed_profile = scenario_params['platoon_base']['leader_speeds_profile']
        stage_duration = scenario_params['platoon_base']['stage_duration']

        test_platoon_manager = platoon_list[0]

        if len(leader_speed_profile) > 1: 
            platoon_speed_adjusting = True
            time_counter = 0
            stage = 0
        else: 
            platoon_speed_adjusting = False

        # run steps
        while True:
            world.tick()
            transform = spectator_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=80),
                                                    carla.Rotation(pitch=-90)))

            test_platoon_manager.update_information()
            test_platoon_manager.run_step()

            # leader speed change to test the stability of the platoon
            if platoon_speed_adjusting:
                leader_speed = get_speed(spectator_vehicle, True)*3.6
                
                # stage 0
                if stage == 0:
                    print('stage 0, accelrate to %f' % leader_speed_profile[0])
                    if leader_speed >= (leader_speed_profile[0]-3):
                        stage = 1
                    else: 
                        test_platoon_manager.origin_leader_target_speed = leader_speed_profile[0]
                
                # stage 1
                if stage == 1:
                    print('stage 1, speed has keep for %f second' % (time_counter * 0.05))
                    if time_counter >= stage_duration / 0.05:
                        stage = 2
                        time_counter = 0
                    else:
                        time_counter += 1
                        test_platoon_manager.origin_leader_target_speed = leader_speed_profile[0]
                
                # stage2: accelerate to next level
                if stage == 2:
                    print('stage 2, accelerate to %f' % leader_speed_profile[1])
                    if leader_speed >= (leader_speed_profile[1]-3):
                        stage = 3
                    else:
                        test_platoon_manager.origin_leader_target_speed = leader_speed_profile[1]

                # stage3: keep velocity for a fixed duration
                if stage == 3:
                    print('stage 3, speed has keep for %f second' % (time_counter * 0.05))
                    if time_counter >= stage_duration / 0.05:
                        stage = 4
                        time_counter = 0
                    else:
                        time_counter += 1
                        test_platoon_manager.origin_leader_target_speed = leader_speed_profile[1]

                # stage4: de-accelerate the velocity back to stage1
                if stage == 4:
                    print('stage 4, de-accelerate to %f' % leader_speed_profile[0])
                    if leader_speed <= (leader_speed_profile[0] + 3):
                        stage = 5
                    else:
                        test_platoon_manager.origin_leader_target_speed = leader_speed_profile[0]

                # stage5: keep the speed for another duration
                if stage == 5:
                    print('stage5, speed has keep for %f second' % (time_counter * 0.05))
                    if time_counter >= stage_duration / 0.05:
                        break
                    else:
                        time_counter += 1
                        test_platoon_manager.origin_leader_target_speed = leader_speed_profile[0]

    finally:
        eval_manager.evaluate()

        if opt.record:
            client.stop_recorder()

        world.apply_settings(origin_settings)

        for platoon in platoon_list:
            platoon.destroy()

        for v in bg_veh_list:
            v.destroy()
