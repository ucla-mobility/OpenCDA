# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway simplified map sorely with carla
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import argparse
from opencda.core.application.platooning.platooning_manager import PlatooningManager
import os

import carla

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api

from opencda.scenario_testing.utils.yaml_utils import load_yaml

# import get speed 
from opencda.core.common.misc import get_speed


def arg_parse():
    parser = argparse.ArgumentParser(description="Platooning Joining Settings")
    parser.add_argument("--config_yaml", required=True, type=str, help='corresponding yaml file of the testing')
    parser.add_argument("--record", action='store_true', help='whether to record playfile')
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.')

    opt = parser.parse_args()
    return opt


def main():
    try:
        # first define the path of the yaml file and 2lanefreemap file
        opt = arg_parse()
        scenario_params = load_yaml(opt.config_yaml)
        current_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(current_path,
                                 '../assets/2lane_freeway_simplified/map_v7.6_12ft_lane.xodr')

        # create simulation world
        simulation_config = scenario_params['world']
        client, world, carla_map, origin_settings = sim_api.createSimulationWorld(simulation_config, xodr_path)

        if opt.record:
            client.start_recorder("platoon_joining_2lanefree_carla.log", True)

        # create background traffic in carla
        # traffic_manager, bg_veh_list = sim_api.createTrafficManager(client, world,
        #                                                             scenario_params['carla_traffic_manager'])

        # create platoon members
        platoon_list, cav_world = sim_api.createPlatoonManagers(world, carla_map, scenario_params,
                                                                apply_ml=opt.apply_ml)
        # create single cavs
        # single_cav_list = sim_api.createVehicleManager(world, scenario_params, ['platooning'], cav_world,
        #                                                carla_map, map_api.spawn_helper_2lanefree)
        # todo spectator wrapper
        spectator = world.get_spectator()
        spectator_vehicle = platoon_list[0].vehicle_manager_list[1].vehicle # also leader vehicle 

        # adjusting leader speed 
        leader_speed_profile = scenario_params['vehicle_base']['behavior']['leader_speeds_profile']
        if len(leader_speed_profile) > 1: 
            variable_platoon_speed = True
            time_counter = 0
            stage = 0
            total_stages = len(leader_speed_profile)
            leader_platoon_manager = platoon_list[0]
            stage_duration = 10
        else: 
            variable_platoon_speed = False

        # run steps
        while True:
            world.tick()
            transform = spectator_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=80),
                                                    carla.Rotation(pitch=-90)))
            for platoon in platoon_list:
                platoon.update_information()
                platoon.run_step()

            # leader speed change 
            if variable_platoon_speed:
                leader_speed = get_speed(spectator_vehicle, True)*3.6
                
                # stage 0
                if stage == 0:
                    print('stage 0, accelrate to %f' % leader_speed_profile[0])
                    if leader_speed >= (leader_speed_profile[0]-3):
                        stage = 1
                    else: 
                        leader_platoon_manager.origin_leader_target_speed = leader_speed_profile[0]
                
                # stage 1
                if stage == 1:
                    print('stage 1, speed has keep for %f second' % (time_counter * 0.05))
                    if time_counter >= stage_duration / 0.05:
                        stage = 2
                        time_counter = 0
                    else:
                        time_counter += 1
                        leader_platoon_manager.origin_leader_target_speed = leader_speed_profile[0]
                
                # stage2: accelerate to next level
                if stage == 2:
                    print('stage 2, accelerate to %f' % leader_speed_profile[1])
                    if leader_speed >= (leader_speed_profile[1]-3):
                        stage = 3
                    else:
                        leader_platoon_manager.origin_leader_target_speed = leader_speed_profile[1]

                # stage3: keep velocity for a fixed duration
                if stage == 3:
                    print('stage 3, speed has keep for %f second' % (time_counter * 0.05))
                    if time_counter >= stage_duration / 0.05:
                        stage = 4
                        time_counter = 0
                    else:
                        time_counter += 1
                        leader_platoon_manager.origin_leader_target_speed = leader_speed_profile[1]

                # stage4: de-accelerate the velocity back to stage1
                if stage == 4:
                    print('stage 4, de-accelerate to %f' % leader_speed_profile[0])
                    if leader_speed <= (leader_speed_profile[0] + 3):
                        stage = 5
                    else:
                        leader_platoon_manager.origin_leader_target_speed = leader_speed_profile[0]

                # stage5: keep the speed for another duration
                if stage == 5:
                    print('stage5, speed has keep for %f second' % (time_counter * 0.05))
                    if time_counter >= stage_duration / 0.05:
                        break
                    else:
                        time_counter += 1
                        leader_platoon_manager.origin_leader_target_speed = leader_speed_profile[0]

            # for i, single_cav in enumerate(single_cav_list):
            #     # this function should be added in wrapper
            #     if single_cav.v2x_manager.in_platoon():
            #         single_cav_list.pop(i)
            #     else:
            #         single_cav.update_info()
            #         control = single_cav.run_step()
            #         single_cav.vehicle.apply_control(control) 


    finally:
        if opt.record:
            client.stop_recorder()

        world.apply_settings(origin_settings)

        for platoon in platoon_list:
            platoon.destroy()




if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
