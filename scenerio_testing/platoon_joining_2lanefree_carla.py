# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway sorely with carla
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import argparse
import os

import carla

import scenerio_testing.utils.sim_api as sim_api
import scenerio_testing.utils.customized_map_api as map_api

from scenerio_testing.utils.yaml_utils import load_yaml


def arg_parse():
    parser = argparse.ArgumentParser(description="Platooning Joining Settings")
    parser.add_argument("--config_yaml", required=True, type=str, help='corresponding yaml file of the testing')

    opt = parser.parse_args()
    return opt


def main():
    try:
        # first define the path of the yaml file and 2lanefreemap file
        opt = arg_parse()
        scenario_params = load_yaml(opt.config_yaml)
        current_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(current_path,
                                 '../assets/2lane_freeway_simplified/map_v7.4_smooth_curve.xodr')

        # create simulation world
        simulation_config = scenario_params['world']
        world, origin_settings = sim_api.createSimulationWorld(simulation_config, xodr_path)

        # create platoon members
        platoon_list, platooning_world = sim_api.createPlatoonManagers(world, scenario_params)
        # create single cavs
        single_cav_list = sim_api.createVehicleManager(world, scenario_params, ['platooning'], platooning_world,
                                                       map_api.spawn_helper_2lanefree)
        # todo spectator wrapper
        spectator = world.get_spectator()
        # run steps
        while True:
            # TODO: Consider aysnc mode later
            world.tick()
            transform = platoon_list[0].vehicle_manager_list[-1].vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=80),
                                                    carla.Rotation(pitch=-90)))
            for platoon in platoon_list:
                platoon.update_information(platooning_world)
                platoon.run_step()

            for i, single_cav in enumerate(single_cav_list):
                # this function should be added in wrapper
                if single_cav.v2x_manager.in_platoon():
                    single_cav_list.pop(i)
                else:
                    single_cav.update_info(platooning_world)
                    control = single_cav.run_step()
                    single_cav.vehicle.apply_control(control)

    finally:
        world.apply_settings(origin_settings)
        for platoon in platoon_list:
            platoon.destroy()
        for cav in single_cav_list:
            cav.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
