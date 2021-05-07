# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway sorely with carla
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import argparse
import os
import sys

import carla

from core.common.vehicle_manager import VehicleManager
from core.application.platooning.platooning_world import PlatooningWorld
from scenerio_testing.utils.yaml_utils import load_yaml
from scenerio_testing.utils.load_customized_world import load_customized_world


def arg_parse():
    parser = argparse.ArgumentParser(description="Platooning Joining Settings")
    parser.add_argument("--config_yaml", required=True, type=str, help='corresponding yaml file of the testing')

    opt = parser.parse_args()
    return opt


def main():
    try:
        opt = arg_parse()
        scenario_params = load_yaml(opt.config_yaml)
        # set simulation
        simulation_config = scenario_params['world']

        client = carla.Client('localhost', simulation_config['client_port'])
        client.set_timeout(2.0)

        # Retrieve the customized map
        current_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(current_path,
                                 '../assets/2lane_freeway_simplified/map_v7.4_smooth_curve.xodr')
        world = load_customized_world(xodr_path, client)
        if not world:
            sys.exit()

        # used to recover the world back to async mode when the testing is done
        origin_settings = world.get_settings()
        new_settings = world.get_settings()

        if simulation_config['sync_mode']:
            new_settings.synchronous_mode = True
            new_settings.fixed_delta_seconds = simulation_config['fixed_delta_seconds']
        world.apply_settings(new_settings)

        cav_vehicle_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz2017')

        vehicle_manager_list = []
        # TODO: Remove this later
        platooning_world = PlatooningWorld()

        # create corresponding vehicle managers
        for i, cav in enumerate(scenario_params['scenario']['single_cav_list']):
            spawn_transform = carla.Transform(carla.Location(x=cav['spawn_position'][0],
                                                             y=cav['spawn_position'][1],
                                                             z=cav['spawn_position'][2]),
                                              carla.Rotation(pitch=0, yaw=0, roll=0))
            destination = carla.Transform(carla.Location(x=cav['destination'][0],
                                                         y=cav['destination'][1],
                                                         z=cav['destination'][2]),
                                          carla.Rotation(pitch=0, yaw=0, roll=0))

            cav_vehicle_bp.set_attribute('color', '0, 0, 0')
            vehicle = world.spawn_actor(cav_vehicle_bp, spawn_transform)
            # create the vehicle manager
            vehicle_manager = VehicleManager(vehicle, cav, ['single'], platooning_world)

            # this is important to update the vehicle position into server
            world.tick()
            vehicle_manager.set_destination(vehicle_manager.vehicle.get_location(),
                                            destination.location,
                                            clean=True)
            vehicle_manager_list.append(vehicle_manager)

        spectator = world.get_spectator()
        # run steps
        while True:
            # TODO: Consider aysnc mode later
            world.tick()
            transform = vehicle_manager_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=40),
                                                    carla.Rotation(pitch=-90)))
            for vm in vehicle_manager_list:
                vm.update_info(platooning_world)
                control = vm.run_step()
                # TODO: Embede apply control inside vm!
                vm.vehicle.apply_control(control)

    finally:
        world.apply_settings(origin_settings)
        for v in vehicle_manager_list:
            v.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
