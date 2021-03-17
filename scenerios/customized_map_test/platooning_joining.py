# -*- coding: utf-8 -*-

"""Platooning back joining test
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import argparse
import os
import sys

import carla

from core.platooning.platooning_world import PlatooningWorld
from core.platooning.platooning_manager import PlatooningManager
from core.vehicle.vehicle_manager import VehicleManager
from core.platooning.fsm import FSM
from scenerios.customized_map_test.load_customized_world import load_customized_world


def arg_parse():
    parser = argparse.ArgumentParser(description="Platooning Joining Settings")
    parser.add_argument("--joining_method",
                        default='frontal_joining', type=str, help='cut_in_joining, back_joining, or frontal_joining')
    parser.add_argument("--bg_traffic", action='store_true', help='whether to create pre-defined background traffic')

    opt = parser.parse_args()

    return opt


def main():
    try:
        opt = arg_parse()

        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        dir_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(dir_path, '../../customized_map_output/map_v7.4_smooth_curve.xodr')

        # load the world with our customized map
        world = load_customized_world(xodr_path, client)
        if not world:
            sys.exit()

        # used to recover the world back to async mode when the testing is done
        origin_settings = world.get_settings()

        # traffic manager setting
        tm = client.get_trafficmanager()
        tm.set_global_distance_to_leading_vehicle(3.0)
        tm.set_synchronous_mode(True)

        # set simulation to sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # get all spawn points
        all_deafault_spawn = world.get_map().get_spawn_points()
        transform_point = all_deafault_spawn[11]

        # the merging vehicle initial position is decided by the joining method
        if opt.joining_method == 'back_joining':
            start_pos = 0.43
        elif opt.joining_method == 'frontal_joining':
            start_pos = 0.51
        elif opt.joining_method == 'cut_in_joining':
            start_pos = 0.50
        else:
            print('only back_joining, frontal_joining, cut_in_joining are supported')
            sys.exit()

        # set up the merging vehicle initial position based on joining method
        transform_point.location.x = transform_point.location.x + \
                                     start_pos * (all_deafault_spawn[2].location.x - all_deafault_spawn[11].location.x)
        transform_point.location.y = transform_point.location.y + \
                                     start_pos * (all_deafault_spawn[2].location.y - all_deafault_spawn[11].location.y)

        # setup the platooning generated position
        transform_1 = carla.Transform(carla.Location(x=-650.722836, y=7.500000, z=0.30000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_2 = carla.Transform(carla.Location(x=-660.722836, y=7.500000, z=0.300000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_3 = carla.Transform(carla.Location(x=-670.722836, y=7.500000, z=0.30000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_4 = carla.Transform(carla.Location(x=-680.722836, y=7.500000, z=0.300000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_5 = transform_point

        ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')

        # platooning destination
        transform_destination_1 = carla.Transform(carla.Location(x=1000.372955, y=7.500000, z=3.000000),
                                                  carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        # merging vehicle initial destination
        transform_destination_2 = all_deafault_spawn[5]

        # create platooning and merging vehicle. Leading vehicle is white, followings are black
        # and merging vehicle is blue
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        vehicle_1 = world.spawn_actor(ego_vehicle_bp, transform_1)

        ego_vehicle_bp.set_attribute('color', '255, 255, 255')
        vehicle_2 = world.spawn_actor(ego_vehicle_bp, transform_2)

        ego_vehicle_bp.set_attribute('color', '255, 255, 255')
        vehicle_3 = world.spawn_actor(ego_vehicle_bp, transform_3)

        ego_vehicle_bp.set_attribute('color', '255, 255, 255')
        vehicle_4 = world.spawn_actor(ego_vehicle_bp, transform_4)

        ego_vehicle_bp.set_attribute('color', '0, 0, 255')
        vehicle_5 = world.spawn_actor(ego_vehicle_bp, transform_5)

        bg_vehicle_list = []

        # we set a human-driver vehicle in front of the platooning in the same lane, a hv behind the platooning
        # in the same lane to test the robustness of front joining and back joining
        if opt.bg_traffic:
            transform_6 = carla.Transform(carla.Location(x=-540.722836, y=7.500000, z=0.3),
                                          carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
            transform_7 = carla.Transform(carla.Location(x=-715.722836, y=7.5, z=0.3),
                                          carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
            transform_8 = carla.Transform(carla.Location(x=-670, y=4.5, z=0.3),
                                          carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))

            # background traffic is green color
            ego_vehicle_bp.set_attribute('color', '0, 255, 0')
            vehicle_6 = world.spawn_actor(ego_vehicle_bp, transform_6)
            vehicle_6.apply_control(carla.VehicleControl(throttle=0.88))
            vehicle_6.set_autopilot(False, 8000)

            vehicle_7 = world.spawn_actor(ego_vehicle_bp, transform_7)
            vehicle_7.apply_control(carla.VehicleControl(throttle=0.84))
            vehicle_7.set_autopilot(False, 8000)

            vehicle_8 = world.spawn_actor(ego_vehicle_bp, transform_8)
            vehicle_8.apply_control(carla.VehicleControl(throttle=0.88))
            vehicle_8.set_autopilot(False, 8000)

            bg_vehicle_list = [vehicle_6, vehicle_7, vehicle_8]

        # This is important. Tick the world to update vehicle start position information
        world.tick()
        # create platooning world
        platooning_world = PlatooningWorld()

        # setup managers
        vehicle_manager_1 = VehicleManager(vehicle_1, platooning_world, sample_resolution=4.5, buffer_size=12,
                                           debug_trajectory=True, debug=False, ignore_traffic_light=True)
        vehicle_manager_2 = VehicleManager(vehicle_2, platooning_world, debug_trajectory=False, debug=False)
        vehicle_manager_3 = VehicleManager(vehicle_3, platooning_world, debug_trajectory=False, debug=False)
        vehicle_manager_4 = VehicleManager(vehicle_4, platooning_world, debug_trajectory=False, debug=False)

        vehicle_manager_5 = VehicleManager(vehicle_5, platooning_world, status=FSM.SEARCHING, sample_resolution=4.5,
                                           buffer_size=12, debug_trajectory=True, debug=True, update_freq=15,
                                           overtake_allowed=True)

        platooning_manager = PlatooningManager(platooning_world)

        # set leader
        platooning_manager.set_lead(vehicle_manager_1)
        # add member
        platooning_manager.add_member(vehicle_manager_2)
        platooning_manager.add_member(vehicle_manager_3)
        platooning_manager.add_member(vehicle_manager_4)

        # set destination
        platooning_manager.set_destination(transform_destination_1.location)
        vehicle_manager_5.agent.set_destination(vehicle_manager_5.vehicle.get_location(),
                                                transform_destination_2.location,
                                                clean=True)
        spectator = world.get_spectator()
        while True:
            world.tick()
            # use spectator to keep following merging vehicle position
            transform = vehicle_3.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=80),
                                                    carla.Rotation(pitch=-90)))

            # update world info to each vehicle
            platooning_manager.update_information(platooning_world)
            control_list = platooning_manager.run_step()

            # assign the control command from platooning to the human-drive vehicle behind to make sure it is
            # close enough to the platooning so we can test back joining algorithm's robustness
            if opt.bg_traffic:
                vehicle_7.apply_control(control_list[-2])

            in_platooning, _, _ = vehicle_manager_5.get_platooning_status()
            if not in_platooning:
                vehicle_manager_5.agent.update_information(platooning_world)
                control = vehicle_manager_5.run_step()
                vehicle_manager_5.vehicle.apply_control(control)

    finally:
        world.apply_settings(origin_settings)
        platooning_manager.destroy()

        for v in bg_vehicle_list:
            v.destroy()

        if not in_platooning:
            vehicle_manager_5.vehicle.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
