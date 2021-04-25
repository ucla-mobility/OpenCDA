# -*- coding: utf-8 -*-

"""Test platooning with plotting. The leading vehicle will first accelerate to reach speed1, ann keep speed1 for
x seconds. Then it will have a constant acceleration to reach speed2 and keep the speed for another x seconds.
Finally, it will de-accelerate and get back to speed1 and keep it for another x seconds.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import argparse
import os
import sys

import carla

from core.application.platooning.platooning_world import PlatooningWorld
from core.application.platooning.platooning_manager import PlatooningManager
from core.common.vehicle_manager import VehicleManager
from core.common.misc import get_speed
from scenerio_testing.platooning.customized_map_test.load_customized_world import load_customized_world


def arg_parse():
    parser = argparse.ArgumentParser(description="Evaluation settings")
    parser.add_argument("--max_throttle", type=float, default=0.80, help='Maximum throttle, in range 0-1')
    parser.add_argument("--max_break", type=float, default=1.0, help='Maximum break, in range 0-1')
    parser.add_argument("--stage_duration", type=float, default=20.0, help='duration for each stage')
    parser.add_argument("--speed1", type=float, default=20.0, help='stage1 desired speed')
    parser.add_argument("--speed2", type=float, default=25.0, help='stage2 desired speed')

    opt = parser.parse_args()

    return opt


def main():
    try:
        opt = arg_parse()

        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        dir_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(dir_path, '../../../assets/2lane_freeway_simplified/map_v7.4_smooth_curve.xodr')

        world = load_customized_world(xodr_path, client)
        if not world:
            sys.exit()

        origin_settings = world.get_settings()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # traffic manager for background traffic

        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        traffic_manager.global_percentage_speed_difference(-80)
        traffic_manager.set_synchronous_mode(True)

        # setup spawn points
        transform_1 = carla.Transform(carla.Location(x=-1000.722836, y=7.500000, z=0.3000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_2 = carla.Transform(carla.Location(x=-1010.722836, y=7.500000, z=0.300000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_3 = carla.Transform(carla.Location(x=-1020.722836, y=7.500000, z=0.300000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_4 = carla.Transform(carla.Location(x=-1030.722836, y=7.500000, z=0.300000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_5 = carla.Transform(carla.Location(x=-1040.722836, y=7.500000, z=0.300000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))

        transform_destination = carla.Transform(carla.Location(x=1100.372955, y=7.500000, z=3.000000),
                                                carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))

        # create the leading vehicle
        ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
        # black color
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        vehicle_1 = world.spawn_actor(ego_vehicle_bp, transform_1)

        ego_vehicle_bp.set_attribute('color', '255, 255, 255')
        vehicle_2 = world.spawn_actor(ego_vehicle_bp, transform_2)

        ego_vehicle_bp.set_attribute('color', '255, 255, 255')
        vehicle_3 = world.spawn_actor(ego_vehicle_bp, transform_3)

        ego_vehicle_bp.set_attribute('color', '255, 255, 255')
        vehicle_4 = world.spawn_actor(ego_vehicle_bp, transform_4)

        ego_vehicle_bp.set_attribute('color', '255, 255, 255')
        vehicle_5 = world.spawn_actor(ego_vehicle_bp, transform_5)

        world.tick()
        # create platooning world
        platooning_world = PlatooningWorld()

        # setup managers
        vehicle_manager_1 = VehicleManager(vehicle_1, platooning_world, sample_resolution=4.5, buffer_size=8,
                                           debug_trajectory=True, debug=False, ignore_traffic_light=True)
        vehicle_manager_2 = VehicleManager(vehicle_2, platooning_world, buffer_size=8,
                                           debug_trajectory=True, debug=False)
        vehicle_manager_3 = VehicleManager(vehicle_3, platooning_world,
                                           debug_trajectory=False, debug=False)
        vehicle_manager_4 = VehicleManager(vehicle_4, platooning_world,
                                           debug_trajectory=False, debug=False)
        vehicle_manager_5 = VehicleManager(vehicle_5, platooning_world,
                                           debug_trajectory=False, debug=False)

        platooning_manager = PlatooningManager(platooning_world)

        # set leader
        platooning_manager.set_lead(vehicle_manager_1)
        # add member
        platooning_manager.add_member(vehicle_manager_2)
        platooning_manager.add_member(vehicle_manager_3)
        platooning_manager.add_member(vehicle_manager_4)
        platooning_manager.add_member(vehicle_manager_5)

        # set destination
        destination = transform_destination.location

        platooning_manager.set_destination(destination)

        # used to count time
        time_counter = 0
        stage = 0
        # set the desired acceleration and de-acceleration
        platooning_manager.set_controller_longitudinal(opt.max_throttle, opt.max_break)

        while True:
            world.tick()
            spectator = world.get_spectator()
            transform = vehicle_3.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

            leading_current_speed = get_speed(vehicle_1, True)
            print('current speed is %f' % leading_current_speed)

            # stage0: normal acceleration stage
            if stage == 0:
                print('stage 0, accelrate to %f' % opt.speed1)
                # give a small buffer of 0.2 m/s
                if leading_current_speed >= (opt.speed1 - 1):
                    stage = 1
                else:
                    platooning_manager.origin_leader_target_speed = opt.speed1 * 3.6

            # stage1: keep velocity for a fixed duration
            if stage == 1:
                print('stage 1, speed has keep for %f second' % (time_counter * 0.05))
                if time_counter >= opt.stage_duration / 0.05:
                    stage = 2
                    time_counter = 0
                else:
                    time_counter += 1
                    platooning_manager.origin_leader_target_speed = opt.speed1 * 3.6

            # stage2: accelerate to next level
            if stage == 2:
                print('stage 2, accelerate to %f' % opt.speed2)
                if leading_current_speed >= (opt.speed2 - 1):
                    stage = 3
                else:
                    platooning_manager.origin_leader_target_speed = opt.speed2 * 3.6

            # stage3: keep velocity for a fixed duration
            if stage == 3:
                print('stage 3, speed has keep for %f second' % (time_counter * 0.05))
                if time_counter >= opt.stage_duration / 0.05:
                    stage = 4
                    time_counter = 0
                else:
                    time_counter += 1
                    platooning_manager.origin_leader_target_speed = opt.speed2 * 3.6

            # stage4: de-accelerate the velocity back to stage1
            if stage == 4:
                print('stage 4, de-accelerate to %f' % opt.speed1)
                if leading_current_speed <= (opt.speed1 + 1):
                    stage = 5
                else:
                    platooning_manager.origin_leader_target_speed = opt.speed1 * 3.6

            # stage5: keep the speed for another duration
            if stage == 5:
                print('stage5, speed has keep for %f second' % (time_counter * 0.05))
                if time_counter >= opt.stage_duration / 0.05:
                    break
                else:
                    time_counter += 1
                    platooning_manager.origin_leader_target_speed = opt.speed1 * 3.6

            platooning_manager.update_information(platooning_world)
            platooning_manager.run_step()

    finally:
        platooning_manager.destroy()
        origin_settings.synchronous_mode = False
        world.apply_settings(origin_settings)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
