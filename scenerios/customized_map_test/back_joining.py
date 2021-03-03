# -*- coding: utf-8 -*-

"""Platooning back joining test
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla
import sys

from core.platooning.platooning_world import PlatooningWorld
from core.platooning.platooning_manager import PlatooningManager
from core.vehicle.vehicle_manager import VehicleManager
from core.platooning.fsm import FSM
from scenerios.customized_map_test.load_customized_world import load_customized_world


def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        xodr_path = '../../customized_map_output/map_v7.3_SUMO_full.xodr'
        world = load_customized_world(xodr_path, client)
        if not world:
            sys.exit()

        origin_settings = world.get_settings()

        # traffic manager setting
        tm = client.get_trafficmanager()
        tm.set_global_distance_to_leading_vehicle(1.0)
        tm.set_synchronous_mode(True)

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # get all spawn points
        all_deafault_spawn = world.get_map().get_spawn_points()
        transform_point = all_deafault_spawn[11]
        # move forward along acceleration lane 0.627 for cut-in-joining
        transform_point.location.x = transform_point.location.x + \
                                     0.625 * (all_deafault_spawn[2].location.x - all_deafault_spawn[11].location.x)
        transform_point.location.y = transform_point.location.y + \
                                     0.625 * (all_deafault_spawn[2].location.y - all_deafault_spawn[11].location.y)

        # setup spawn points
        transform_1 = carla.Transform(carla.Location(x=-450.722836, y=7.500000, z=3.000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_2 = carla.Transform(carla.Location(x=-460.722836, y=7.500000, z=3.000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_3 = carla.Transform(carla.Location(x=-470.722836, y=7.500000, z=3.000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_4 = transform_point

        # background testing traffic car
        transform_5 = carla.Transform(carla.Location(x=-100.722836, y=7.500000, z=3.000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_6 = carla.Transform(carla.Location(x=-130.722836, y=4.5, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_7 = carla.Transform(carla.Location(x=-480, y=7.5, z=3.0),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        # transform_8 = carla.Transform(carla.Location(x=151.7194, y=139.51, z=0.3),
        #                               carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))

        transform_destination_1 = carla.Transform(carla.Location(x=700.372955, y=7.500000, z=3.000000),
                                                  carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))

        transform_destination_2 = all_deafault_spawn[5]

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

        # vehicle 5-7 are background traffic
        ego_vehicle_bp.set_attribute('color', '0, 255, 0')
        vehicle_5 = world.spawn_actor(ego_vehicle_bp, transform_5)
        vehicle_5.set_autopilot(False, 8000)
        vehicle_5.apply_control(carla.VehicleControl(throttle=0.90))

        vehicle_6 = world.spawn_actor(ego_vehicle_bp, transform_6)
        vehicle_6.set_autopilot(True, 8000)

        vehicle_7 = world.spawn_actor(ego_vehicle_bp, transform_7)
        vehicle_7.set_autopilot(False, 8000)
        vehicle_7.apply_control(carla.VehicleControl(throttle=0.83))
        #
        # vehicle_8 = world.spawn_actor(ego_vehicle_bp, transform_8)
        # vehicle_8.set_autopilot(True, 8000)

        tm.global_percentage_speed_difference(-600.0)

        world.tick()
        # create platooning world
        platooning_world = PlatooningWorld()

        # setup managers
        vehicle_manager_1 = VehicleManager(vehicle_1, platooning_world, sample_resolution=6.0, buffer_size=8,
                                           debug_trajectory=False, debug=False, ignore_traffic_light=True)
        vehicle_manager_2 = VehicleManager(vehicle_2, platooning_world, debug_trajectory=False, debug=False)
        vehicle_manager_3 = VehicleManager(vehicle_3, platooning_world, debug_trajectory=False, debug=False)

        vehicle_manager_4 = VehicleManager(vehicle_4, platooning_world, status=FSM.SEARCHING, sample_resolution=4.5,
                                           buffer_size=8, debug_trajectory=True, debug=True, update_freq=15,
                                           overtake_allowed=True)

        platooning_manager = PlatooningManager(platooning_world)

        # set leader
        platooning_manager.set_lead(vehicle_manager_1)
        # add member
        platooning_manager.add_member(vehicle_manager_2)
        platooning_manager.add_member(vehicle_manager_3)

        # set destination
        platooning_manager.set_destination(transform_destination_1.location)
        vehicle_manager_4.agent.set_destination(vehicle_manager_4.vehicle.get_location(),
                                                transform_destination_2.location,
                                                clean=True)
        spectator = world.get_spectator()
        while True:
            # if not world.wait_for_tick(10.0):
            #     continue
            world.tick()
            transform = vehicle_4.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

            platooning_manager.update_information(platooning_world)
            platooning_manager.run_step()
            in_platooning, _, _ = vehicle_manager_4.get_platooning_status()
            if not in_platooning:
                vehicle_manager_4.agent.update_information(platooning_world)
                control = vehicle_manager_4.run_step()
                vehicle_manager_4.vehicle.apply_control(control)

    finally:
        world.apply_settings(origin_settings)
        platooning_manager.destroy()
        vehicle_manager_4.vehicle.destroy()
        vehicle_5.destroy()
        vehicle_6.destroy()
        vehicle_7.destroy()
        vehicle_8.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
