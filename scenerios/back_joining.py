# -*- coding: utf-8 -*-

"""Platooning back joining test
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla

from core.platooning.platooning_world import PlatooningWorld
from core.platooning.platooning_manager import PlatooningManager
from core.vehicle.vehicle_manager import VehicleManager
from core.platooning.fsm import FSM


def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()
        origin_settings = world.get_settings()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # setup spawn points
        transform_1 = carla.Transform(carla.Location(x=111.7194, y=139.51, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_2 = carla.Transform(carla.Location(x=101.7194, y=139.51, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_3 = carla.Transform(carla.Location(x=91.7194, y=139.51, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_4 = carla.Transform(carla.Location(x=27.7194, y=143.51, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        # background testing traffic car
        transform_5 = carla.Transform(carla.Location(x=81.7194, y=139.51, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_6 = carla.Transform(carla.Location(x=57.7194, y=143.51, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))

        transform_destination_1 = carla.Transform(carla.Location(x=630, y=141.39, z=0.3),
                                                  carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_destination_2 = carla.Transform(carla.Location(x=606.87, y=145.39, z=0.3),
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

        # vehicle 5-7 are background traffic
        ego_vehicle_bp.set_attribute('color', '0, 255, 0')
        # vehicle_5 = world.spawn_actor(ego_vehicle_bp, transform_5)
        # vehicle_5.apply_control(carla.VehicleControl(throttle=0.65))
        # vehicle_5.set_autopilot(False)
        #
        # vehicle_6 = world.spawn_actor(ego_vehicle_bp, transform_6)
        # vehicle_6.apply_control(carla.VehicleControl(throttle=0.55))
        # vehicle_6.set_autopilot(False)

        world.tick()
        # create platooning world
        platooning_world = PlatooningWorld()

        # setup managers
        vehicle_manager_1 = VehicleManager(vehicle_1, platooning_world, sample_resolution=6.5, buffer_size=8,
                                           debug_trajectory=False, debug=False, ignore_traffic_light=True)
        vehicle_manager_2 = VehicleManager(vehicle_2, platooning_world, debug_trajectory=False, debug=False)
        vehicle_manager_3 = VehicleManager(vehicle_3, platooning_world, debug_trajectory=False, debug=False)

        vehicle_manager_4 = VehicleManager(vehicle_4, platooning_world, status=FSM.SEARCHING, sample_resolution=6.5,
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
                vehicle_manager_4. vehicle.apply_control(control)

    finally:
        world.apply_settings(origin_settings)
        platooning_manager.destroy()
        vehicle_manager_4.vehicle.destroy()
        # vehicle_5.destroy()
        # vehicle_6.destroy()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')