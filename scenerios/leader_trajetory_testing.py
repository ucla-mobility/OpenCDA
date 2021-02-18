# -*- coding: utf-8 -*-

"""Simple platooning following test
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla

from core.agents.tools.misc import get_speed
from core.platooning.platooning_world import PlatooningWorld
from core.platooning.platooning_manager import PlatooningManager
from core.vehicle.vehicle_manager import VehicleManager


def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()
        origin_settings = world.get_settings()

        # traffic manager for background traffic
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(False)

        blueprint_library = world.get_blueprint_library()

        # setup spawn points
        transform_1 = carla.Transform(carla.Location(x=51.71, y=139.53, z=1.0),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_2 = carla.Transform(carla.Location(x=201.7194, y=139.51, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_3 = carla.Transform(carla.Location(x=221.7194, y=143.51, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))

        transform_destination = carla.Transform(carla.Location(x=630, y=142, z=0.3),
                                                carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        # create the leading vehicle
        ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
        # black color
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        vehicle_1 = world.spawn_actor(ego_vehicle_bp, transform_1)

        # simple background traffic
        ego_vehicle_bp.set_attribute('color', '0, 255, 0')
        vehicle_2 = world.spawn_actor(ego_vehicle_bp, transform_2)
        vehicle_2.apply_control(carla.VehicleControl(throttle=0.55))
        vehicle_2.set_autopilot(False)

        vehicle_3 = world.spawn_actor(ego_vehicle_bp, transform_3)
        vehicle_3.apply_control(carla.VehicleControl(throttle=0.5))
        vehicle_3.set_autopilot(False)

        # create platooning world
        platooning_world = PlatooningWorld()

        # setup managers
        vehicle_manager_1 = VehicleManager(vehicle_1, platooning_world, sample_resolution=4.5, buffer_size=8,
                                           debug_trajectory=True, debug=True, ignore_traffic_light=True,
                                           overtake_allowed=True)
        platooning_manager = PlatooningManager(platooning_world)

        # set leader
        platooning_manager.set_lead(vehicle_manager_1)

        # set destination TODO: the spawn point may have conflict
        destination = transform_destination.location

        platooning_manager.set_destination(destination)

        while True:
            if not world.wait_for_tick(10.0):
                continue
            spectator = world.get_spectator()
            transform = vehicle_1.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))
            # print(get_speed(vehicle_1))
            platooning_manager.update_information(platooning_world)
            platooning_manager.run_step()

    finally:
        platooning_manager.destroy()
        vehicle_2.destroy()
        vehicle_3.destroy()
        world.apply_settings(origin_settings)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
