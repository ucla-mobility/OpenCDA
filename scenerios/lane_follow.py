# -*- coding: utf-8 -*-

"""Simple platooning following test
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla
import os
import random

from core.platooning.platooning_manager import PlatooningManager
from core.vehicle.vehicle_manager import VehicleManager


def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # setup spawn points
        transform_1 = carla.Transform(carla.Location(x=79.40094727, y=-193.74714844, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0.855804, roll=0.000000))
        transform_2 = carla.Transform(carla.Location(x=69.36635742, y=-193.63253906, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0.855804, roll=0.000000))
        transform_3 = carla.Transform(carla.Location(x=49.36635742, y=-193.63253906, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0.855804, roll=0.000000))
        transform_4 = carla.Transform(carla.Location(x=39.36635742, y=-193.63253906, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0.855804, roll=0.000000))
        transform_5 = carla.Transform(carla.Location(x=29.36635742, y=-193.63253906, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0.855804, roll=0.000000))
        transform_destination = carla.Transform(carla.Location(x=58.18258789, y=187.85683594, z=0.3),
                                                carla.Rotation(pitch=0.000000, yaw=-179.712173, roll=0.000000))

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

        # setup managers
        vehicle_manager_1 = VehicleManager(vehicle_1)
        vehicle_manager_2 = VehicleManager(vehicle_2)
        vehicle_manager_3 = VehicleManager(vehicle_3)
        vehicle_manager_4 = VehicleManager(vehicle_4)
        vehicle_manager_5 = VehicleManager(vehicle_5)
        platooning_manager = PlatooningManager()

        # set leader
        platooning_manager.set_lead(vehicle_manager_1)
        # add member
        platooning_manager.add_member(vehicle_manager_2)
        platooning_manager.add_member(vehicle_manager_3)
        platooning_manager.add_member(vehicle_manager_4)
        platooning_manager.add_member(vehicle_manager_5)

        # set destination TODO: the spawn point may have conflict
        destination = transform_destination.location

        platooning_manager.set_destination(destination)

        while True:
            if not world.wait_for_tick(10.0):
                continue
            spectator = world.get_spectator()
            transform = vehicle_3.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

            platooning_manager.update_information(world)
            platooning_manager.run_step()

    finally:
        platooning_manager.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
