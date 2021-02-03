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
        blueprint_library = world.get_blueprint_library()

        # setup spawn points
        transform_1 = carla.Transform(carla.Location(x=89.40094727, y=-193.74714844, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0.855804, roll=0.000000))
        transform_2 = carla.Transform(carla.Location(x=79.36635742, y=-193.63253906, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0.855804, roll=0.000000))
        transform_3 = carla.Transform(carla.Location(x=59.36635742, y=-190.2471679, z=0.3),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=-0.242500))
        transform_destination_1 = carla.Transform(carla.Location(x=7.116189, y=-193.484, z=0.3),
                                                carla.Rotation(pitch=0.000000, yaw=-0.767, roll=-0.2425))
        transform_destination_2 = carla.Transform(carla.Location(x=193.54, y=13.6585, z=0.3),
                                                carla.Rotation(pitch=0.000000, yaw=0, roll=90))

        # create the leading vehicle
        ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
        # black color
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        vehicle_1 = world.spawn_actor(ego_vehicle_bp, transform_1)

        ego_vehicle_bp.set_attribute('color', '255, 255, 255')
        vehicle_2 = world.spawn_actor(ego_vehicle_bp, transform_2)

        ego_vehicle_bp.set_attribute('color', '255, 255, 255')
        vehicle_3 = world.spawn_actor(ego_vehicle_bp, transform_3)

        # create platooning world
        platooning_world = PlatooningWorld()

        # setup managers
        vehicle_manager_1 = VehicleManager(vehicle_1, platooning_world, sample_resolution=6.0, buffer_size=8,
                                           debug_trajectory=False, debug=False, ignore_traffic_light=True)
        vehicle_manager_2 = VehicleManager(vehicle_2, platooning_world)
        vehicle_manager_3 = VehicleManager(vehicle_3, platooning_world, status=FSM.SEARCHING, sample_resolution=6.0,
                                           buffer_size=8, debug_trajectory=True, debug=True)

        platooning_manager = PlatooningManager(platooning_world)

        # set leader
        platooning_manager.set_lead(vehicle_manager_1)
        # add member
        platooning_manager.add_member(vehicle_manager_2)

        # set destination TODO: the spawn point may have conflict
        platooning_manager.set_destination(transform_destination_1.location)
        vehicle_manager_3.agent.set_destination(vehicle_manager_3.vehicle.get_location(),
                                                transform_destination_2.location,
                                                clean=True)

        while True:
            if not world.wait_for_tick(10.0):
                continue
            spectator = world.get_spectator()
            transform = vehicle_3.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

            platooning_manager.update_information(world)

            in_platooning, _, _ = vehicle_manager_3.get_platooning_status()
            if not in_platooning:
                vehicle_manager_3.agent.update_information(world)

            platooning_manager.run_step()

            if not in_platooning:
                control = vehicle_manager_3.run_step()
                vehicle_manager_3. vehicle.apply_control(control)

    finally:
        platooning_manager.destroy()
        vehicle_manager_3.vehicle.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')