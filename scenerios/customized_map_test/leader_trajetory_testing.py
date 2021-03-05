# -*- coding: utf-8 -*-

"""Simple platooning following test
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT
import sys
import carla

from core.platooning.platooning_world import PlatooningWorld
from core.platooning.platooning_manager import PlatooningManager
from core.vehicle.vehicle_manager import VehicleManager
from scenerios.customized_map_test.load_customized_world import load_customized_world


def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        xodr_path = '../../customized_map_output/map_v7.4_smooth_curve.xodr'
        world = load_customized_world(xodr_path, client)
        if not world:
            sys.exit()

        origin_settings = world.get_settings()

        # set sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # read all default waypoints
        all_deafault_spawn = world.get_map().get_spawn_points()
        # setup spawn points
        transform_point = all_deafault_spawn[11]
        # move forward along acceleration lane
        transform_point.location.x = transform_point.location.x + \
                                     0.45 * (all_deafault_spawn[2].location.x - all_deafault_spawn[11].location.x)
        transform_point.location.y = transform_point.location.y + \
                                     0.45 * (all_deafault_spawn[2].location.y - all_deafault_spawn[11].location.y)
        # destination
        # transform_destination = all_deafault_spawn[4]  # left lane
        transform_destination = all_deafault_spawn[5]  # middle lane
        # transform_destination = all_deafault_spawn[5] # acceleration lane

        # create the leading vehicle
        ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
        # black color
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        vehicle_1 = world.spawn_actor(ego_vehicle_bp, transform_point)

        world.tick()
        # create platooning world
        platooning_world = PlatooningWorld()

        # setup managers
        vehicle_manager_1 = VehicleManager(vehicle_1, platooning_world, sample_resolution=4.5, buffer_size=8,
                                           debug_trajectory=True, debug=True, ignore_traffic_light=True)
        platooning_manager = PlatooningManager(platooning_world)

        # set leader
        platooning_manager.set_lead(vehicle_manager_1)

        # set destination TODO: the spawn point may have conflict
        destination = transform_destination.location + carla.Location(x=-160)
        # destination = carla.Location(x=319.547150, y=-82.862183, z=0.033884)

        platooning_manager.set_destination(destination)

        while True:
            world.tick()
            # top view
            spectator = world.get_spectator()
            transform = vehicle_1.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))
            # print(get_speed(vehicle_1))
            platooning_manager.update_information(platooning_world)
            platooning_manager.run_step()

    finally:
        world.apply_settings(origin_settings)
        platooning_manager.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
