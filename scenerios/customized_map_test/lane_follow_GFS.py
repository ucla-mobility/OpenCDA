# -*- coding: utf-8 -*-

"""Simple platooning following test
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla

from core.platooning.platooning_world import PlatooningWorld
from core.platooning.platooning_manager import PlatooningManager
from core.vehicle.vehicle_manager import VehicleManager
from scenerios.customized_map_test.load_customized_world import load_customized_world

# import GFS_controller
from model.GFS.GFS_controller import GFSController
import pickle
import pdb
import sys
import model.GFS.FISmodule as FISmodule
import model.GFS.FISmoduleGFSBestMergePoint as FISmoduleGFSBestMergePoint
# need to match this two modules for defuzz
sys.modules['FISmodule'] = FISmodule
sys.modules['FISmoduleGFSBestMergePoint'] = FISmoduleGFSBestMergePoint

# # import GFS_controller --- old version --
# from model.GFS.GFS_controller import GFSController
# from model.GFS.FISmoduleGFSBestMergePoint import FIS
# # import GFS dependencies
# import numpy as np
# import pdb
# import pickle

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

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # test without background traffic 
        # traffic manager for background traffic
        # traffic_manager = client.get_trafficmanager(8000)
        # traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        # traffic_manager.global_percentage_speed_difference(-80)
        # traffic_manager.set_synchronous_mode(True)

        # setup spawn points
        transform_1 = carla.Transform(carla.Location(x=-200.722836, y=7.500000, z=3.000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_2 = carla.Transform(carla.Location(x=-210.722836, y=7.500000, z=3.000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_3 = carla.Transform(carla.Location(x=-220.722836, y=7.500000, z=3.000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        transform_4 = carla.Transform(carla.Location(x=-230.722836, y=7.500000, z=3.000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        # merging vehicle spawn
        # merge line is wpt#11 -- wpt#2
        # wpt#11(x=-1204.197754, y=460.242096, z=3.000000) -- wpt#2(x=-4.621987, y=12.058071, z=3.000000) 
        merge_x = -120.722836
        merge_y = -0.37361877201042193*merge_x + 10.331209892811867 # calculatet merge line
        transform_5 = carla.Transform(carla.Location(merge_x, merge_y, z=3.000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))

        # destination
        transform_destination = carla.Transform(carla.Location(x=700.372955, y=7.500000, z=3.000000),
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

        world.tick()
        # add vehicle 5 as merging vehicle
        ego_vehicle_bp.set_attribute('color', '0, 255, 0')
        vehicle_5 = world.spawn_actor(ego_vehicle_bp, transform_5)
        vehicle_5.set_autopilot(True)

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
        # merger
        vehicle_manager_5 = VehicleManager(vehicle_5, platooning_world,sample_resolution=4.5, buffer_size=8,
                                           debug_trajectory=True, debug=False, ignore_traffic_light=True)

        platooning_manager = PlatooningManager(platooning_world)

        # set leader
        platooning_manager.set_lead(vehicle_manager_1)
        # add member
        platooning_manager.add_member(vehicle_manager_2)
        platooning_manager.add_member(vehicle_manager_3)
        platooning_manager.add_member(vehicle_manager_4)

        # set destination TODO: the spawn point may have conflict
        destination = transform_destination.location

        # set destination for platoon
        platooning_manager.set_destination(destination)
        # set destination for merger
        vehicle_manager_5.agent.set_destination(
            vehicle_manager_5.vehicle.get_location(), destination, clean=True)

        # manual spectator
        spectator = world.get_spectator()
        transform = vehicle_1.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                carla.Rotation(pitch=-90)))

        # load GFS model
        with open('/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestFIS-theBest37-Safe.pickle', 'rb') as f:
            gfs_m = pickle.load(f)

        with open('/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestFIS-pl-score.pickle', 'rb') as g:
            gfs_pl_score = pickle.load(g)

        with open('/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestGFS_PL_speed.pickle', 'rb') as h:
            gfs_pl_speed = pickle.load(h)

        while True:
            # if not world.wait_for_tick(10.0):
            #     continue
            world.tick()

            # not updating spectator
            # spectator = world.get_spectator()
            # transform = vehicle_1.get_transform()
            # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
            #                                         carla.Rotation(pitch=-90)))

            # get GFS output
            sensor_range = 150
            dt = 0.25
            GFS_contoller = GFSController(gfs_pl_score, gfs_pl_speed, gfs_m, platooning_manager, sensor_range, dt)
            # platoon position
            leadVeh, rearVeh = GFS_contoller.getBestMergePosition(vehicle_5, platooning_manager) # v5 is merger
            merge_speed = GFS_contoller.getDesiredSpeed_m(vehicle_5) # v5 is merger
            leader_speed = GFS_contoller.getDesiredSpeed_pl(vehicle_1)  # v1 is leader

            platooning_manager.update_information(platooning_world)
            platooning_manager.run_step()

    finally:
        platooning_manager.destroy()
        origin_settings.synchronous_mode = False
        world.apply_settings(origin_settings)

        vehicle_5.destroy()
        # vehicle_6.destroy()
        # vehicle_7.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
