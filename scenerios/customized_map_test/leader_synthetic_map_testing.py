# -*- coding: utf-8 -*-

"""Simple platooning following test
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import glob
import os
import sys
import carla

from core.agents.tools.misc import get_speed
from core.platooning.platooning_world import PlatooningWorld
from core.platooning.platooning_manager import PlatooningManager
from core.vehicle.vehicle_manager import VehicleManager
from scenerios.customized_map_test.load_customized_world import load_customized_world

# import GFS_controller
from model.GFS.GFS_controller import GFSController
from model.GFS.FISmoduleGFSBestMergePoint import FIS
# import GFS dependencies
import numpy as np
import pdb
import pickle


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

        # set sync mode
        # settings = world.get_settings()
        # settings.synchronous_mode = True
        # settings.fixed_delta_seconds = 0.05
        # world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # read all default waypoints
        all_deafault_spawn = world.get_map().get_spawn_points()

        # setup spawn points
        # merge lane
        transform_point = all_deafault_spawn[11]
        # move forward along acceleration lane 
        # transform_point.location.x = transform_point.location.x + 0.7*(all_deafault_spawn[2].location.x-all_deafault_spawn[11].location.x)
        # transform_point.location.y = transform_point.location.y + 0.7*(all_deafault_spawn[2].location.y-all_deafault_spawn[11].location.y)

        # main line
        transform_point = carla.Transform(carla.Location(x=-20.000000, y=7.500000, z=3.000000),
                                      carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))


        # destination 
        # transform_destination = all_deafault_spawn[4]  # left lane 
        transform_destination = all_deafault_spawn[3] # middle lane 
        # transform_destination = all_deafault_spawn[5] # acceleration lane 

        # create the leading vehicle
        ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
        # black color
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        vehicle_1 = world.spawn_actor(ego_vehicle_bp, transform_point)
        vehicle_1.apply_control(carla.VehicleControl(brake=1.0))

        # create platooning world
        platooning_world = PlatooningWorld()

        # setup managers
        vehicle_manager_1 = VehicleManager(vehicle_1, platooning_world, sample_resolution=4.5, buffer_size=8,
                                           debug_trajectory=True, debug=True, ignore_traffic_light=True)
        platooning_manager = PlatooningManager(platooning_world)

        # set leader
        platooning_manager.set_lead(vehicle_manager_1)

        # set destination TODO: the spawn point may have conflict
        destination = transform_destination.location
        # destination = carla.Location(x=319.547150, y=-82.862183, z=0.033884)
        platooning_manager.set_destination(destination)
        
        # # manaul spectator
        # spectator = world.get_spectator()
        # transform = vehicle_1.get_transform()
        # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),# + carla.Location(x=-65),
        #                                         carla.Rotation(pitch=-90)))

        # load rules for GFS
        with open(r'/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestFIS-theBest37-Safe.pickle','rb') as f:
            gfs_m_speed = pickle.load(f)

        with open(r'/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestFIS-pl-score.pickle','rb') as g:
            gfs_pl_score = pickle.load(g)

        with open(r'/home/xuhan/Carla-0.9.10/OpenPlatooning/model/GFS/BestGFS_PL_speed.pickle','rb') as h:
            gfs_pl_speed = pickle.load(h)

        # simulation loop
        while True:
            if not world.wait_for_tick(10.0):
                continue
            # top view
            spectator = world.get_spectator()
            transform = vehicle_1.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),# + carla.Location(x=-65),
                                                    carla.Rotation(pitch=-90)))
            # print(get_speed(vehicle_1))
            platooning_manager.update_information(world)
            platooning_manager.run_step()

            # read GFS controller output
            sensor_range = 150
            dt = 0.25
            GFS_contoller = GFSController(gfs_pl_score, gfs_pl_speed, gfs_m_speed, platooning_manager, sensor_range, dt)
            # platoon position
            leadVeh, rearVeh = GFS_contoller.getBestMergePosition(vehicle_1, platooning_manager)
            merge_speed = GFS_contoller.getDesiredSpeed_m(vehicle_1)
            leader_speed = GFS_contoller.getDesiredSpeed_pl(vehicle_1) # v1 is leader
    finally:
        platooning_manager.destroy()
        world.apply_settings(origin_settings)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
