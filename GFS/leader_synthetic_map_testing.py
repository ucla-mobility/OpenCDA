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

# import Vehicle class 
from src.vehicle_CARLA import Vehicle # use to replace SUMO vehicle class(extract data from simulation)


def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        xodr_path = 'map_output/map_v7.3_SUMO_full.xodr'
        if os.path.exists(xodr_path):
            with open(xodr_path) as od_file:
                try:
                    data = od_file.read()
                except OSError:
                    print('file could not be readed.')
                    sys.exit()
            print('load opendrive map %r.' % os.path.basename(xodr_path))
            vertex_distance = 2.0  # in meters
            max_road_length = 500.0 # in meters
            wall_height = 1.0      # in meters
            extra_width = 0.6      # in meters
            world = client.generate_opendrive_world(
                data, carla.OpendriveGenerationParameters(
                    vertex_distance=vertex_distance,
                    max_road_length=max_road_length,
                    wall_height=wall_height,
                    additional_width=extra_width,
                    smooth_junctions=False,
                    enable_mesh_visibility=True))
        else:
            print('file not found.')
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
        transform_point = all_deafault_spawn[11]
        # move forward along acceleration lane 
        transform_point.location.x = transform_point.location.x + 0.7*(all_deafault_spawn[2].location.x-all_deafault_spawn[11].location.x)
        transform_point.location.y = transform_point.location.y + 0.7*(all_deafault_spawn[2].location.y-all_deafault_spawn[11].location.y)
        
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

        # init Vehicle class to read simulation data 
        leader = Vehicle(vehicle_1)

        # create platooning world
        platooning_world = PlatooningWorld()

        # setup managers
        vehicle_manager_1 = VehicleManager(vehicle_1, platooning_world, sample_resolution=4.5, buffer_size=8,
                                           debug_trajectory=True, debug=True, ignore_traffic_light=True)
        platooning_manager = PlatooningManager(platooning_world)

        # set leader
        platooning_manager.set_lead(vehicle_manager_1)

        # set destination TODO: the spawn point may have conflict
        destination = transform_destination.location #+ carla.Location(x=-160)
        # destination = carla.Location(x=319.547150, y=-82.862183, z=0.033884)
        platooning_manager.set_destination(destination)
        
        # top view 
        spectator = world.get_spectator()
        transform = vehicle_1.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),# + carla.Location(x=-65),
                                                carla.Rotation(pitch=-90)))

        # simulation loop
        while True:
            if not world.wait_for_tick(10.0):
                continue
            # # top view 
            # spectator = world.get_spectator()
            # transform = vehicle_1.get_transform()
            # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),# + carla.Location(x=-65),
            #                                         carla.Rotation(pitch=-90)))
            # print(get_speed(vehicle_1))
            platooning_manager.update_information(world)
            platooning_manager.run_step()

            # ---------------------- read simulation data ---------------------
            # nearby vehicles
            leadVeh = leader.getLeader(50)
            leftLeadVeh = leader.getLeftLeader()
            rightLeadVeh = leader.getRightLeader()
            leftFollowVeh = leader.getLeftFollower()
            rightFollowVeh = leader.getRightFollower()

            # leader info
            l_id = leader.getName()
            l_state = leader.getState()
            l_position = leader.getPosition()
            l_speed = leader.getSpeed()
            l_lane = leader.getLane()
            l_tar_lane = leader.getTargetLane()
            l_active = leader.isActive()
            l_edge = leader.getEdge()
            l_lane_index = leader.getLaneIndex()
            if leadVeh is not None:
                lead, dist = leadVeh
                l_distance_to_front = leader.getDistance(Vehicle(lead))
            l_max_speed = leader.getMaxSpeed()

            # # setters 
            # leader.setState("RequestDissovle")
            # l_state = leader.getState()
            # leader.setTau(0.66776677)
            # l_Tau = leader.getTau()
            # leader.setInActive()
            # leader.setMinGap(10.010101)
            # leader.setSpeedFactor(0101)
            # leader.setTargetLane(-1)

    finally:
        platooning_manager.destroy()
        world.apply_settings(origin_settings)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
