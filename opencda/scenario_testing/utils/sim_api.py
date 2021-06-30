# -*- coding: utf-8 -*-
"""
Simulation API for create simulation world, vehicle manager and so on
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT
import sys

import carla

from opencda.core.common.vehicle_manager import VehicleManager
from opencda.core.application.platooning.platooning_manager import PlatooningManager
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.utils.customized_map_api import load_customized_world, bcolors


def createSimulationWorld(simulation_config, xodr_path=None, town=None):
    """
    Create client and simulation world.

    Args:
        -simulation_config (dict): configuration dictionary for simulation
        -xodr_path (string): optional, used only when customized map needed
        -town (string): default town name if not using customized map, eg. 'Town06'
    
    Return: 
        -client (carla.client): The client that is running the CARLA simulation. 
        -world (carla.world): The current simulation world.
        -map (carla.map): The map of the current simulation world.
        -origin setting (string): The settings of the current simulation world.
    """

    client = carla.Client('localhost', simulation_config['client_port'])
    client.set_timeout(10.0)

    if xodr_path:
        world = load_customized_world(xodr_path, client)
    elif town:
        try:
            world = client.load_world(town)
        except RuntimeError:
            print(f"{bcolors.FAIL} %s is not found in your CARLA repo! Please r"
                  f"download all town maps to your CARLA repo!{bcolors.ENDC}" % town)
    else:
        world = client.get_world()

    if not world:
        sys.exit('World loading failed')

    origin_settings = world.get_settings()
    new_settings = world.get_settings()

    if simulation_config['sync_mode']:
        new_settings.synchronous_mode = True
        new_settings.fixed_delta_seconds = simulation_config['fixed_delta_seconds']
    else:
        sys.exit('ERROR: Current version only supports sync simulation mode, v0.2 will support async mode.')

    world.apply_settings(new_settings)

    return client, world, world.get_map(), origin_settings


def car_blueprint_filter(blueprints):
    """
    Exclude the uncommon vehicles from the default CARLA blueprint library (i.e., isetta, carlacola, cybertruck, t2).
    """
    blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
    blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
    blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
    blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
    blueprints = [x for x in blueprints if not x.id.endswith('t2')]

    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    return blueprints


def createTrafficManager(client, world, traffic_config):
    """
    Create background traffic.

    Args:
        -client (carla.client): The client that is running the CARLA simulation. 
        -world (carla.world): The current simulation world.
        -traffic_config (dict): The simulation configuration file.
    Returns:
        -tm (carla.trafficmanager): The traffic manager that is controlling the background traffic.
        -bg_list (list): A list that contains all the background traffic vehicles.
    """

    tm = client.get_trafficmanager()

    tm.set_global_distance_to_leading_vehicle(traffic_config['global_distance'])
    tm.set_synchronous_mode(traffic_config['sync_mode'])
    tm.set_osm_mode(traffic_config['set_osm_mode'])
    tm.global_percentage_speed_difference(traffic_config['global_speed_perc'])

    bg_list = []

    blueprint_library = world.get_blueprint_library()
    # todo: traffic model should be random
    ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')

    for i, vehicle_config in enumerate(traffic_config['vehicle_list']):
        spawn_transform = carla.Transform(carla.Location(x=vehicle_config['spawn_position'][0],
                                                         y=vehicle_config['spawn_position'][1],
                                                         z=vehicle_config['spawn_position'][2]),
                                          carla.Rotation(pitch=vehicle_config['spawn_position'][5],
                                                         yaw=vehicle_config['spawn_position'][4],
                                                         roll=vehicle_config['spawn_position'][3]))
        ego_vehicle_bp.set_attribute('color', '0, 255, 0')
        vehicle = world.spawn_actor(ego_vehicle_bp, spawn_transform)
        vehicle.set_autopilot(True, 8000)

        if 'vehicle_speed_perc' in vehicle_config:
            tm.vehicle_percentage_speed_difference(vehicle, vehicle_config['vehicle_speed_perc'])
        tm.auto_lane_change(vehicle, traffic_config['auto_lane_change'])

        bg_list.append(vehicle)

    return tm, bg_list


def createPlatoonManagers(world, carla_map, scenario_params, apply_ml, map_helper=None):
    """
    Create Platooning Managers based on given params.

    Args:
        -world (carla.World): World from CARLA simulator.
        -carla_map (carla.Map): Map obtained from CARLA server.
        -scenario_params (dict): Platoon paramters.
        -apply_ml (bool): whether ml/dl model is included. Pytorch/sklearn required to install if set to true.
        -map_helper (function): Specific function to convert certain parameters to spawn position in certain map.

    Returns:
	   -platoon_list (list): A list contains all platoon members.
	   -cav_world (carla.World): The current simulation world.
    """

    platoon_list = []
    cav_world = CavWorld(apply_ml)

    # we use lincoln as default choice since our UCLA mobility lab use the same car
    cav_vehicle_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz2017')

    # create platoons
    for i, platoon in enumerate(scenario_params['scenario']['platoon_list']):
        platoon_manager = PlatooningManager(platoon, cav_world)
        for j, cav in enumerate(platoon['members']):
            if 'spawn_special' not in cav:
                spawn_transform = carla.Transform(carla.Location(x=cav['spawn_position'][0],
                                                                 y=cav['spawn_position'][1],
                                                                 z=cav['spawn_position'][2]),
                                                  carla.Rotation(pitch=cav['spawn_position'][5],
                                                                 yaw=cav['spawn_position'][4],
                                                                 roll=cav['spawn_position'][3]))
            else:
                spawn_transform = map_helper(carla_map, *cav['spawn_special'])

            cav_vehicle_bp.set_attribute('color', '0, 0, 0')
            vehicle = world.spawn_actor(cav_vehicle_bp, spawn_transform)

            # create vehicle manager for each cav
            vehicle_manager = VehicleManager(vehicle, cav, ['platooning'], carla_map, cav_world)
            # add the vehicle manager to platoon
            if j == 0:
                platoon_manager.set_lead(vehicle_manager)
            else:
                platoon_manager.add_member(vehicle_manager, leader=False)

        world.tick()
        destination = carla.Location(x=platoon['destination'][0],
                                     y=platoon['destination'][1],
                                     z=platoon['destination'][2])

        platoon_manager.set_destination(destination)
        platoon_manager.update_member_order()
        platoon_list.append(platoon_manager)

        return platoon_list, cav_world


def createVehicleManager(world, scenario_params, application, cav_world, carla_map, map_helper=None):
    """
    Create single CAV manager.
    
    Args:
        -world (carla.world): simulation world.
        -scenario_params (dict): scenario configuration.
        -application (string): the application purpose, a list, eg. ['single'].
        -cav_world (carla.world): object containing all cav info.
        -carla_map (carla.map): carla HD Map.
        -map_helper (opencda.map_helper): A function used for conveniently set the spawn position depending on different maps.
    Returns: 
        -single_cav_list (list): A list of vehicle managers.
    """

    cav_vehicle_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz2017')
    single_cav_list = []

    for i, cav in enumerate(scenario_params['scenario']['single_cav_list']):

        # if the spawn position is a single scalar, we need to use map helper to transfer to spawn transform
        if 'spawn_special' not in cav:
            spawn_transform = carla.Transform(carla.Location(x=cav['spawn_position'][0],
                                                             y=cav['spawn_position'][1],
                                                             z=cav['spawn_position'][2]),
                                              carla.Rotation(pitch=cav['spawn_position'][5],
                                                             yaw=cav['spawn_position'][4],
                                                             roll=cav['spawn_position'][3]))
        else:
            spawn_transform = map_helper(carla_map, *cav['spawn_special'])

        cav_vehicle_bp.set_attribute('color', '0, 0, 0')
        vehicle = world.spawn_actor(cav_vehicle_bp, spawn_transform)

        # create vehicle manager for each cav
        vehicle_manager = VehicleManager(vehicle, cav, application, carla_map, cav_world)
        world.tick()

        vehicle_manager.v2x_manager.set_platoon(None)

        destination = carla.Location(x=cav['destination'][0],
                                     y=cav['destination'][1],
                                     z=cav['destination'][2])
        vehicle_manager.update_info()
        vehicle_manager.set_destination(vehicle_manager.vehicle.get_location(),
                                        destination,
                                        clean=True)

        single_cav_list.append(vehicle_manager)

    return single_cav_list


def destroyActors(world):
    """
    Destroy all actors in the world.
    """

    actor_list = world.get_actors()
    for actor in actor_list:
        actor.destroy()
