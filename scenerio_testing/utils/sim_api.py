# -*- coding: utf-8 -*-
"""
Simulation API for create simulation world, vehicle manager and so on
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT
import sys

import carla

from core.common.vehicle_manager import VehicleManager
from core.application.platooning.platooning_manager import PlatooningManager
from core.application.platooning.platooning_world import PlatooningWorld
from scenerio_testing.utils.customized_map_api import load_customized_world


def createSimulationWorld(simulation_config, xodr_path=None):
    """
    Create client and simulation world
    :param simulation_config: configuration dictionary for simulation
    :param xodr_path: optional, used only when customized map needed
    :return: simulation world, origin setting
    """

    client = carla.Client('localhost', simulation_config['client_port'])
    client.set_timeout(2.0)
    world = load_customized_world(xodr_path, client)

    if not world:
        sys.exit()

    origin_settings = world.get_settings()
    new_settings = world.get_settings()

    if simulation_config['sync_mode']:
        new_settings.synchronous_mode = True
        new_settings.fixed_delta_seconds = simulation_config['fixed_delta_seconds']
    else:
        sys.exit('Current version only support sync mode')

    world.apply_settings(new_settings)

    return world, origin_settings


def createPlatoonManagers(world, scenario_params, map_helper=None):
    """
    Create platoon managers based on the yaml file
    :param world: simulation world
    :param scenario_params: configuration of scenario
    :param map_helper: A function used for conveniently set the spawn position depending on different maps
    :return: a list of platoon managers, platoon world
    """

    platoon_list = []
    platooning_world = PlatooningWorld()

    # we use lincoln as default choice since our UCLA mobility lab use the same car
    cav_vehicle_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz2017')

    # create platoons
    for i, platoon in enumerate(scenario_params['scenario']['platoon_list']):
        platoon_manager = PlatooningManager(platoon, platooning_world)
        for j, cav in enumerate(platoon['members']):
            if 'spawn_special' not in cav:
                spawn_transform = carla.Transform(carla.Location(x=cav['spawn_position'][0],
                                                                 y=cav['spawn_position'][1],
                                                                 z=cav['spawn_position'][2]),
                                                  carla.Rotation(pitch=cav['spawn_position'][5],
                                                                 yaw=cav['spawn_position'][4],
                                                                 roll=cav['spawn_position'][3]))
            else:
                spawn_transform = map_helper(world.get_map(), *cav['spawn_special'])

            cav_vehicle_bp.set_attribute('color', '0, 0, 0')
            vehicle = world.spawn_actor(cav_vehicle_bp, spawn_transform)

            # create vehicle manager for each cav
            vehicle_manager = VehicleManager(vehicle, cav, ['platooning'], platooning_world)
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

        return platoon_list, platooning_world


def createVehicleManager(world, scenario_params, application, platooning_world, map_helper=None):
    """
    Create single CAV manager
    :param world: simulation world
    :param scenario_params: scenario configuration
    :param application: the application purpose, a list, eg. ['single']
    :param platooning_world: object containing all platoon info
    :param map_helper: A function used for conveniently set the spawn position depending on different maps
    :return: a list of vehicle managers
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
            spawn_transform = map_helper(world.get_map(), *cav['spawn_special'])

        cav_vehicle_bp.set_attribute('color', '0, 0, 0')
        vehicle = world.spawn_actor(cav_vehicle_bp, spawn_transform)

        # create vehicle manager for each cav
        vehicle_manager = VehicleManager(vehicle, cav, application, platooning_world)
        vehicle_manager.v2x_manager.set_platoon(None)

        world.tick()

        destination = carla.Location(x=cav['destination'][0],
                                     y=cav['destination'][1],
                                     z=cav['destination'][2])
        vehicle_manager.update_info(platooning_world)
        vehicle_manager.set_destination(vehicle_manager.vehicle.get_location(),
                                        destination,
                                        clean=True)

        single_cav_list.append(vehicle_manager)

    return single_cav_list
