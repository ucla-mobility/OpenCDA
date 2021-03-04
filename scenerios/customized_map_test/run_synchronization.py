#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Script to integrate CARLA and SUMO simulations

@authors: Yi Guo, Jiaqi Ma
"""
import argparse
import logging
import os
import sys
import time
import pickle

import carla

from co_simulation.sumo_integration.bridge_helper import BridgeHelper
from co_simulation.sumo_integration.carla_simulation import CarlaSimulation
from co_simulation.sumo_integration.constants import INVALID_ACTOR_ID
from co_simulation.sumo_integration.sumo_simulation import SumoSimulation
from co_simulation.sumo_src.simulationmanager import SimulationManager
from core.platooning.platooning_world import PlatooningWorld
from core.platooning.platooning_manager import PlatooningManager
from core.platooning.platooning_plugin import FSM
from core.vehicle.vehicle_manager import VehicleManager

os.environ['SUMO_HOME'] = "/usr/share/sumo"
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """

    def __init__(self,
                 sumo_simulation,
                 carla_simulation,
                 tls_manager='none',
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False):

        self.sumo = sumo_simulation
        self.carla = carla_simulation
        # add a simulation manager
        self.manager = SimulationManager(pCreation=True, iCoordination=False, iZipping=False)

        self.tls_manager = tls_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        if tls_manager == 'carla':
            self.sumo.switch_off_traffic_lights()
        elif tls_manager == 'sumo':
            self.carla.switch_off_traffic_lights()

        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.
        self.carla2sumo_ids = {}  # Contains only actors controlled by carla.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        BridgeHelper.offset = self.sumo.get_net_offset()

        # Configuring carla simulation in sync mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.carla.step_length
        self.carla.world.apply_settings(settings)

    def tick(self, time_tmp):
        """
        Tick to simulation synchronization
        """
        # -----------------
        # sumo-->carla sync
        # -----------------

        # add platooning protocol here
        self.manager.handleSimulationStep(time_tmp)
        self.sumo.tick()

        # Spawning new sumo actors in carla (i.e, not controlled by carla).
        sumo_spawned_actors = self.sumo.spawned_actors - set(self.carla2sumo_ids.values())
        for sumo_actor_id in sumo_spawned_actors:
            self.sumo.subscribe(sumo_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.sync_vehicle_color)
            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                                   sumo_actor.extent)

                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
            else:
                self.sumo.unsubscribe(sumo_actor_id)

        # Destroying sumo arrived actors in carla.
        for sumo_actor_id in self.sumo.destroyed_actors:
            if sumo_actor_id in self.sumo2carla_ids:
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))

        # Updating sumo actors in carla.
        for sumo_actor_id in self.sumo2carla_ids:
            carla_actor_id = self.sumo2carla_ids[sumo_actor_id]

            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            carla_actor = self.carla.get_actor(carla_actor_id)

            carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                               sumo_actor.extent)
            if self.sync_vehicle_lights:
                carla_lights = BridgeHelper.get_carla_lights_state(carla_actor.get_light_state(),
                                                                   sumo_actor.signals)
            else:
                carla_lights = None

            self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)

        # Updates traffic lights in carla based on sumo information.
        if self.tls_manager == 'sumo':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                sumo_tl_state = self.sumo.get_traffic_light_state(landmark_id)
                carla_tl_state = BridgeHelper.get_carla_traffic_light_state(sumo_tl_state)

                self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)

        # -----------------
        # carla-->sumo sync
        # -----------------
        self.carla.tick()

        # Spawning new carla actors (not controlled by sumo)
        carla_spawned_actors = self.carla.spawned_actors - set(self.sumo2carla_ids.values())
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.carla.get_actor(carla_actor_id)

            type_id = BridgeHelper.get_sumo_vtype(carla_actor)
            color = carla_actor.attributes.get('color', None) if self.sync_vehicle_color else None
            if type_id is not None:
                sumo_actor_id = self.sumo.spawn_actor(type_id, color)
                if sumo_actor_id != INVALID_ACTOR_ID:
                    self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                    self.sumo.subscribe(sumo_actor_id)

        # Destroying required carla actors in sumo.
        for carla_actor_id in self.carla.destroyed_actors:
            if carla_actor_id in self.carla2sumo_ids:
                self.sumo.destroy_actor(self.carla2sumo_ids.pop(carla_actor_id))

        # Updating carla actors in sumo.
        for carla_actor_id in self.carla2sumo_ids:
            sumo_actor_id = self.carla2sumo_ids[carla_actor_id]

            carla_actor = self.carla.get_actor(carla_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            sumo_transform = BridgeHelper.get_sumo_transform(carla_actor.get_transform(),
                                                             carla_actor.bounding_box.extent)
            if self.sync_vehicle_lights:
                carla_lights = self.carla.get_actor_light_state(carla_actor_id)
                if carla_lights is not None:
                    sumo_lights = BridgeHelper.get_sumo_lights_state(sumo_actor.signals,
                                                                     carla_lights)
                else:
                    sumo_lights = None
            else:
                sumo_lights = None

            self.sumo.synchronize_vehicle(sumo_actor_id, sumo_transform, sumo_lights)

        # Updates traffic lights in sumo based on carla information.
        if self.tls_manager == 'carla':
            common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
            for landmark_id in common_landmarks:
                carla_tl_state = self.carla.get_traffic_light_state(landmark_id)
                sumo_tl_state = BridgeHelper.get_sumo_traffic_light_state(carla_tl_state)

                # Updates all the sumo links related to this landmark.
                self.sumo.synchronize_traffic_light(landmark_id, sumo_tl_state)

    def close(self):
        """
        Cleans synchronization.
        """
        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.sumo2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)

        for sumo_actor_id in self.carla2sumo_ids.values():
            self.sumo.destroy_actor(sumo_actor_id)

        # Closing sumo and carla client.
        self.carla.close()
        self.sumo.close()


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    sumo_simulation = SumoSimulation(args.sumo_cfg_file, args.step_length, args.sumo_host,
                                     args.sumo_port, args.sumo_gui, args.client_order)
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length,
                                       '../../customized_map_output/map_v7.3_SUMO_full.xodr')

    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)

    blueprint_library = carla_simulation.world.get_blueprint_library()

    # get all spawn points
    all_deafault_spawn = carla_simulation.world.get_map().get_spawn_points()
    transform_point = all_deafault_spawn[11]
    # move forward along acceleration lane 0.630 for cut-in-joining, 623 for back joining
    transform_point.location.x = transform_point.location.x + \
                                 0.44 * (all_deafault_spawn[2].location.x - all_deafault_spawn[11].location.x)
    transform_point.location.y = transform_point.location.y + \
                                 0.44 * (all_deafault_spawn[2].location.y - all_deafault_spawn[11].location.y)

    # setup spawn points
    transform_1 = carla.Transform(carla.Location(x=-650.722836, y=7.500000, z=3.000000),
                                  carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
    transform_2 = carla.Transform(carla.Location(x=-660.722836, y=7.500000, z=3.000000),
                                  carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
    transform_3 = carla.Transform(carla.Location(x=-670.722836, y=7.500000, z=3.000000),
                                  carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
    transform_4 = transform_point

    transform_5 = carla.Transform(carla.Location(x=-480.722836, y=7.500000, z=3.000000),
                                  carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))

    transform_destination_1 = carla.Transform(carla.Location(x=700.372955, y=7.500000, z=3.000000),
                                              carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))

    transform_destination_2 = all_deafault_spawn[5]

    # create the leading vehicle
    ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
    # black color
    ego_vehicle_bp.set_attribute('color', '0, 0, 0')
    vehicle_1 = carla_simulation.world.spawn_actor(ego_vehicle_bp, transform_1)

    ego_vehicle_bp.set_attribute('color', '255, 255, 255')
    vehicle_2 = carla_simulation.world.spawn_actor(ego_vehicle_bp, transform_2)

    ego_vehicle_bp.set_attribute('color', '255, 255, 255')
    vehicle_3 = carla_simulation.world.spawn_actor(ego_vehicle_bp, transform_3)

    ego_vehicle_bp.set_attribute('color', '255, 255, 255')
    vehicle_4 = carla_simulation.world.spawn_actor(ego_vehicle_bp, transform_4)

    # ego_vehicle_bp.set_attribute('color', '0, 255, 0')
    # vehicle_5 = carla_simulation.world.spawn_actor(ego_vehicle_bp, transform_5)
    # vehicle_5.apply_control(carla.VehicleControl(throttle=0.75))

    carla_simulation.world.tick()

    # create platooning world
    platooning_world = PlatooningWorld()

    # setup managers
    vehicle_manager_1 = VehicleManager(vehicle_1, platooning_world, sample_resolution=6.0, buffer_size=8,
                                       debug_trajectory=False, debug=False, ignore_traffic_light=True)
    vehicle_manager_2 = VehicleManager(vehicle_2, platooning_world, debug_trajectory=False, debug=False)
    vehicle_manager_3 = VehicleManager(vehicle_3, platooning_world, debug_trajectory=False, debug=False)

    vehicle_manager_4 = VehicleManager(vehicle_4, platooning_world, status=FSM.SEARCHING, sample_resolution=4.5,
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
    spectator = carla_simulation.world.get_spectator()

    try:
        time_tmp = 0

        while True:
            start = time.time()

            synchronization.tick(time_tmp)

            transform = vehicle_4.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

            platooning_manager.update_information(platooning_world)
            platooning_manager.run_step()
            in_platooning, _, _ = vehicle_manager_4.get_platooning_status()
            if not in_platooning:
                vehicle_manager_4.agent.update_information(platooning_world)
                control = vehicle_manager_4.run_step()
                vehicle_manager_4.vehicle.apply_control(control)

            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)
            time_tmp += 0.25

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning synchronization')

        synchronization.close()
        platooning_manager.destroy()
        vehicle_4.destroy()
        # vehicle_5.destroy()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--sumo_cfg_file', type=str, help='sumo configuration file',
                           default='../../customized_map_output/OpenDriveMap73.sumocfg')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default=None,
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=None,
                           type=int,
                           help='TCP port to listen to (default: 8813)')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo', default=True)
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)
