# -*- coding: utf-8 -*-
"""
Utilize scenario manager to manage CARLA simulation construction. This script
is used for carla simulation only, and if you want to manage the Co-simulation,
please use cosim_api.py.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import random
import sys
from random import shuffle

import carla

from opencda.core.common.vehicle_manager import VehicleManager
from opencda.core.application.platooning.platooning_manager import \
    PlatooningManager
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.utils.customized_map_api import \
    load_customized_world, bcolors


def car_blueprint_filter(blueprint_library):
    """
    Exclude the uncommon vehicles from the default CARLA blueprint library
    (i.e., isetta, carlacola, cybertruck, t2).

    Parameters
    ----------
    blueprint_library : carla.blueprint_library
        The blueprint library that contains all models.

    Returns
    -------
    blueprints : list
        The list of suitable blueprints for vehicles.
    """

    blueprints = [
        blueprint_library.find('vehicle.audi.a2'),
        blueprint_library.find('vehicle.audi.tt'),
        blueprint_library.find('vehicle.dodge_charger.police'),
        blueprint_library.find('vehicle.jeep.wrangler_rubicon'),
        blueprint_library.find('vehicle.chevrolet.impala'),
        blueprint_library.find('vehicle.mini.cooperst'),
        blueprint_library.find('vehicle.audi.etron'),
        blueprint_library.find('vehicle.mercedes-benz.coupe'),
        blueprint_library.find('vehicle.bmw.grandtourer'),
        blueprint_library.find('vehicle.toyota.prius'),
        blueprint_library.find('vehicle.citroen.c3'),
        blueprint_library.find('vehicle.mustang.mustang'),
        blueprint_library.find('vehicle.tesla.model3'),
        blueprint_library.find('vehicle.lincoln.mkz2017'),
        blueprint_library.find('vehicle.seat.leon'),
        blueprint_library.find('vehicle.nissan.patrol'),
        blueprint_library.find('vehicle.nissan.micra'),
    ]
    return blueprints


class ScenarioManager:
    """
    The manager that controls simulation construction, backgound traffic
    generation and CAVs spawning.

    Parameters
    ----------
    scenario_params : dict
        The dictionary contains all simulation configurations.

    xodr_path : str
        The xodr file to the customized map, default: None.

    town : str
        Town name if not using customized map, eg. 'Town06'.

    apply_ml : bool
        Whether need to load dl/ml model(pytorch required) in this simulation.

    Attributes
    ----------
    client : carla.client
        The client that connects to carla server.

    world : carla.world
        Carla simulation server.

    origin_settings : dict
        The origin setting of the simulation server.

    cav_world : opencda object
        CAV World that contains the information of all CAVs.

    carla_map : carla.map
        Car;a HD Map.

    """

    def __init__(self, scenario_params,
                 apply_ml,
                 xodr_path=None,
                 town=None,
                 cav_world=None):
        self.scenario_params = scenario_params

        simulation_config = scenario_params['world']
        self.client = \
            carla.Client('localhost', simulation_config['client_port'])
        self.client.set_timeout(10.0)

        if xodr_path:
            self.world = load_customized_world(xodr_path, self.client)
        elif town:
            try:
                self.world = self.client.load_world(town)
            except RuntimeError:
                print(
                    f"{bcolors.FAIL} %s is not found in your CARLA repo! "
                    f"Please download all town maps to your CARLA "
                    f"repo!{bcolors.ENDC}" % town)
        else:
            self.world = self.client.get_world()

        if not self.world:
            sys.exit('World loading failed')

        self.origin_settings = self.world.get_settings()
        new_settings = self.world.get_settings()

        if simulation_config['sync_mode']:
            new_settings.synchronous_mode = True
            new_settings.fixed_delta_seconds = \
                simulation_config['fixed_delta_seconds']
        else:
            sys.exit(
                'ERROR: Current version only supports sync simulation mode, '
                'v0.2 will support async mode.')

        self.world.apply_settings(new_settings)

        self.cav_world = cav_world
        self.carla_map = self.world.get_map()
        self.apply_ml = apply_ml

    def create_vehicle_manager(self, application,
                               map_helper=None,
                               data_dump=False):
        """
        Create a list of single CAVs.

        Parameters
        ----------
        application : list
            The application purpose, a list, eg. ['single'], ['platoon'].

        map_helper : function
            A function to help spawn vehicle on a specific position in
            a specific map.

        data_dump : bool
            Whether to dump sensor data.

        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager.
        """
        print('Creating single CAVs.')
        # By default, we use lincoln as our cav model.
        cav_vehicle_bp = \
            self.world.get_blueprint_library().find('vehicle.lincoln.mkz2017')
        single_cav_list = []

        for i, cav_config in enumerate(
                self.scenario_params['scenario']['single_cav_list']):

            # if the spawn position is a single scalar, we need to use map
            # helper to transfer to spawn transform
            if 'spawn_special' not in cav_config:
                spawn_transform = carla.Transform(
                    carla.Location(
                        x=cav_config['spawn_position'][0],
                        y=cav_config['spawn_position'][1],
                        z=cav_config['spawn_position'][2]),
                    carla.Rotation(
                        pitch=cav_config['spawn_position'][5],
                        yaw=cav_config['spawn_position'][4],
                        roll=cav_config['spawn_position'][3]))
            else:
                spawn_transform = map_helper(self.carla_map,
                                             *cav_config['spawn_special'])

            cav_vehicle_bp.set_attribute('color', '0, 0, 255')
            vehicle = self.world.spawn_actor(cav_vehicle_bp, spawn_transform)

            # create vehicle manager for each cav
            vehicle_manager = VehicleManager(
                vehicle, cav_config, application,
                self.carla_map, self.cav_world,
                current_time=self.scenario_params['current_time'],
                data_dumping=data_dump)

            self.world.tick()

            vehicle_manager.v2x_manager.set_platoon(None)

            destination = carla.Location(x=cav_config['destination'][0],
                                         y=cav_config['destination'][1],
                                         z=cav_config['destination'][2])
            vehicle_manager.update_info()
            vehicle_manager.set_destination(
                vehicle_manager.vehicle.get_location(),
                destination,
                clean=True)

            single_cav_list.append(vehicle_manager)

        return single_cav_list

    def create_platoon_manager(self, map_helper=None, data_dump=False):
        """
        Create a list of platoons.

        Parameters
        ----------
        map_helper : function
            A function to help spawn vehicle on a specific position in a
            specific map.

        data_dump : bool
            Whether to dump sensor data.

        Returns
        -------
        single_cav_list : list
            A list contains all single CAVs' vehicle manager.
        """
        print('Creating platoons/')
        platoon_list = []
        self.cav_world = CavWorld(self.apply_ml)

        # we use lincoln as default choice since our UCLA mobility lab use the
        # same car
        cav_vehicle_bp = \
            self.world.get_blueprint_library().find('vehicle.lincoln.mkz2017')

        # create platoons
        for i, platoon in enumerate(
                self.scenario_params['scenario']['platoon_list']):
            platoon_manager = PlatooningManager(platoon, self.cav_world)
            for j, cav in enumerate(platoon['members']):
                if 'spawn_special' not in cav:
                    spawn_transform = carla.Transform(
                        carla.Location(
                            x=cav['spawn_position'][0],
                            y=cav['spawn_position'][1],
                            z=cav['spawn_position'][2]),
                        carla.Rotation(
                            pitch=cav['spawn_position'][5],
                            yaw=cav['spawn_position'][4],
                            roll=cav['spawn_position'][3]))
                else:
                    spawn_transform = map_helper(self.carla_map,
                                                 *cav['spawn_special'])

                cav_vehicle_bp.set_attribute('color', '0, 0, 255')
                vehicle = self.world.spawn_actor(cav_vehicle_bp,
                                                 spawn_transform)

                # create vehicle manager for each cav
                vehicle_manager = VehicleManager(
                    vehicle, cav, ['platooning'],
                    self.carla_map, self.cav_world,
                    current_time=self.scenario_params['current_time'],
                    data_dumping=data_dump)

                # add the vehicle manager to platoon
                if j == 0:
                    platoon_manager.set_lead(vehicle_manager)
                else:
                    platoon_manager.add_member(vehicle_manager, leader=False)

            self.world.tick()
            destination = carla.Location(x=platoon['destination'][0],
                                         y=platoon['destination'][1],
                                         z=platoon['destination'][2])

            platoon_manager.set_destination(destination)
            platoon_manager.update_member_order()
            platoon_list.append(platoon_manager)

        return platoon_list

    def spawn_vehicles_by_list(self, tm, traffic_config, bg_list):
        """
        Spawn the traffic vehicles by the given list.

        Parameters
        ----------
        tm : carla.TrafficManager
            Traffic manager.

        traffic_config : dict
            Background traffic configuration.

        bg_list : list
            The list contains all background traffic.

        Returns
        -------
        bg_list : list
            Update traffic list.
        """

        blueprint_library = self.world.get_blueprint_library()

        ego_vehicle_random_list = car_blueprint_filter(blueprint_library)
        # if not random select, we always choose lincoln.mkz with green color
        ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')

        for i, vehicle_config in enumerate(traffic_config['vehicle_list']):
            spawn_transform = carla.Transform(
                carla.Location(
                    x=vehicle_config['spawn_position'][0],
                    y=vehicle_config['spawn_position'][1],
                    z=vehicle_config['spawn_position'][2]),
                carla.Rotation(
                    pitch=vehicle_config['spawn_position'][5],
                    yaw=vehicle_config['spawn_position'][4],
                    roll=vehicle_config['spawn_position'][3]))

            if not traffic_config['random']:
                ego_vehicle_bp.set_attribute('color', '0, 255, 0')

            else:
                ego_vehicle_bp = random.choice(ego_vehicle_random_list)

                color = random.choice(
                    ego_vehicle_bp.get_attribute('color').recommended_values)
                ego_vehicle_bp.set_attribute('color', color)

            vehicle = self.world.spawn_actor(ego_vehicle_bp, spawn_transform)
            vehicle.set_autopilot(True, 8000)

            if 'vehicle_speed_perc' in vehicle_config:
                tm.vehicle_percentage_speed_difference(
                    vehicle, vehicle_config['vehicle_speed_perc'])
            tm.auto_lane_change(vehicle, traffic_config['auto_lane_change'])

            bg_list.append(vehicle)

        return bg_list

    def spawn_vehicle_by_range(self, tm, traffic_config, bg_list):
        """
        Spawn the traffic vehicles by the given range.

        Parameters
        ----------
        tm : carla.TrafficManager
            Traffic manager.

        traffic_config : dict
            Background traffic configuration.

        bg_list : list
            The list contains all background traffic.

        Returns
        -------
        bg_list : list
            Update traffic list.
        """
        blueprint_library = self.world.get_blueprint_library()

        ego_vehicle_random_list = car_blueprint_filter(blueprint_library)
        # if not random select, we always choose lincoln.mkz with green color
        ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')

        spawn_num = traffic_config['vehicle_list']
        spawn_range = traffic_config['range']

        x_min, x_max, y_min, y_max = \
            math.floor(spawn_range[0]), math.ceil(spawn_range[1]), \
            math.floor(spawn_range[2]), math.ceil(spawn_range[3])

        spawn_set = set()

        for x in range(x_min, x_max, int(spawn_range[4])):
            for y in range(y_min, y_max, int(spawn_range[5])):
                location = carla.Location(x=x, y=y, z=0.3)
                way_point = self.carla_map.get_waypoint(location).transform

                spawn_set.add((way_point.location.x,
                               way_point.location.y,
                               way_point.location.z,
                               way_point.rotation.roll,
                               way_point.rotation.yaw,
                               way_point.rotation.pitch))
        count = 0
        spawn_list = list(spawn_set)
        shuffle(spawn_list)

        while count < spawn_num:
            if len(spawn_list) == 0:
                break

            coordinates = spawn_list[0]
            spawn_list.pop(0)

            spawn_transform = carla.Transform(carla.Location(x=coordinates[0],
                                                             y=coordinates[1],
                                                             z=coordinates[
                                                                   2] + 0.3),
                                              carla.Rotation(
                                                  roll=coordinates[3],
                                                  yaw=coordinates[4],
                                                  pitch=coordinates[5]))
            if not traffic_config['random']:
                ego_vehicle_bp.set_attribute('color', '0, 255, 0')

            else:
                ego_vehicle_bp = random.choice(ego_vehicle_random_list)

                color = random.choice(
                    ego_vehicle_bp.get_attribute('color').recommended_values)
                ego_vehicle_bp.set_attribute('color', color)

            vehicle = \
                self.world.try_spawn_actor(ego_vehicle_bp, spawn_transform)

            if not vehicle:
                continue

            vehicle.set_autopilot(True, 8000)
            tm.auto_lane_change(vehicle, traffic_config['auto_lane_change'])

            # each vehicle have slight different speed
            tm.vehicle_percentage_speed_difference(
                vehicle,
                traffic_config['global_speed_perc'] + random.randint(-30, 30))

            bg_list.append(vehicle)
            count += 1

        return bg_list

    def create_traffic_carla(self):
        """
        Create traffic flow.

        Returns
        -------
        tm : carla.traffic_manager
            Carla traffic manager.

        bg_list : list
            The list that contains all the background traffic vehicles.
        """
        print('Spawning CARLA traffic flow.')
        traffic_config = self.scenario_params['carla_traffic_manager']
        tm = self.client.get_trafficmanager()

        tm.set_global_distance_to_leading_vehicle(
            traffic_config['global_distance'])
        tm.set_synchronous_mode(traffic_config['sync_mode'])
        tm.set_osm_mode(traffic_config['set_osm_mode'])
        tm.global_percentage_speed_difference(
            traffic_config['global_speed_perc'])

        bg_list = []

        if isinstance(traffic_config['vehicle_list'], list):
            bg_list = self.spawn_vehicles_by_list(tm,
                                                  traffic_config,
                                                  bg_list)

        elif isinstance(traffic_config['vehicle_list'], int):
            bg_list = self.spawn_vehicle_by_range(tm, traffic_config, bg_list)

        else:
            sys.exit('Traffic vehicle list param has to be a list or int!')
        print('CARLA traffic flow generated.')
        return tm, bg_list

    def tick(self):
        """
        Tick the server.
        """
        self.world.tick()

    def destroyActors(self):
        """
        Destroy all actors in the world.
        """

        actor_list = self.world.get_actors()
        for actor in actor_list:
            actor.destroy()

    def close(self):
        """
        Simulation close.
        """
        # restore to origin setting
        self.world.apply_settings(self.origin_settings)