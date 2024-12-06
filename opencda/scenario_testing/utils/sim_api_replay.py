
import random
import sys
import json
from omegaconf import OmegaConf

import carla
import numpy as np

from opencda.core.common.vehicle_manager_replay import VehicleManagerReplay
from opencda.core.common.rsu_manager_replay import RSUManagerReplay, RSUManagerReplayPly
from opencda.scenario_testing.utils.customized_map_api import load_customized_world, bcolors
from opencda.scenario_testing.utils.sim_api import car_blueprint_filter, multi_class_vehicle_blueprint_filter, ScenarioManager

class ScenarioManagerReplay(ScenarioManager):
    
    def __init__(self, scenario_params,
                 apply_ml,
                 carla_version,
                 xodr_path=None,
                 town=None,
                 cav_world=None):
        self.scenario_params = scenario_params
        self.carla_version = carla_version

        simulation_config = scenario_params['world']

        # set random seed if stated
        if 'seed' in simulation_config:
            np.random.seed(simulation_config['seed'])
            random.seed(simulation_config['seed'])

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
                'ERROR: Current version only supports sync simulation mode')

        self.world.apply_settings(new_settings)

        # set weather
        weather = self.set_weather(simulation_config['weather'])
        self.world.set_weather(weather)

        # Define probabilities for each type of blueprint
        self.use_multi_class_bp = scenario_params["blueprint"][
            'use_multi_class_bp'] if 'blueprint' in scenario_params else False
        if self.use_multi_class_bp:
            # bbx/blueprint meta
            with open(scenario_params['blueprint']['bp_meta_path']) as f:
                self.bp_meta = json.load(f)
            self.bp_class_sample_prob = scenario_params['blueprint'][
                'bp_class_sample_prob']

            # normalize probability
            self.bp_class_sample_prob = {
                k: v / sum(self.bp_class_sample_prob.values()) for k, v in
                self.bp_class_sample_prob.items()}

        self.cav_world = cav_world
        self.carla_map = self.world.get_map()
        self.apply_ml = apply_ml

    def create_vehicle_manager(self, cav_positions, cur_timestamp, application,
                               map_helper=None,
                               data_dump=False):
        print('Creating single CAVs.')
        # By default, we use lincoln as our cav model.
        default_model = 'vehicle.lincoln.mkz2017' \
            if self.carla_version == '0.9.11' else 'vehicle.lincoln.mkz_2017'

        cav_vehicle_bp = \
            self.world.get_blueprint_library().find(default_model)
        single_cav_dict = {}

        cav_configs = self.scenario_params['scenario']['single_cav_list']

        for i, (cav_id, cav_pos) in enumerate(cav_positions.items()):

            cav_config = cav_configs[i]

            # in case the cav wants to join a platoon later
            # it will be empty dictionary for single cav application
            platoon_base = OmegaConf.create({'platoon': self.scenario_params.get('platoon_base',{})})
            cav_config = OmegaConf.merge(self.scenario_params['vehicle_base'],
                                         platoon_base,
                                         cav_config)
            # if the spawn position is a single scalar, we need to use map
            # helper to transfer to spawn transform
            if 'spawn_special' not in cav_config:
                spawn_transform = carla.Transform(
                    carla.Location(
                        x=cav_pos[0],
                        y=cav_pos[1],
                        z=cav_pos[2]),
                    carla.Rotation(
                        pitch=cav_pos[5],
                        yaw=cav_pos[4],
                        roll=cav_pos[3]))
            else:
                spawn_transform = map_helper(self.carla_version, *cav_config['spawn_special'])

            cav_vehicle_bp.set_attribute('color', '0, 0, 255')
            
            vehicle = self.world.try_spawn_actor(cav_vehicle_bp, spawn_transform)
            while not vehicle:
                spawn_transform.location.z += 0.01
                vehicle = self.world.try_spawn_actor(cav_vehicle_bp, spawn_transform)
            
            # create vehicle manager for each cav
            data_dump_path = self.scenario_params['output_dir'] if data_dump else None
            vehicle_manager = VehicleManagerReplay(
                vehicle, cav_config, application,
                self.carla_map, self.cav_world,
                current_time=self.scenario_params['current_time'],
                data_dumping=data_dump_path)

            self.world.tick()

            single_cav_dict[cav_id] = {'actor': vehicle_manager, 'time': cur_timestamp}

        return single_cav_dict
        
    def create_rsu_manager(self, data_dump):
        """
        Create a list of RSU.

        Parameters
        ----------
        data_dump : bool
            Whether to dump sensor data.

        Returns
        -------
        rsu_list : list
            A list contains all rsu managers..
        """
        print('Creating RSU.')
        rsu_list = []
        
        data_dump_path = self.scenario_params['output_dir'] if data_dump else None
        for i, rsu_config in enumerate(self.scenario_params['scenario']['rsu_list'][:-1]):
            rsu_config = OmegaConf.merge(self.scenario_params['rsu_base'],
                                         rsu_config)
            rsu_manager = RSUManagerReplay(self.world, rsu_config,
                                     self.carla_map,
                                     self.cav_world,
                                     self.scenario_params['current_time'],
                                     data_dumping=data_dump_path)

            rsu_list.append(rsu_manager)

        return rsu_list

    def spawn_vehicles_by_dict(self, traffic_config, bg_veh_pos, cur_timestamp):

        blueprint_library = self.world.get_blueprint_library()
        if not self.use_multi_class_bp:
            ego_vehicle_random_list = car_blueprint_filter(blueprint_library,
                                                           self.carla_version)
        else:
            label_list = list(self.bp_class_sample_prob.keys())
            prob = [self.bp_class_sample_prob[itm] for itm in label_list]

        # if not random select, we always choose lincoln.mkz with green color
        color = '0, 255, 0'
        default_model = 'vehicle.lincoln.mkz2017' \
            if self.carla_version == '0.9.11' else 'vehicle.lincoln.mkz_2017'
        ego_vehicle_bp = blueprint_library.find(default_model)

        bg_dict = {}
        for i, (veh_id, veh_pos) in enumerate(bg_veh_pos.items()):
            spawn_transform = carla.Transform(
                carla.Location(
                    x=veh_pos[0],
                    y=veh_pos[1],
                    z=veh_pos[2]),
                carla.Rotation(
                    pitch=veh_pos[5],
                    yaw=veh_pos[4],
                    roll=veh_pos[3]))

            if not traffic_config['random']:
                ego_vehicle_bp.set_attribute('color', color)

            else:
                # sample a bp from various classes
                if self.use_multi_class_bp:
                    label = np.random.choice(label_list, p=prob)
                    # Given the label (class), find all associated blueprints in CARLA
                    ego_vehicle_random_list = multi_class_vehicle_blueprint_filter(
                        label, blueprint_library, self.bp_meta)
                ego_vehicle_bp = random.choice(ego_vehicle_random_list)

                if ego_vehicle_bp.has_attribute("color"):
                    color = random.choice(
                        ego_vehicle_bp.get_attribute(
                            'color').recommended_values)
                    ego_vehicle_bp.set_attribute('color', color)

            vehicle = self.world.try_spawn_actor(ego_vehicle_bp, spawn_transform)

            while not vehicle:
                spawn_transform.location.z += 0.01
                vehicle = self.world.try_spawn_actor(ego_vehicle_bp, spawn_transform)
            
            bg_dict[veh_id] = {'actor': vehicle, 'time': cur_timestamp}

        return bg_dict

    def spawn_vehicles_by_dict_bp(self, traffic_config, bg_veh_pos, cur_timestamp, bg_veh_bp):

        blueprint_library = self.world.get_blueprint_library()
        bg_dict = {}

        for i, (veh_id, veh_pos) in enumerate(bg_veh_pos.items()):

            bp = bg_veh_bp[veh_id]
            ego_vehicle_bp = blueprint_library.find(bp['bp_id'])
            ego_vehicle_bp.set_attribute('color', bp['color'])

            spawn_transform = carla.Transform(
                carla.Location(
                    x=veh_pos[0],
                    y=veh_pos[1],
                    z=veh_pos[2]),
                carla.Rotation(
                    pitch=veh_pos[5],
                    yaw=veh_pos[4],
                    roll=veh_pos[3]))

            vehicle = self.world.try_spawn_actor(ego_vehicle_bp, spawn_transform)

            while not vehicle:
                spawn_transform.location.z += 0.01
                vehicle = self.world.try_spawn_actor(ego_vehicle_bp, spawn_transform)
            
            bg_dict[veh_id] = {'actor': vehicle, 'time': cur_timestamp}

        return bg_dict

    def move_vehicle(self, cav_content, cur_timestamp, transform):
        """
        Move the

        Parameters
        ----------
        veh_id : str
            Vehicle's id

        cur_timestamp : str
            Current timestamp

        transform : carla.Transform
            Current pose/
        """
        # this represent this vehicle is already moved in this round
        if cav_content['time'] == cur_timestamp:
            return

        veh = cav_content['actor']
        if isinstance(veh, VehicleManagerReplay):
            veh = veh.vehicle
        elif isinstance(veh, carla.libcarla.Vehicle):
            pass
        else:
            raise NotImplementedError(f'{type(veh)}')
        
        veh.set_transform(transform)
        cav_content['time'] = cur_timestamp

    def create_traffic_carla(self, bg_veh_pos, cur_timestamp, bg_veh_bp):

        print('Spawning CARLA traffic flow.')
        traffic_config = self.scenario_params['carla_traffic_manager']
        
        if bg_veh_bp is None:
            bg_dict = self.spawn_vehicles_by_dict(traffic_config, bg_veh_pos, cur_timestamp)
        else:
            bg_dict = self.spawn_vehicles_by_dict_bp(traffic_config, bg_veh_pos, cur_timestamp, bg_veh_bp)
        
        print('CARLA traffic flow generated.')
        return bg_dict

class ScenarioManagerReplayPly(ScenarioManagerReplay):

    def create_rsu_manager(self, data_dump):

        print('Creating RSU.')
        rsu_list = []
        
        data_dump_path = self.scenario_params['output_dir'] if data_dump else None
        for i, rsu_config in enumerate(self.scenario_params['scenario']['rsu_list'][:-1]):
            rsu_config = OmegaConf.merge(self.scenario_params['rsu_base'],
                                         rsu_config)
            rsu_manager = RSUManagerReplayPly(self.world, rsu_config,
                                     self.carla_map,
                                     self.cav_world,
                                     self.scenario_params['current_time'],
                                     data_dumping=data_dump_path)

            rsu_list.append(rsu_manager)

        return rsu_list