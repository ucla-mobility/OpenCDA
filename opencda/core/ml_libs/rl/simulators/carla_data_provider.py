import re
import math
import numpy as np
from six import iteritems
from typing import Any, Dict, List, Optional, Tuple

import carla
import shapely

from opencda.core.common.misc import cal_distance_angle
from opencda.core.ml_libs.rl.utils.simulator_utils.carla_utils \
    import calculate_speed, convert_waypoint_to_transform
from opencda.core.ml_libs.rl.utils.simulator_utils \
    import RoadOption


def _numpy(carla_vector, normalize=False):
    result = np.float32([carla_vector.x, carla_vector.y])

    if normalize:
        return result / (np.linalg.norm(result) + 1e-4)

    return result


def _orientation(yaw):
    return np.float32([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])


class CarlaDataProvider(object):
    """
    This class provides access to various data for all registered actors
    In addition it provides access to the map and the transform of all traffic lights
    It buffers the data and updates it on every CARLA tick
    It aims to get rid of frequently updating data from Carla server

    """

    _client = None
    _world = None
    _map = None
    _sync_mode = True
    _actor_speed_map = dict()
    _actor_transform_map = dict()
    _actor_acceleration_map = dict()
    _actor_angular_velocity_map = dict()
    _actor_speed_vector_map = dict()
    _traffic_light_map = dict()
    _carla_actor_pool = dict()
    _hero_vehicle_route = None
    _target_waypoint = None
    _spawn_points = None
    _available_points = set()
    _blueprint_library = None
    _rng = np.random.RandomState(2000)

    @staticmethod
    def set_random_seed(seed: int) -> None:
        CarlaDataProvider._rng = np.random.RandomState(seed)

    @staticmethod
    def register_actor(actor: carla.Actor) -> None:
        """
        Add new actor to dictionaries
        If actor already exists, throw an exception
        """
        if actor in CarlaDataProvider._carla_actor_pool:
            raise KeyError("Vehicle '{}' already registered. Cannot register twice!".format(actor.id))
        else:
            CarlaDataProvider._carla_actor_pool[actor.id] = actor

        if actor in CarlaDataProvider._actor_speed_map:
            raise KeyError("Vehicle '{}' already registered. Cannot register twice!".format(actor.id))
        else:
            CarlaDataProvider._actor_speed_map[actor] = 0

        if actor in CarlaDataProvider._actor_transform_map:
            raise KeyError("Vehicle '{}' already registered. Cannot register twice!".format(actor.id))
        else:
            CarlaDataProvider._actor_transform_map[actor] = actor.get_transform()

        if actor in CarlaDataProvider._actor_acceleration_map:
            raise KeyError("Vehicle '{}' already registered. Cannot register twice!".format(actor.id))
        else:
            CarlaDataProvider._actor_acceleration_map[actor] = None

        if actor in CarlaDataProvider._actor_angular_velocity_map:
            raise KeyError("Vehicle '{}' already registered. Cannot register twice!".format(actor.id))
        else:
            CarlaDataProvider._actor_angular_velocity_map[actor] = None

        if actor in CarlaDataProvider._actor_speed_vector_map:
            raise KeyError("Vehicle '{}' already registered. Cannot register twice!".format(actor.id))
        else:
            CarlaDataProvider._actor_speed_vector_map[actor] = None

    @staticmethod
    def register_actors(actors: List) -> None:
        """
        Add new set of actors to dictionaries
        """
        for actor in actors:
            CarlaDataProvider.register_actor(actor)

    @staticmethod
    def on_carla_tick() -> None:
        """
        Callback from CARLA
        """
        for actor in CarlaDataProvider._actor_speed_map:
            if actor is not None and actor.is_alive:
                CarlaDataProvider._actor_speed_map[actor] = calculate_speed(actor)

        for actor in CarlaDataProvider._actor_transform_map:
            if actor is not None and actor.is_alive:
                CarlaDataProvider._actor_transform_map[actor] = actor.get_transform()

        for actor in CarlaDataProvider._actor_acceleration_map:
            if actor is not None and actor.is_alive:
                CarlaDataProvider._actor_acceleration_map[actor] = actor.get_acceleration()

        for actor in CarlaDataProvider._actor_angular_velocity_map:
            if actor is not None and actor.is_alive:
                CarlaDataProvider._actor_angular_velocity_map[actor] = actor.get_angular_velocity()

        for actor in CarlaDataProvider._actor_speed_vector_map:
            if actor is not None and actor.is_alive:
                CarlaDataProvider._actor_speed_vector_map[actor] = actor.get_velocity()

        world = CarlaDataProvider._world
        if world is None:
            print("WARNING: CarlaDataProvider couldn't find the world")

    @staticmethod
    def get_speed(actor: carla.Actor) -> float:
        """
        returns the absolute speed for the given actor
        """
        for key in CarlaDataProvider._actor_speed_map:
            if key.id == actor.id:
                return CarlaDataProvider._actor_speed_map[key]
        print('WARNING: {}.get_speed: {} not found!'.format(__name__, actor))
        return -1

    @staticmethod
    def get_velocity(actor: carla.Actor) -> float:
        """
        returns the absolute velocity for the given actor
        """
        return CarlaDataProvider.get_speed(actor)

    @staticmethod
    def get_transform(actor: carla.Actor) -> Optional[carla.Transform]:
        """
        returns the transform for the given actor
        """
        for key in CarlaDataProvider._actor_transform_map:
            if key.id == actor.id:
                return CarlaDataProvider._actor_transform_map[key]
        print('WARNING: {}.get_transform: {} not found!'.format(__name__, actor))
        return None

    @staticmethod
    def get_location(actor: carla.Actor) -> Optional[carla.Vector3D]:
        """
        returns the location for the given actor
        """
        for key in CarlaDataProvider._actor_transform_map:
            if key.id == actor.id:
                return CarlaDataProvider._actor_transform_map[key].location
        print('WARNING: {}.get_location: {} not found!'.format(__name__, actor))
        return None

    @staticmethod
    def get_acceleration(actor: carla.Actor) -> Optional[carla.Vector3D]:
        """
        returns the absolute acceleration for the given actor
        """
        for key in CarlaDataProvider._actor_acceleration_map:
            if key.id == actor.id:
                return CarlaDataProvider._actor_acceleration_map[key]
        print('WARNING: {}.get_acceleration: {} not found!'.format(__name__, actor))
        return None

    @staticmethod
    def get_angular_velocity(actor: carla.Actor) -> Optional[carla.Vector3D]:
        """
        returns the angular velocity for the given actor
        """
        for key in CarlaDataProvider._actor_angular_velocity_map:
            if key.id == actor.id:
                return CarlaDataProvider._actor_angular_velocity_map[key]
        print('WARNING: {}.get_angular_velocity: {} not found!'.format(__name__, actor))
        return None

    @staticmethod
    def get_speed_vector(actor: carla.Actor) -> Optional[carla.Vector3D]:
        """
        returns the absolute speed for the given actor
        """
        for key in CarlaDataProvider._actor_speed_vector_map:
            if key.id == actor.id:
                return CarlaDataProvider._actor_speed_vector_map[key]
        print('WARNING: {}.get_speed: {} not found!'.format(__name__, actor))
        return None

    @staticmethod
    def set_client(client: carla.Client) -> None:
        """
        Set the CARLA client
        """
        CarlaDataProvider._client = client

    @staticmethod
    def get_client() -> None:
        """
        Get the CARLA client
        """
        return CarlaDataProvider._client

    @staticmethod
    def set_world(world: carla.World) -> None:
        """
        Set the world and world settings
        """
        CarlaDataProvider._world = world
        CarlaDataProvider._sync_mode = world.get_settings().synchronous_mode
        CarlaDataProvider._map = world.get_map()
        CarlaDataProvider._blueprint_library = world.get_blueprint_library()
        CarlaDataProvider._generate_spawn_points()
        CarlaDataProvider.prepare_map()

    @staticmethod
    def get_world() -> Any:
        """
        Return world
        """
        return CarlaDataProvider._world

    @staticmethod
    def get_traffic_manager_port() -> int:
        """
        Get the port of the traffic manager.
        """
        return CarlaDataProvider._traffic_manager_port

    @staticmethod
    def set_traffic_manager_port(tm_port: int) -> None:
        """
        Set the port to use for the traffic manager.
        """
        CarlaDataProvider._traffic_manager_port = tm_port

    @staticmethod
    def get_map(world: Optional[carla.World] = None) -> Any:
        """
        Get the current map
        """
        if CarlaDataProvider._map is None:
            if world is None:
                if CarlaDataProvider._world is None:
                    raise ValueError("class member 'world' not initialized yet")
                else:
                    CarlaDataProvider._map = CarlaDataProvider._world.get_map()
            else:
                CarlaDataProvider._map = world.get_map()
        return CarlaDataProvider._map

    @staticmethod
    def find_weather_presets() -> List:
        """
        Get weather presets from CARLA
        """
        rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
        name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
        presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
        return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

    @staticmethod
    def is_sync_mode() -> bool:
        """
        Return true if synchronous mode is used
        """
        return CarlaDataProvider._sync_mode

    @staticmethod
    def prepare_map() -> None:
        """
        This function set the current map and loads all traffic lights for this map to
        _traffic_light_map
        """
        if CarlaDataProvider._map is None:
            CarlaDataProvider._map = CarlaDataProvider._world.get_map()

        # Parse all traffic lights
        CarlaDataProvider._traffic_light_map.clear()
        for traffic_light in CarlaDataProvider._world.get_actors().filter('*traffic_light*'):
            if traffic_light not in CarlaDataProvider._traffic_light_map.keys():
                CarlaDataProvider._traffic_light_map[traffic_light] = traffic_light.get_transform()
            else:
                raise KeyError("Traffic light '{}' already registered. Cannot register twice!".format(traffic_light.id))

    @staticmethod
    def annotate_trafficlight_in_group(traffic_light: carla.Actor) -> Dict:
        """
        Get dictionary with traffic light group info for a given traffic light
        """
        dict_annotations = {'ref': [], 'opposite': [], 'left': [], 'right': []}

        # Get the waypoints
        ref_location = CarlaDataProvider.get_trafficlight_trigger_location(traffic_light)
        ref_waypoint = CarlaDataProvider.get_map().get_waypoint(ref_location)
        ref_yaw = ref_waypoint.transform.rotation.yaw

        group_tl = traffic_light.get_group_traffic_lights()

        for target_tl in group_tl:
            if traffic_light.id == target_tl.id:
                dict_annotations['ref'].append(target_tl)
            else:
                # Get the angle between yaws
                target_location = CarlaDataProvider.get_trafficlight_trigger_location(target_tl)
                target_waypoint = CarlaDataProvider.get_map().get_waypoint(target_location)
                target_yaw = target_waypoint.transform.rotation.yaw

                diff = (target_yaw - ref_yaw) % 360

                if diff > 330:
                    continue
                elif diff > 225:
                    dict_annotations['right'].append(target_tl)
                elif diff > 135.0:
                    dict_annotations['opposite'].append(target_tl)
                elif diff > 30:
                    dict_annotations['left'].append(target_tl)

        return dict_annotations

    @staticmethod
    def get_trafficlight_trigger_location(traffic_light: carla.Actor) -> carla.Vector3D:  # pylint: disable=invalid-name
        """
        Calculates the yaw of the waypoint that represents the trigger volume of the traffic light
        """

        def rotate_point(point, angle):
            """
            rotate a given point by a given angle
            """
            x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
            y_ = math.sin(math.radians(angle)) * point.x - math.cos(math.radians(angle)) * point.y

            return carla.Vector3D(x_, y_, point.z)

        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(traffic_light.trigger_volume.location)
        area_ext = traffic_light.trigger_volume.extent

        point = rotate_point(carla.Vector3D(0, 0, area_ext.z), base_rot)
        point_location = area_loc + carla.Location(x=point.x, y=point.y)

        return carla.Location(point_location.x, point_location.y, point_location.z)

    @staticmethod
    def update_light_states(
            ego_light: carla.Actor,
            annotations: Dict,
            states: Dict,
            freeze: bool = False,
            timeout: int = 1000000000
    ) -> List:
        """
        Update traffic light states
        """
        reset_params = []

        for state in states:
            relevant_lights = []
            if state == 'ego':
                relevant_lights = [ego_light]
            else:
                relevant_lights = annotations[state]
            for light in relevant_lights:
                prev_state = light.get_state()
                prev_green_time = light.get_green_time()
                prev_red_time = light.get_red_time()
                prev_yellow_time = light.get_yellow_time()
                reset_params.append(
                    {
                        'light': light,
                        'state': prev_state,
                        'green_time': prev_green_time,
                        'red_time': prev_red_time,
                        'yellow_time': prev_yellow_time
                    }
                )

                light.set_state(states[state])
                if freeze:
                    light.set_green_time(timeout)
                    light.set_red_time(timeout)
                    light.set_yellow_time(timeout)

        return reset_params

    @staticmethod
    def reset_lights(reset_params: List) -> None:
        """
        Reset traffic lights
        """
        for param in reset_params:
            param['light'].set_state(param['state'])
            param['light'].set_green_time(param['green_time'])
            param['light'].set_red_time(param['red_time'])
            param['light'].set_yellow_time(param['yellow_time'])

    @staticmethod
    def get_next_traffic_light(actor: carla.Actor, use_cached_location: bool = True) -> carla.Actor:
        """
        returns the next relevant traffic light for the provided actor
        """

        if not use_cached_location:
            location = actor.get_transform().location
        else:
            location = CarlaDataProvider.get_location(actor)

        waypoint = CarlaDataProvider.get_map().get_waypoint(location)
        # Create list of all waypoints until next intersection
        list_of_waypoints = []
        while waypoint and not waypoint.is_intersection:
            list_of_waypoints.append(waypoint)
            waypoint = waypoint.next(2.0)[0]

        # If the list is empty, the actor is in an intersection
        if not list_of_waypoints:
            return None

        relevant_traffic_light = None
        distance_to_relevant_traffic_light = float("inf")

        for traffic_light in CarlaDataProvider._traffic_light_map:
            if hasattr(traffic_light, 'trigger_volume'):
                tl_t = CarlaDataProvider._traffic_light_map[traffic_light]
                transformed_tv = tl_t.transform(traffic_light.trigger_volume.location)
                distance = carla.Location(transformed_tv).distance(list_of_waypoints[-1].transform.location)

                if distance < distance_to_relevant_traffic_light:
                    relevant_traffic_light = traffic_light
                    distance_to_relevant_traffic_light = distance

        return relevant_traffic_light

    @staticmethod
    def is_light_red(vehicle: carla.Actor,
                     proximity_tlight_threshold: float = 10.0) -> Tuple[bool, Optional[carla.Actor]]:
        """
        Method to check if there is a red light affecting us. This version of
        the method is compatible with both European and US style traffic lights.

        :Arguments:
            - proximity_tlight_threshold: threshold to judge if light affecting
        :Returns: a tuple given by (bool_flag, traffic_light), where
            - bool_flag: True if there is a traffic light in RED affecting us and False otherwise
            - traffic_light: The object itself or None if there is no red traffic light affecting us
        """
        lights_list = CarlaDataProvider.get_actor_list().filter("*traffic_light*")

        vehicle_location = vehicle.get_location()
        vehicle_waypoint = CarlaDataProvider._map.get_waypoint(vehicle_location)

        for traffic_light in lights_list:
            object_location = CarlaDataProvider.get_trafficlight_trigger_location(traffic_light)
            object_waypoint = CarlaDataProvider._map.get_waypoint(object_location)

            if object_waypoint.road_id != vehicle_waypoint.road_id:
                continue

            ve_dir = vehicle_waypoint.transform.get_forward_vector()
            wp_dir = object_waypoint.transform.get_forward_vector()
            dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

            if dot_ve_wp < 0:
                continue

            while not object_waypoint.is_intersection:
                next_waypoint = object_waypoint.next(0.5)[0]
                if next_waypoint and not next_waypoint.is_intersection:
                    object_waypoint = next_waypoint
                else:
                    break

            # find norm and angle
            norm_target, d_angle = cal_distance_angle(object_waypoint.transform.location,
                                             vehicle.get_transform().location,
                                             vehicle.get_transform().rotation.yaw)
            # determine if obstacle is in front
            is_within_distance_ahead_host = (norm_target < 0.001 or d_angle < 90) \
                                            if norm_target <= proximity_tlight_threshold else False

            if is_within_distance_ahead_host:
                if traffic_light.state == carla.TrafficLightState.Red:
                    return (True, traffic_light)

        return (False, None)

    @staticmethod
    def get_next_traffic_light_from_waypoint(waypoint: carla.Waypoint) -> carla.Actor:
        # Create list of all waypoints until next intersection
        list_of_waypoints = []
        while waypoint and not waypoint.is_intersection:
            list_of_waypoints.append(waypoint)
            waypoint = waypoint.next(2.0)[0]

        # If the list is empty, the actor is in an intersection
        if not list_of_waypoints:
            return None

        relevant_traffic_light = None
        distance_to_relevant_traffic_light = float("inf")

        for traffic_light in CarlaDataProvider._traffic_light_map:
            if hasattr(traffic_light, 'trigger_volume'):
                tl_t = CarlaDataProvider._traffic_light_map[traffic_light]
                transformed_tv = tl_t.transform(traffic_light.trigger_volume.location)
                distance = carla.Location(transformed_tv).distance(list_of_waypoints[-1].transform.location)

                if distance < distance_to_relevant_traffic_light:
                    relevant_traffic_light = traffic_light
                    distance_to_relevant_traffic_light = distance

        return relevant_traffic_light

    @staticmethod
    def set_hero_vehicle_route(route: List) -> None:
        """
        Set the route of the ego vehicle
        """
        CarlaDataProvider._hero_vehicle_route = route

    @staticmethod
    def get_hero_vehicle_route() -> List:
        """
        returns the currently set route of the ego vehicle
        Note: Can be None
        """
        return CarlaDataProvider._hero_vehicle_route

    @staticmethod
    def get_ego_vehicle_route() -> List:
        """
        returns the currently set route of the ego vehicle
        Note: waypoints are convert to transforms
        """
        if CarlaDataProvider._hero_vehicle_route is not None:
            return convert_waypoint_to_transform(CarlaDataProvider._hero_vehicle_route)
        return None

    @staticmethod
    def _generate_spawn_points():
        """
        Generate spawn points for the current map
        """
        CarlaDataProvider._available_points.clear()
        spawn_points = list(CarlaDataProvider._map.get_spawn_points())
        for i in range(len(spawn_points)):
            CarlaDataProvider._available_points.add(i)
        CarlaDataProvider._spawn_points = spawn_points

    @staticmethod
    def get_spawn_point(index: int) -> carla.Vector3D:
        """
        Get spawn point from index
        """
        return CarlaDataProvider._spawn_points[index]

    @staticmethod
    def create_blueprint(
            model: str,
            rolename: str = 'npc',
            color: Any = None,
            actor_category: str = "car",
            disable_two_wheels: bool = False
    ) -> Any:
        """
        Function to setup the blueprint of an actor given its model and other relevant parameters
        """

        _actor_blueprint_categories = {
            'car': 'vehicle.tesla.model3',
            'van': 'vehicle.volkswagen.t2',
            'truck': 'vehicle.carlamotors.carlacola',
            'trailer': '',
            'semitrailer': '',
            'bus': 'vehicle.volkswagen.t2',
            'motorbike': 'vehicle.kawasaki.ninja',
            'bicycle': 'vehicle.diamondback.century',
            'train': '',
            'tram': '',
            'pedestrian': 'walker.pedestrian.0001',
        }

        # Set the model
        try:
            blueprints = CarlaDataProvider._blueprint_library.filter(model)
            if disable_two_wheels:
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprint = np.random.choice(blueprints)
        except ValueError:
            # The model is not part of the blueprint library. Let's take a default one for the given category
            bp_filter = "vehicle.*"
            new_model = _actor_blueprint_categories[actor_category]
            if new_model != '':
                bp_filter = new_model
            print("WARNING: Actor model {} not available. Using instead {}".format(model, new_model))
            blueprint = np.random.choice(CarlaDataProvider._blueprint_library.filter(bp_filter))

        # Set the color
        if color:
            if not blueprint.has_attribute('color'):
                print(
                    "WARNING: Cannot set Color ({}) for actor {} due to missing blueprint attribute".format(
                        color, blueprint.id
                    )
                )
            else:
                default_color_rgba = blueprint.get_attribute('color').as_color()
                default_color = '({}, {}, {})'.format(default_color_rgba.r, default_color_rgba.g, default_color_rgba.b)
                try:
                    blueprint.set_attribute('color', color)
                except ValueError:
                    # Color can't be set for this vehicle
                    print(
                        "WARNING: Color ({}) cannot be set for actor {}. Using instead: ({})".format(
                            color, blueprint.id, default_color
                        )
                    )
                    blueprint.set_attribute('color', default_color)
        else:
            if blueprint.has_attribute('color') and rolename != 'hero':
                color = np.random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)

        # Make pedestrians mortal
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'false')

        # Set the rolename
        if blueprint.has_attribute('role_name'):
            blueprint.set_attribute('role_name', rolename)

        return blueprint

    @staticmethod
    def request_new_actor(
            model: str,
            spawn_point: Any,
            rolename: str = 'actor',
            autopilot: bool = False,
            random_location: bool = False,
            color: Any = None,
            actor_category: str = "car",
            disable_two_wheels: bool = False,
    ) -> carla.Actor:
        """
        This method tries to create a new actor, returning it if successful (None otherwise).
        """
        blueprint = CarlaDataProvider.create_blueprint(model, rolename, color, actor_category, disable_two_wheels)

        if random_location:
            actor = None
            spawn_index = -1
            while not actor:
                spawn_index = CarlaDataProvider._rng.choice(list(CarlaDataProvider._available_points))
                spawn_point = CarlaDataProvider._spawn_points[spawn_index]
                actor = CarlaDataProvider._world.try_spawn_actor(blueprint, spawn_point)
            CarlaDataProvider._available_points.remove(spawn_index)

        else:
            # slightly lift the actor to avoid collisions with ground when spawning the actor
            # DO NOT USE spawn_point directly, as this will modify spawn_point permanently
            _spawn_point = carla.Transform(carla.Location(), spawn_point.rotation)
            _spawn_point.location.x = spawn_point.location.x
            _spawn_point.location.y = spawn_point.location.y
            _spawn_point.location.z = spawn_point.location.z + 0.2
            actor = CarlaDataProvider._world.try_spawn_actor(blueprint, _spawn_point)

        if actor is None:
            raise RuntimeError("Error: Unable to spawn vehicle {} at {}".format(blueprint.id, spawn_point))

        # wait for the actor to be spawned properly before we do anything
        if CarlaDataProvider.is_sync_mode():
            CarlaDataProvider._world.tick()
        else:
            CarlaDataProvider._world.wait_for_tick()

        if actor is None:
            return None
        else:
            # Let's deactivate the autopilot of the actor if it belongs to vehicle
            if actor in CarlaDataProvider._blueprint_library.filter('vehicle.*'):
                actor.set_autopilot(autopilot)

        CarlaDataProvider._carla_actor_pool[actor.id] = actor
        CarlaDataProvider.register_actor(actor)

        return actor

    @staticmethod
    def request_new_batch_actors(
            model: str,
            amount: int,
            spawn_points: List,
            autopilot: bool = False,
            random_location: bool = False,
            rolename: str = 'npc',
            disable_two_wheels: bool = False
    ) -> List:
        """
        Simplified version of "request_new_actors". This method also create several actors in batch.

        Instead of needing a list of ActorConfigurationData, an "amount" parameter is used.
        This makes actor spawning easier but reduces the amount of configurability.

        Some parameters are the same for all actors (rolename, autopilot and random location)
        while others are randomized (color)
        """

        SpawnActor = carla.command.SpawnActor  # pylint: disable=invalid-name
        SetAutopilot = carla.command.SetAutopilot  # pylint: disable=invalid-name
        FutureActor = carla.command.FutureActor  # pylint: disable=invalid-name

        #CarlaDataProvider._generate_spawn_points()

        batch = []

        for i in range(amount):
            # Get vehicle by model
            blueprint = CarlaDataProvider.create_blueprint(model, rolename, disable_two_wheels=disable_two_wheels)

            if random_location:
                if len(CarlaDataProvider._available_points) == 0:
                    print("WARNING: No more spawn points to use. Spawned {} actors out of {}".format(i + 1, amount))
                    break
                else:
                    spawn_index = CarlaDataProvider._rng.choice(list(CarlaDataProvider._available_points))
                    spawn_point = CarlaDataProvider._spawn_points[spawn_index]
                    CarlaDataProvider._available_points.remove(spawn_index)
            else:
                try:
                    spawn_point = spawn_points[i]
                except IndexError:
                    print("The amount of spawn points is lower than the amount of vehicles spawned")
                    break

            if spawn_point:
                batch.append(
                    SpawnActor(blueprint, spawn_point).then(
                        SetAutopilot(FutureActor, autopilot, CarlaDataProvider._traffic_manager_port)
                    )
                )

        actors = CarlaDataProvider.handle_actor_batch(batch)

        if actors is None:
            return None

        for actor in actors:
            if actor is None:
                continue
            CarlaDataProvider._carla_actor_pool[actor.id] = actor
            CarlaDataProvider.register_actor(actor)
        return actors

    @staticmethod
    def handle_actor_batch(batch: List) -> Optional[List]:
        """
        Forward a CARLA command batch to spawn actors to CARLA, and gather the responses.
        Returns list of actors on success, none otherwise
        """

        actors = []

        sync_mode = CarlaDataProvider.is_sync_mode()

        if CarlaDataProvider._client and batch is not None:
            responses = CarlaDataProvider._client.apply_batch_sync(batch, sync_mode)
        else:
            return None

        # wait for the actors to be spawned properly before we do anything
        if sync_mode:
            CarlaDataProvider._world.tick()
        else:
            CarlaDataProvider._world.wait_for_tick()

        actor_ids = []
        if responses:
            for response in responses:
                if not response.error:
                    actor_ids.append(response.actor_id)

        carla_actors = CarlaDataProvider._world.get_actors(actor_ids)
        for actor in carla_actors:
            actors.append(actor)

        return actors

    @staticmethod
    def is_vehicle_hazard(vehicle: carla.Actor,
                          proximity_vehicle_threshold: float = 10.0) -> Tuple[bool, Optional[carla.Actor]]:
        """
        :Arguments:
            - vehicle: Potential obstacle to check
            - proximity_vehicle_threshold: Threshold to judge hazard
        :Returns: a tuple given by (bool_flag, vehicle), where
            - bool_flag: True if there is a vehicle ahead blocking us and False otherwise
            - vehicle: The blocker object itself
        """
        vehicle_list = CarlaDataProvider.get_actor_list().filter("*vehicle*")
        vehicle_location = CarlaDataProvider.get_location(vehicle)
        vehicle_waypoint = CarlaDataProvider._map.get_waypoint(vehicle_location)

        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == vehicle.id or not target_vehicle.is_alive:
                continue

            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint = CarlaDataProvider._map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != vehicle_waypoint.road_id or \
                    target_vehicle_waypoint.lane_id != vehicle_waypoint.lane_id:
                continue

            # find norm and angle
            norm_target, d_angle = cal_distance_angle(object_waypoint.transform.location,
                                                      vehicle.get_transform().location,
                                                      vehicle.get_transform().rotation.yaw)
            # determine if obstacle is in front
            is_within_distance_ahead_host = (norm_target < 0.001 or d_angle < 90) \
                if norm_target <= proximity_tlight_threshold else False

            if is_within_distance_ahead_host:
                return (True, target_vehicle)

        return (False, None)

    @staticmethod
    def is_junction_vehicle_hazard(vehicle: carla.Actor, command: RoadOption) -> Tuple[bool, Optional[carla.Actor]]:
        """
        :Arguments:
            - vehicle: Potential obstacle to check
            - command: future driving command
        :Returns: a tuple given by (bool_flag, vehicle), where
            - bool_flag: True if there is a vehicle ahead blocking us in junction and False otherwise
            - vehicle: The blocker object itself
        """
        vehicle_list = CarlaDataProvider.get_actor_list().filter("*vehicle*")
        o1 = _orientation(CarlaDataProvider.get_transform(vehicle).rotation.yaw)
        x1 = vehicle.bounding_box.extent.x
        p1 = CarlaDataProvider.get_location(vehicle) + CarlaDataProvider.get_transform(vehicle).get_forward_vector()
        w1 = CarlaDataProvider._map.get_waypoint(p1)
        s1 = CarlaDataProvider.get_speed(vehicle)
        if command == RoadOption.RIGHT:
            shift_angle = 25
        elif command == RoadOption.LEFT:
            shift_angle = -25
        else:
            shift_angle = 0
        v1 = (5 * s1 + 6) * _orientation(CarlaDataProvider.get_transform(vehicle).rotation.yaw + shift_angle)

        for target_vehicle in vehicle_list:
            if target_vehicle.id == vehicle.id or not target_vehicle.is_alive:
                continue

            o2 = _orientation(CarlaDataProvider.get_transform(target_vehicle).rotation.yaw)
            o2_left = _orientation(CarlaDataProvider.get_transform(target_vehicle).rotation.yaw - 15)
            o2_right = _orientation(CarlaDataProvider.get_transform(target_vehicle).rotation.yaw + 15)
            x2 = target_vehicle.bounding_box.extent.x

            p2 = CarlaDataProvider.get_location(target_vehicle)
            p2_hat = p2 - (x2 + 2) * CarlaDataProvider.get_transform(target_vehicle).get_forward_vector()
            w2 = CarlaDataProvider._map.get_waypoint(p2)
            s2 = CarlaDataProvider.get_speed(target_vehicle)

            v2 = (4 * s2 + 2 * x2 + 7) * o2
            v2_left = (4 * s2 + 2 * x2 + 7) * o2_left
            v2_right = (4 * s2 + 2 * x2 + 7) * o2_right

            angle_between_heading = np.degrees(np.arccos(np.clip(o1.dot(o2), -1, 1)))

            if vehicle.get_location().distance(p2) > 25:
                continue
            if w1.is_junction is False and w2.is_junction is False:
                continue
            if (angle_between_heading < 15.0 or angle_between_heading > 165) and command == RoadOption.STRAIGHT:
                continue
            collides, collision_point = CarlaDataProvider.get_collision(_numpy(p1), v1, _numpy(p2_hat), v2)
            if collides is None:
                collides, collision_point = CarlaDataProvider.get_collision(_numpy(p1), v1, _numpy(p2_hat), v2_left)
            if collides is None:
                collides, collision_point = CarlaDataProvider.get_collision(_numpy(p1), v1, _numpy(p2_hat), v2_right)
                continue
            if collides:
                return (True, target_vehicle)
        return (False, None)

    @staticmethod
    def is_lane_vehicle_hazard(vehicle: carla.Actor, command: RoadOption) -> Tuple[bool, Optional[carla.Actor]]:
        """
        :Arguments:
            - vehicle: Potential obstacle to check
            - command: future driving command
        :Returns: a tuple given by (bool_flag, vehicle), where
            - bool_flag: True if there is a vehicle in other lanes blocking us and False otherwise
            - vehicle: The blocker object itself
        """
        vehicle_list = CarlaDataProvider.get_actor_list().filter("*vehicle*")
        if command != RoadOption.CHANGELANELEFT and command != RoadOption.CHANGELANERIGHT:
            return (False, None)
        w1 = CarlaDataProvider._map.get_waypoint(vehicle.get_location())
        o1 = _orientation(CarlaDataProvider.get_transform(vehicle).rotation.yaw)
        p1 = CarlaDataProvider.get_location(vehicle)

        yaw_w1 = w1.transform.rotation.yaw
        lane_width = w1.lane_width
        location_w1 = w1.transform.location

        lft_shift = 0.5
        rgt_shift = 0.5
        if command == RoadOption.CHANGELANELEFT:
            rgt_shift += 1
        else:
            lft_shift += 1

        lft_lane_wp = CarlaDataProvider.rotate_point(
            carla.Vector3D(lft_shift * lane_width, 0.0, location_w1.z), yaw_w1 + 90
        )
        lft_lane_wp = location_w1 + carla.Location(lft_lane_wp)
        rgt_lane_wp = CarlaDataProvider.rotate_point(
            carla.Vector3D(rgt_shift * lane_width, 0.0, location_w1.z), yaw_w1 - 90
        )
        rgt_lane_wp = location_w1 + carla.Location(rgt_lane_wp)

        for target_vehicle in vehicle_list:
            if target_vehicle.id == vehicle.id or not target_vehicle.is_alive:
                continue

            w2 = CarlaDataProvider._map.get_waypoint(CarlaDataProvider.get_location(target_vehicle))
            o2 = _orientation(CarlaDataProvider.get_transform(target_vehicle).rotation.yaw)
            p2 = CarlaDataProvider.get_location(target_vehicle)
            x2 = target_vehicle.bounding_box.extent.x
            p2_hat = p2 - CarlaDataProvider.get_transform(target_vehicle).get_forward_vector() * x2 * 2
            s2 = CarlaDataProvider.get_speed_vector(
                target_vehicle
            ) + CarlaDataProvider.get_transform(target_vehicle).get_forward_vector() * x2
            s2_value = max(12, 2 + 2 * x2 + 3.0 * CarlaDataProvider.get_speed(target_vehicle))

            distance = p1.distance(p2)

            if distance > s2_value:
                continue
            if w1.road_id != w2.road_id or w1.lane_id * w2.lane_id < 0:
                continue
            if command == RoadOption.CHANGELANELEFT:
                if w1.lane_id > 0:
                    if w2.lane_id != w1.lane_id - 1:
                        continue
                if w1.lane_id < 0:
                    if w2.lane_id != w1.lane_id + 1:
                        continue
            if command == RoadOption.CHANGELANERIGHT:
                if w1.lane_id > 0:
                    if w2.lane_id != w1.lane_id + 1:
                        continue
                if w1.lane_id < 0:
                    if w2.lane_id != w1.lane_id - 1:
                        continue

            if CarlaDataProvider.is_vehicle_crossing_future(p2_hat, s2, lft_lane_wp, rgt_lane_wp):
                return (True, target_vehicle)
        return (False, None)

    @staticmethod
    def is_bike_hazard(vehicle: carla.Actor) -> Tuple[bool, Optional[carla.Actor]]:
        """
        :Arguments:
            - vehicle: Potential obstacle to check
        :Returns: a tuple given by (bool_flag, vehicle), where
            - bool_flag: True if there is a bike ahead blocking us and False otherwise
            - bike: The blocker object itself
        """
        bikes_list = CarlaDataProvider.get_actor_list().filter("*vehicle*")
        o1 = _orientation(CarlaDataProvider.get_transform(vehicle).rotation.yaw)
        v1_hat = o1
        p1 = _numpy(CarlaDataProvider.get_location(vehicle))
        v1 = 10.0 * o1

        for bike in bikes_list:
            if 'driver_id' not in bike.attributes or not bike.is_alive:
                continue
            o2 = _orientation(CarlaDataProvider.get_transform(bike).rotation.yaw)
            s2 = CarlaDataProvider.get_speed(bike)
            v2_hat = o2
            p2 = _numpy(CarlaDataProvider.get_location(bike))

            p2_p1 = p2 - p1
            distance = np.linalg.norm(p2_p1)
            if distance > 20:
                continue
            p2_p1_hat = p2_p1 / (distance + 1e-4)

            angle_to_car = np.degrees(np.arccos(np.clip(v1_hat.dot(p2_p1_hat), -1, 1)))
            angle_between_heading = np.degrees(np.arccos(np.clip(o1.dot(o2), -1, 1)))

            # to consider -ve angles too
            angle_to_car = min(angle_to_car, 360.0 - angle_to_car)
            angle_between_heading = min(angle_between_heading, 360.0 - angle_between_heading)
            if angle_to_car > 30:
                continue
            if angle_between_heading < 75 and angle_between_heading > 105:
                continue

            p2_hat = -2.0 * v2_hat + p1
            v2 = 8.0 * v2_hat

            collides, collision_point = CarlaDataProvider.get_collision(p1, v1, p2_hat, v2)

            if collides:
                return (True, bike)

        return (False, None)

    @staticmethod
    def is_walker_hazard(vehicle: carla.Actor) -> Tuple[bool, Optional[carla.Actor]]:
        """
        :Arguments:
            - vehicle: Potential obstacle to check
        :Returns: a tuple given by (bool_flag, vehicle), where
            - bool_flag: True if there is a walker ahead blocking us and False otherwise
            - walker: The blocker object itself
        """
        walkers_list = CarlaDataProvider.get_actor_list().filter("*walker.*")
        p1 = _numpy(CarlaDataProvider.get_location(vehicle))
        v1 = 10.0 * _orientation(CarlaDataProvider.get_transform(vehicle).rotation.yaw)

        for walker in walkers_list:
            if not isinstance(walker, carla.Walker):
                continue
            v2_hat = _orientation(CarlaDataProvider.get_transform(walker).rotation.yaw)
            s2 = CarlaDataProvider.get_speed(walker)

            if s2 < 0.05:
                v2_hat *= s2

            p2 = -3.0 * v2_hat + _numpy(CarlaDataProvider.get_location(walker))
            v2 = 8.0 * v2_hat

            collides, collision_point = CarlaDataProvider.get_collision(p1, v1, p2, v2)

            if collides:
                return (True, walker)
        return (False, None)

    @staticmethod
    def is_vehicle_crossing_future(p1, s1, lft_lane, rgt_lane):
        p1_hat = carla.Location(x=p1.x + 3 * s1.x, y=p1.y + 3 * s1.y)
        line1 = shapely.geometry.LineString([(p1.x, p1.y), (p1_hat.x, p1_hat.y)])
        line2 = shapely.geometry.LineString([(lft_lane.x, lft_lane.y), (rgt_lane.x, rgt_lane.y)])
        inter = line1.intersection(line2)
        return not inter.is_empty

    @staticmethod
    def get_actors() -> List:
        """
        Return list of actors and their ids

        Note: iteritems from six is used to allow compatibility with Python 2 and 3
        """
        return iteritems(CarlaDataProvider._carla_actor_pool)

    @staticmethod
    def get_actor_list() -> List:
        """
        Return all actors in world
        """
        return CarlaDataProvider._world.get_actors()

    @staticmethod
    def actor_id_exists(actor_id: int) -> bool:
        """
        Check if a certain id is still at the simulation
        """
        if actor_id in CarlaDataProvider._carla_actor_pool:
            return True

        return False

    @staticmethod
    def get_hero_actor() -> Optional[carla.Actor]:
        """
        Get the actor object of the hero actor if it exists, returns none otherwise.
        """
        for actor_id in CarlaDataProvider._carla_actor_pool:
            if CarlaDataProvider._carla_actor_pool[actor_id].attributes['role_name'] == 'hero':
                return CarlaDataProvider._carla_actor_pool[actor_id]
        return None

    @staticmethod
    def get_actor_by_id(actor_id: int) -> Optional[carla.Actor]:
        """
        Get an actor from the pool by using its ID. If the actor
        does not exist, None is returned.
        """
        if actor_id in CarlaDataProvider._carla_actor_pool:
            return CarlaDataProvider._carla_actor_pool[actor_id]

        print("WARNING: Non-existing actor id {}".format(actor_id))
        return None

    @staticmethod
    def remove_actor_by_id(actor_id: int) -> None:
        """
        Remove an actor from the pool using its ID
        """
        if actor_id in CarlaDataProvider._carla_actor_pool:
            CarlaDataProvider._carla_actor_pool[actor_id].destroy()
            CarlaDataProvider._carla_actor_pool[actor_id] = None
            CarlaDataProvider._carla_actor_pool.pop(actor_id)
        else:
            print("WARNING: Trying to remove a non-existing actor id {}".format(actor_id))

    @staticmethod
    def clean_up() -> None:
        """
        Cleanup and remove all entries from all dictionaries
        """
        DestroyActor = carla.command.DestroyActor  # pylint: disable=invalid-name
        batch = []

        for actor_id in CarlaDataProvider._carla_actor_pool:
            actor = CarlaDataProvider._carla_actor_pool[actor_id]
            if actor.is_alive:
                batch.append(DestroyActor(actor))

        if CarlaDataProvider._client:
            try:
                CarlaDataProvider._client.apply_batch_sync(batch)
            except RuntimeError as e:
                if "time-out" in str(e):
                    pass
                    print(e)
                else:
                    raise e

        CarlaDataProvider._actor_speed_map.clear()
        CarlaDataProvider._actor_transform_map.clear()
        CarlaDataProvider._actor_acceleration_map.clear()
        CarlaDataProvider._traffic_light_map.clear()
        CarlaDataProvider._hero_vehicle_route = None
        CarlaDataProvider._target_waypoint = None
        CarlaDataProvider._map = None
        CarlaDataProvider._world = None
        CarlaDataProvider._sync_mode = False
        CarlaDataProvider._carla_actor_pool.clear()
        CarlaDataProvider._client = None
        CarlaDataProvider._spawn_points = None
        CarlaDataProvider._available_points.clear()

    @staticmethod
    def remove_actors_in_surrounding(location, distance) -> None:
        """
        Remove all actors from the pool that are closer than distance to the
        provided location
        """
        for actor_id in CarlaDataProvider._carla_actor_pool.copy():
            if CarlaDataProvider._carla_actor_pool[actor_id].get_location().distance(location) < distance:
                CarlaDataProvider._carla_actor_pool[actor_id].destroy()
                CarlaDataProvider._carla_actor_pool.pop(actor_id)

        # Remove all keys with None values
        CarlaDataProvider._carla_actor_pool = dict({k: v for k, v in CarlaDataProvider._carla_actor_pool.items() if v})

    @staticmethod
    def get_collision(p1, v1, p2, v2):
        A = np.stack([v1, -v2], 1)
        b = p2 - p1

        if abs(np.linalg.det(A)) < 1e-3:
            return False, None

        x = np.linalg.solve(A, b)
        collides = all(x >= 0) and all(x <= 1)  # how many seconds until collision

        return collides, p1 + x[0] * v1

    @staticmethod
    def rotate_point(point, angle):
        """
        rotate a given point by a given angle
        """
        x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
        y_ = math.sin(math.radians(angle)) * point.x + math.cos(math.radians(angle)) * point.y
        return carla.Vector3D(x_, y_, point.z)
