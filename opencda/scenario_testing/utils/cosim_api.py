# -*- coding: utf-8 -*-
"""
Co-simulation scenario manager. The code is modified from CARLA official
cosimulation code.
"""
# Author: CARLA Team, Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import logging
import os
import sys

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import carla

from opencda.co_simulation.sumo_integration.constants import SPAWN_OFFSET_Z
from opencda.co_simulation.sumo_integration.bridge_helper import BridgeHelper
from opencda.co_simulation.sumo_integration.constants import INVALID_ACTOR_ID
from opencda.co_simulation.sumo_integration.sumo_simulation import \
    SumoSimulation
from opencda.scenario_testing.utils.sim_api import ScenarioManager


class CoScenarioManager(ScenarioManager):
    """
    The Scenario manager for co-simulation(CARLA-SUMO). All sumo-related
    functions and variables will start with self.sumo.xx, and all carla-related
    functions and members won't have such prefixes.

    Parameters
    ----------
    scenario_params : dict
        The dictionary contains all simulation configurations.

    carla_version : str
        CARLA simulator version, it currently supports 0.9.11 and 0.9.12

    xodr_path : str
        The xodr file to the customized map, default: None.

    town : str
        Town name if not using customized map, eg. 'Town06'.

    apply_ml : bool
        Whether need to load dl/ml model(pytorch required) in this simulation.

    """

    def __init__(self, scenario_params, apply_ml, carla_version,
                 xodr_path=None,
                 town=None,
                 cav_world=None,
                 sumo_file_parent_path=None):
        # carla side initializations(partial init is already done in scenario
        # manager
        super(CoScenarioManager, self).__init__(scenario_params,
                                                apply_ml,
                                                carla_version,
                                                xodr_path,
                                                town,
                                                cav_world)

        # these following sets are used to track the vehicles controlled
        # by sumo side
        self._active_actors = set()
        self.spawned_actors = set()
        self.destroyed_actors = set()

        # contains all carla traffic lights objects
        self._tls = {}
        for landmark in self.carla_map.get_all_landmarks_of_type('1000001'):
            if landmark.id != '':
                traffic_ligth = self.world.get_traffic_light(landmark)
                if traffic_ligth is not None:
                    self._tls[landmark.id] = traffic_ligth
                else:
                    logging.warning('Landmark %s is not linked to any '
                                    'traffic light', landmark.id)

        # sumo side initialization
        base_name = \
            os.path.basename(sumo_file_parent_path)
        sumo_cfg = \
            os.path.join(sumo_file_parent_path, base_name + '.sumocfg')
        # todo: use yaml file to generate the route file
        assert os.path.isfile(sumo_cfg), '%s does not exist, make sure' \
                                         'your config file name has the' \
                                         'same basename as the directory' \
                                         'and use .sumocfg as extension' \
                                         % sumo_cfg

        sumo_port = scenario_params['sumo']['port']
        sumo_host = scenario_params['sumo']['host']
        sumo_gui = scenario_params['sumo']['gui']
        sumo_client_order = scenario_params['sumo']['client_order']
        # tick freq, the same as carla
        sumo_step_length = scenario_params['sumo']['step_length']

        self.sumo = SumoSimulation(sumo_cfg, sumo_step_length,
                                   sumo_host, sumo_port, sumo_gui,
                                   sumo_client_order)
        # the sumo traffic light should be synchronized with carla
        self.sumo.switch_off_traffic_lights()

        # Mapped actor ids. All vehicles controlled by sumo is
        # in sumo2carla_ids, all vehicles controlled by carla
        # is saved in carla2sumo_ids
        self.sumo2carla_ids = {}  # key: sumo id, value: carla id
        self.carla2sumo_ids = {}  # key: carla id, value: sumo id

        BridgeHelper.blueprint_library = self.world.get_blueprint_library()
        BridgeHelper.offset = self.sumo.get_net_offset()

    def tick(self):
        """
        Execute a single step of co-simulation. Logic: sumo will move the
        sumo vehicles to certain positions and then carla use set_transform to
        move the corresponding actors to the same location.

        Returns
        -------

        """
        # -----------------
        # sumo-->carla sync
        # -----------------
        self.sumo.tick()

        # Spawning new sumo actors in carla (i.e, not controlled by carla).
        sumo_spawned_actors = self.sumo.spawned_actors - set(
            self.carla2sumo_ids.values())

        for sumo_actor_id in sumo_spawned_actors:
            self.sumo.subscribe(sumo_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            # given the sumo vehicle type, return the corresponding
            # carla vehicle type. If there is no such correspondence,
            # carla will choose a random vehicle type.
            carla_blueprint = \
                BridgeHelper.get_carla_blueprint(sumo_actor, False)

            if carla_blueprint is not None:
                # return the sumo-controlled vehicle position under
                # Carla coordinate system. There is a translation between
                # the two.
                carla_transform = \
                    BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                     sumo_actor.extent)
                carla_actor_id = self.spawn_actor(carla_blueprint,
                                                  carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    self.sumo2carla_ids[sumo_actor_id] = carla_actor_id

            else:
                self.sumo.unsubscribe(sumo_actor_id)

        # Destroying sumo arrived actors in carla.
        for sumo_actor_id in self.sumo.destroyed_actors:
            if sumo_actor_id in self.sumo2carla_ids:
                self.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))

        # Updating sumo actors in carla.
        for sumo_actor_id in self.sumo2carla_ids:
            carla_actor_id = self.sumo2carla_ids[sumo_actor_id]

            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            carla_transform = \
                BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                 sumo_actor.extent)
            assert self.synchronize_vehicle(carla_actor_id,
                                            carla_transform)

        # -----------------
        # carla-->sumo sync
        # -----------------
        self.world.tick()

        # Update data structures for the current frame.
        current_actors = set(
            [vehicle.id for vehicle in
             self.world.get_actors().filter('vehicle.*')])
        self.spawned_actors = current_actors.difference(self._active_actors)
        self.destroyed_actors = self._active_actors.difference(current_actors)
        self._active_actors = current_actors

        # Spawning new carla actors (not controlled by sumo). For example,
        # the CAV we created on the carla side.
        carla_spawned_actors = self.spawned_actors - set(
            self.sumo2carla_ids.values())
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.world.get_actor(carla_actor_id)
            type_id = BridgeHelper.get_sumo_vtype(carla_actor)
            color = carla_actor.attributes.get('color',
                                               None)
            if type_id is not None:
                sumo_actor_id = self.sumo.spawn_actor(type_id, color)
                if sumo_actor_id != INVALID_ACTOR_ID:
                    self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                    self.sumo.subscribe(sumo_actor_id)

        # Destroying required carla actors in sumo.
        for carla_actor_id in self.destroyed_actors:
            if carla_actor_id in self.carla2sumo_ids:
                self.sumo.destroy_actor(
                    self.carla2sumo_ids.pop(carla_actor_id))

        # updating carla actors in sumo. For instance, update the new CAV
        # position in sumo
        for carla_actor_id in self.carla2sumo_ids:
            sumo_actor_id = self.carla2sumo_ids[carla_actor_id]

            carla_actor = self.world.get_actor(carla_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            sumo_transform = \
                BridgeHelper.get_sumo_transform(carla_actor.get_transform(),
                                            carla_actor.bounding_box.extent)
            self.sumo.synchronize_vehicle(sumo_actor_id, sumo_transform,
                                          None)

        # Updates traffic lights in sumo based on carla information.
        # todo make sure the tl is synced
        common_landmarks = self.sumo.traffic_light_ids & \
                           self.traffic_light_ids
        for landmark_id in common_landmarks:
            carla_tl_state = self.get_traffic_light_state(landmark_id)
            sumo_tl_state = BridgeHelper.get_sumo_traffic_light_state(
                carla_tl_state)

            # Updates all the sumo links related to this landmark.
            self.sumo.synchronize_traffic_light(landmark_id, sumo_tl_state)

        # update the sumo2carla dict to cav world
        self.cav_world.update_sumo_vehicles(self.sumo2carla_ids)

    @property
    def traffic_light_ids(self):
        return set(self._tls.keys())

    def get_traffic_light_state(self, landmark_id):
        """
        Accessor for traffic light state.

        If the traffic ligth does not exist, returns None.
        """
        if landmark_id not in self._tls:
            return None
        return self._tls[landmark_id].state

    def spawn_actor(self, blueprint, transform):
        """
        Spawns a new carla actor based on the given coordinate.

        Parameters
        ----------
        blueprint : carla.blueprint
            Blueprint of the actor to be spawned.
        transform : carla.transform
            Transform where the actor will be spawned.

        Returns
        -------
        actor_id : int
            The carla actor id the actor is successfully spawned. Otherwise,
            INVALID_ACTOR_ID will be return.
        """
        transform = carla.Transform(
            transform.location + carla.Location(0, 0, SPAWN_OFFSET_Z),
            transform.rotation)

        batch = [
            carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor,
                                                 False))
        ]
        response = self.client.apply_batch_sync(batch, False)[0]
        if response.error:
            logging.error('Spawn carla actor failed. %s', response.error)
            return INVALID_ACTOR_ID

        return response.actor_id

    def synchronize_vehicle(self, vehicle_id, transform):
        """
        The key function of co-simulation. Given the updated location in sumo,
        carla will move the corresponding vehicle to the same location.

        Parameters
        ----------
        vehicle_id : int
            The id of the carla actor to be updated.

        transform : carla.Transform
            The new vehicle transform.

        Returns
        -------
        success : bool
            Whether update is successful.
        """
        vehicle = self.world.get_actor(vehicle_id)
        if vehicle is None:
            return False

        vehicle.set_transform(transform)
        return True

    def destroy_actor(self, actor_id):
        """
        Destroys the given carla actor.

        Parameters
        ----------
        actor_id : int
            The actor id in carla.
        """
        actor = self.world.get_actor(actor_id)
        if actor is not None:
            return actor.destroy()
        return False

    def close(self):
        """
        Simulation close.
        """
        # restore to origin setting
        self.world.apply_settings(self.origin_settings)

        # Destroying synchronized actors.
        print('destroying carla actor')
        for carla_actor_id in self.sumo2carla_ids.values():
            self.destroy_actor(carla_actor_id)

        print('destroying sumo actor')
        for sumo_actor_id in self.carla2sumo_ids.values():
            self.sumo.destroy_actor(sumo_actor_id)

        # unfreeze traffic lights, since sumo may freeze the traffic light
        for actor in self.world.get_actors():
            if actor.type_id == 'traffic.traffic_light':
                actor.freeze(False)

        self.sumo.close()
