#!/usr/bin/env python

"""
Overtake Scenario:

This is demo module showing integration of OpenCDA with ScenarioRunner.

The scripts simulate a scenario where an ego vehicle has to overtake a background vehicle
that is ahead of the ego vehicle and at a lower speed. There are two fearless pedestrians
that suddenly appear in front of the ego vehicle and the ego vehicle has to avoid a collision
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario


class Overtake(BasicScenario):

    """
    The class spawns two background vehicles and two pedestrians in front of the ego vehicle.
    The ego vehicle is driving behind and overtaking the fast vehicle ahead

    self.other_actors[0] = fast car
    self.other_actors[1] = slow car
    self.other_actors[2] = pedestrian 1
    self.other_actors[3] = pedestrian 2
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
        Setup all relevant parameters and create scenario
        """
        print("Running Overtake Scenario")
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        self._fast_vehicle_velocity = 30
        self._slow_vehicle_velocity = 20
        self._pedestrian_velocity = 5
        self._trigger_distance = 35

        super(Overtake, self).__init__("Overtake",
                                       ego_vehicles,
                                       config,
                                       world,
                                       debug_mode,
                                       criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        # Spawn leading vehicles and the fearless pedestrian
        for actor_config in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(
                actor_config.model, actor_config.transform)
            self.other_actors.append(actor)
            actor.set_simulate_physics(enabled=False)

        # Transformation that renders the fast vehicle visible
        fast_car_transform = self.other_actors[0].get_transform()
        self.fast_car_visible = carla.Transform(
            carla.Location(fast_car_transform.location.x,
                           fast_car_transform.location.y,
                           fast_car_transform.location.z + 501),
            fast_car_transform.rotation)
        # Transformation that renders the slow vehicle visible
        slow_car_transform = self.other_actors[1].get_transform()
        self.slow_car_visible = carla.Transform(
            carla.Location(slow_car_transform.location.x,
                           slow_car_transform.location.y,
                           slow_car_transform.location.z + 501),
            slow_car_transform.rotation)

        # Transformation that renders the pedestrian visible
        pedestrian_transform_1 = self.other_actors[2].get_transform()
        self.pedestrian_visible_1 = carla.Transform(
            carla.Location(pedestrian_transform_1.location.x,
                           pedestrian_transform_1.location.y,
                           pedestrian_transform_1.location.z + 501),
            pedestrian_transform_1.rotation)
        pedestrian_transform_2 = self.other_actors[3].get_transform()
        self.pedestrian_visible_2 = carla.Transform(
            carla.Location(pedestrian_transform_2.location.x,
                           pedestrian_transform_2.location.y,
                           pedestrian_transform_2.location.z + 501),
            pedestrian_transform_2.rotation)

        # Trigger location for the actors
        self.fast_vehicle_trigger_location = carla.Location(
            fast_car_transform.location.x,
            fast_car_transform.location.y,
            fast_car_transform.location.z + 501,
        )
        self.slow_vehicle_trigger_location = carla.Location(
            slow_car_transform.location.x,
            slow_car_transform.location.y,
            slow_car_transform.location.z + 501,
        )
        self.pedestrian_trigger_location = carla.Location(
            (pedestrian_transform_1.location.x +
             pedestrian_transform_2.location.x) / 2,
            pedestrian_transform_1.location.y,
            pedestrian_transform_1.location.z + 501,
        )

        self._pedestrian_target_1 = carla.Location(
            carla.Location(pedestrian_transform_2.location.x - 15,
                           pedestrian_transform_2.location.y,
                           pedestrian_transform_2.location.z + 501)
        )
        self._pedestrian_target_2 = carla.Location(
            carla.Location(pedestrian_transform_1.location.x - 15,
                           pedestrian_transform_1.location.y,
                           pedestrian_transform_1.location.z + 501)
        )

    def _create_behavior(self):
        # Slow vehicle behaviors
        sequence_slow_vehicle = py_trees.composites.Sequence("Slow Vehicle")
        trigger_slow_vehicle = InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.slow_vehicle_trigger_location, self._trigger_distance)
        set_slow_visible = ActorTransformSetter(
            self.other_actors[1], self.slow_car_visible)
        slow_vehicle_drive = WaypointFollower(
            self.other_actors[1], self._slow_vehicle_velocity)
        sequence_slow_vehicle.add_child(set_slow_visible)
        sequence_slow_vehicle.add_child(trigger_slow_vehicle)
        sequence_slow_vehicle.add_child(slow_vehicle_drive)
        sequence_slow_vehicle.add_child(Idle())

        # Fast vehicle behaviors
        sequence_fast_vehicle = py_trees.composites.Sequence("Fast Vehicle")
        trigger_fast_vehicle = InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.fast_vehicle_trigger_location, self._trigger_distance)
        set_fast_visible = ActorTransformSetter(
            self.other_actors[0], self.fast_car_visible)
        fast_vehicle_drive = WaypointFollower(
            self.other_actors[0], self._fast_vehicle_velocity)
        # Drive fast vehicle
        sequence_fast_vehicle.add_child(set_fast_visible)
        sequence_fast_vehicle.add_child(trigger_fast_vehicle)
        sequence_fast_vehicle.add_child(fast_vehicle_drive)
        sequence_fast_vehicle.add_child(Idle())

        # Pedestrian behaviors
        sequence_pedestrians = py_trees.composites.Sequence(
            "Pedestrians", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        trigger_slow_vehicle = InTriggerDistanceToLocation(
            self.ego_vehicles[0], self.pedestrian_trigger_location, 15)
        # Pedestrian 1 behaviors
        sequence_pedestrian_1 = py_trees.composites.Sequence("Pedestrian 1")
        set_pedestrian_visible_1 = ActorTransformSetter(
            self.other_actors[2], self.pedestrian_visible_1)
        walk_1 = WaypointFollower(
            self.other_actors[2], self._pedestrian_velocity, [self._pedestrian_target_1])
        sequence_pedestrian_1.add_child(set_pedestrian_visible_1)
        sequence_pedestrian_1.add_child(trigger_slow_vehicle)
        sequence_pedestrian_1.add_child(walk_1)

        # Pedestrian 2 behaviors
        sequence_pedestrian_2 = py_trees.composites.Sequence("Pedestrian 2")
        set_pedestrian_visible_2 = ActorTransformSetter(
            self.other_actors[3], self.pedestrian_visible_2)
        walk_2 = WaypointFollower(
            self.other_actors[3], self._pedestrian_velocity, [self._pedestrian_target_2])
        sequence_pedestrian_2.add_child(set_pedestrian_visible_2)
        sequence_pedestrian_2.add_child(trigger_slow_vehicle)
        sequence_pedestrian_2.add_child(walk_2)

        pedestrians_move = py_trees.composites.Parallel(
            "Pedestrians Move", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        pedestrians_move.add_child(sequence_pedestrian_1)
        pedestrians_move.add_child(sequence_pedestrian_2)
        sequence_pedestrians.add_child(pedestrians_move)

        # End condition
        termination = DriveDistance(self.ego_vehicles[0], 250)

        # Build composite behavior tree
        root = py_trees.composites.Parallel(
            "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(sequence_slow_vehicle)
        root.add_child(sequence_fast_vehicle)
        root.add_child(sequence_pedestrians)
        root.add_child(termination)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
