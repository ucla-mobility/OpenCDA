#!/usr/bin/env python

"""
Overtake Scenario:

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


class Scenario_2(BasicScenario):
    """
    The class spawns two background vehicles and two pedestrians in front of the ego vehicle.
    The ego vehicle is driving behind and overtaking the fast vehicle ahead

    self.other_actors[0] = fast car
    self.other_actors[1] = slow car
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
        Setup all relevant parameters and create scenario
        """
        print("Running Unprotected Left Turn Scenario")
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        self.num_vehicle = 7
        self.vehicle_01_velocity = 0  # Stopped large vehicle
        self.vehicle_02_velocity = 5  # Vehicle blocked by large vehicle
        self.vehicle_03_velocity = 5  # Vehicle in front of ego vehicle
        self.vehicle_04_velocity = 5  # Vehicle behind ego vehicle
        self.vehicle_05_velocity = 7  # Vehicle in front of block vehicle
        self.vehicle_06_velocity = 5  # Vehicle on the right side of ego vehicle
        self.vehicle_07_velocity = 5
        self._trigger_distance = 150

        super(Scenario_2, self).__init__("Scenario_2",
                                         ego_vehicles,
                                         config,
                                         world,
                                         debug_mode,
                                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        # Spawn vehicles
        for actor_config in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(
                actor_config.model, actor_config.transform)
            self.other_actors.append(actor)
            actor.set_simulate_physics(enabled=False)

        # Transformation that renders the vehicle visible
        for i in range(self.num_vehicle):
            car_transform = self.other_actors[i].get_transform()
            setattr(self, f"car_0{i + 1}_visible", carla.Transform(
                carla.Location(car_transform.location.x,
                               car_transform.location.y,
                               car_transform.location.z + 501),
                car_transform.rotation))

            # Trigger location for the actors
            setattr(self, f"vehicle_0{i + 1}_trigger_location", carla.Location(
                car_transform.location.x,
                car_transform.location.y,
                car_transform.location.z + 501, ))

    def _create_behavior(self):

        sequence_vehicle = []

        # Vehicle behaviors
        for i in range(self.num_vehicle):
            sequence_vehicle.append(py_trees.composites.Sequence(f"Vehicle_0{i + 1}"))
            trigger_location = getattr(self, f"vehicle_0{i + 1}_trigger_location")
            actor = self.other_actors[i]
            transform = getattr(self, f"car_0{i + 1}_visible")
            velocity = getattr(self, f"vehicle_0{i + 1}_velocity")

            trigger_behavior = InTriggerDistanceToLocation(self.ego_vehicles[0], trigger_location,
                                                           self._trigger_distance)
            set_transform_behavior = ActorTransformSetter(actor, transform)
            drive_behavior = WaypointFollower(actor, velocity)

            sequence_vehicle[i].add_child(set_transform_behavior)
            sequence_vehicle[i].add_child(trigger_behavior)
            sequence_vehicle[i].add_child(drive_behavior)
            sequence_vehicle[i].add_child(Idle())

        # End condition
        termination = DriveDistance(self.ego_vehicles[0], 100)

        # Build composite behavior tree
        root = py_trees.composites.Parallel(
            "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for i in range(self.num_vehicle):
            root.add_child(sequence_vehicle[i])
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
