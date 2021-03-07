# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


""" This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights,
traffic signs, and has different possible configurations. """

import random
import numpy as np
import carla

from core.agents.navigation.collision_check import CollisionChecker
from core.agents.navigation.agent import Agent
from core.agents.navigation.local_planner_behavior import RoadOption
from core.agents.navigation.global_route_planner import GlobalRoutePlanner
from core.agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from core.agents.navigation.types_behavior import Cautious, Aggressive, Normal
from core.agents.tools.misc import get_speed, positive
from customize.local_planner_behavior import CustomizedLocalPlanner


class BehaviorAgent(Agent):
    """
    A modulized version of BehaviorAgent
    """

    def __init__(self, vehicle, ignore_traffic_light=False, behavior='normal', overtake_allowed=False,
                 sampling_resolution=4.5, buffer_size=5, dynamic_pid=False, debug_trajectory=False,
                 debug=False, update_freq=15, time_ahead=2.0):
        """
        Construct class
        :param vehicle: actor
        :param ignore_traffic_light: whether to ignore certain traffic light
        :param behavior: driving style
        :param overtake_allowed: whether overtaking is allowed
        :param sampling_resolution: the minimum distance between each waypoint
        :param buffer_size: buffer size for local route
        :param dynamic_pid; whether to use dynamic pid params generation. Set to true will require users
        provide customized function under customize/controller
        """

        super(BehaviorAgent, self).__init__(vehicle)

        self.vehicle = vehicle
        # the frontal vehicle manager in the platooning
        self.frontal_vehicle = None
        self._platooning_world = None

        self.ignore_traffic_light = ignore_traffic_light
        self.look_ahead_steps = 0

        self._local_planner = CustomizedLocalPlanner(self, buffer_size=buffer_size, dynamic_pid=dynamic_pid,
                                                     debug_trajectory=debug_trajectory, debug=debug,
                                                     update_freq=update_freq)
        self._global_planner = None

        # Vehicle information
        self.speed = 0
        self.speed_limit = 0
        self.direction = None

        self.incoming_direction = None
        self.incoming_waypoint = None
        self.start_waypoint = None
        self.end_waypoint = None

        self.is_at_traffic_light = 0
        self.light_state = "Green"
        self.light_id_to_ignore = -1

        self.min_speed = 5
        self.behavior = None
        self._sampling_resolution = sampling_resolution

        # collision checker
        self._collision_check = CollisionChecker(time_ahead=time_ahead)
        self.overtake_allowed = overtake_allowed

        # emergency stop flag
        self.hazard_flag = False

        # car following flag
        self.car_following_flag = False

        # Parameters for agent behavior
        if behavior == 'cautious':
            self.behavior = Cautious()

        elif behavior == 'normal':
            self.behavior = Normal()

        elif behavior == 'aggressive':
            self.behavior = Aggressive()

    def update_information(self, world, frontal_vehicle=None):
        """
        This method updates the information regarding the ego
        vehicle based on the surrounding world.

            :param frontal_vehicle: the vehicle manager in front in the platooning
            :param world: platooning world object
        """
        self.speed = get_speed(self.vehicle)
        self.speed_limit = self.vehicle.get_speed_limit()
        self._local_planner.set_speed(self.speed_limit)
        self.direction = self._local_planner.target_road_option
        if self.direction is None:
            self.direction = RoadOption.LANEFOLLOW

        self.frontal_vehicle = frontal_vehicle
        self.look_ahead_steps = int(self.speed_limit / 10)

        self.incoming_waypoint, self.incoming_direction = self._local_planner.get_incoming_waypoint_and_direction(
            steps=self.look_ahead_steps)
        if self.incoming_direction is None:
            self.incoming_direction = RoadOption.LANEFOLLOW

        self.is_at_traffic_light = self.vehicle.is_at_traffic_light()
        if self.ignore_traffic_light:
            self.light_state = "Green"
        else:
            # This method also includes stop signs and intersections.
            self.light_state = str(self.vehicle.get_traffic_light_state())

        self._platooning_world = world

    def set_destination(self, start_location, end_location, clean=False):
        """
        This method creates a list of waypoints from agent's position to destination location
        based on the route returned by the global router.

            :param start_location: initial position
            :param end_location: final position
            :param clean: boolean to clean the waypoint queue
        """
        if clean:
            self._local_planner.waypoints_queue.clear()
            self._local_planner._trajectory_buffer.clear()
            self._local_planner._waypoint_buffer.clear()

        self.start_waypoint = self._map.get_waypoint(start_location)
        self.end_waypoint = self._map.get_waypoint(end_location)

        route_trace = self._trace_route(self.start_waypoint, self.end_waypoint)

        self._local_planner.set_global_plan(route_trace, clean)

    def get_local_planner(self):
        """return the local planner
        """
        return self._local_planner

    def set_controller_longitudinal(self, max_throttle, max_brake):
        """
        Set the max throttle and brake for controller
        :param max_throttle:
        :param max_brake:
        :return:
        """
        self._local_planner.set_controller_longitudinal(max_throttle, max_brake)

    def reroute(self, spawn_points):
        """
        This method implements re-routing for vehicles approaching its destination.
        It finds a new target and computes another path to reach it.

            :param spawn_points: list of possible destinations for the agent
        """

        print("Target almost reached, setting new destination...")
        random.shuffle(spawn_points)
        new_start = self._local_planner.waypoints_queue[-1][0].transform.location
        destination = spawn_points[0].location if spawn_points[0].location != new_start else spawn_points[1].location
        print("New destination: " + str(destination))

        self.set_destination(new_start, destination)

    def _trace_route(self, start_waypoint, end_waypoint):
        """
        This method sets up a global router and returns the
        optimal route from start_waypoint to end_waypoint.

            :param start_waypoint: initial position
            :param end_waypoint: final position
        """
        # Setting up global router
        if self._global_planner is None:
            wld = self.vehicle.get_world()
            dao = GlobalRoutePlannerDAO(
                wld.get_map(), sampling_resolution=self._sampling_resolution)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._global_planner = grp

        # Obtain route plan
        route = self._global_planner.trace_route(
            start_waypoint.transform.location,
            end_waypoint.transform.location)

        return route

    def traffic_light_manager(self, waypoint):
        """
        This method is in charge of behaviors for red lights and stops.

        WARNING: What follows is a proxy to avoid having a car brake after running a yellow light.
        This happens because the car is still under the influence of the semaphore,
        even after passing it. So, the semaphore id is temporarely saved to
        ignore it and go around this issue, until the car is near a new one.

            :param waypoint: current waypoint of the agent
        """

        light_id = self.vehicle.get_traffic_light().id if self.vehicle.get_traffic_light() is not None else -1

        if self.light_state == "Red":
            if not waypoint.is_junction and (self.light_id_to_ignore != light_id or light_id == -1):
                return 1
            elif waypoint.is_junction and light_id != -1:
                self.light_id_to_ignore = light_id
        if self.light_id_to_ignore != light_id:
            self.light_id_to_ignore = -1
        return 0

    def collision_manager(self, rx, ry, ryaw, waypoint, overtake_check=False):
        """
        This module is in charge of warning in case of a collision
        :param overtake_check: whether it is a check for overtaking
        :param rx: x coordinates of plan path
        :param ry: y coordinates of plan path
        :param ryaw: yaw angle
        :param waypoint: current waypoint of the agent
        :return vehicle_state: True if there is a vehicle nearby, False if not
        :return vehicle: nearby vehicle
        :return distance: distance to nearby vehicle
        """

        vehicle_list = self._world.get_actors().filter("*vehicle*")

        def dist(v):
            return v.get_location().distance(waypoint.transform.location)

        # only consider vehicles in 45 meters, not in the platooning as the candidate of collision
        vehicle_list = [v for v in vehicle_list if dist(v) < 60 and
                        v.id != self.vehicle.id and
                        v.id not in self._platooning_world.vehicle_id_set]

        vehicle_state = False
        min_distance = 100000
        target_vehicle = None

        for vehicle in vehicle_list:
            collision_free = self._collision_check.collision_circle_check(rx, ry, ryaw, vehicle,
                                                                          get_speed(self.vehicle, True),
                                                                          overtake_check=overtake_check)
            if not collision_free:
                vehicle_state = True
                distance = dist(vehicle)
                if distance < min_distance:
                    min_distance = distance
                    target_vehicle = vehicle

        return vehicle_state, target_vehicle, min_distance

    def car_following_manager(self, vehicle, distance, target_speed=None):
        """
        Module in charge of car-following behaviors when there's
        someone in front of us.

            :param target_speed:
            :param vehicle: car to follow
            :param distance: distance from vehicle
            :return control: carla.VehicleControl
        """
        if not target_speed:
            target_speed = self.behavior.max_speed - self.behavior.speed_lim_dist

        vehicle_speed = get_speed(vehicle)
        delta_v = max(1, (self.speed - vehicle_speed) / 3.6)
        ttc = distance / delta_v if delta_v != 0 else distance / np.nextafter(0., 1.)
        # Under safety time distance, slow down.
        if self.behavior.safety_time > ttc > 0.0:
            target_speed = min(positive(vehicle_speed - self.behavior.speed_decrease),
                               target_speed)

        # Actual safety distance area, try to follow the speed of the vehicle in front.
        elif 3 * self.behavior.safety_time > ttc >= self.behavior.safety_time:
            target_speed = min(max(self.min_speed, vehicle_speed),
                              target_speed)
        return target_speed

    def overtake_management(self, obstacle_vehicle):
        """
        Overtake behavior for back_joining car
        :param obstacle_vehicle: the vehicle
        :return:
        """
        # obstacle vehicle's location
        obstacle_vehicle_loc = obstacle_vehicle.get_location()
        obstacle_vehicle_wpt = self._map.get_waypoint(obstacle_vehicle_loc)

        # whether a lane change is allowed
        left_turn = obstacle_vehicle_wpt.left_lane_marking.lane_change
        right_turn = obstacle_vehicle_wpt.right_lane_marking.lane_change

        # left and right waypoint of the obstacle vehicle
        left_wpt = obstacle_vehicle_wpt.get_left_lane()
        right_wpt = obstacle_vehicle_wpt.get_right_lane()

        # if the vehicle is able to operate left overtake
        if (left_turn == carla.LaneChange.Left or left_turn ==
            carla.LaneChange.Both) and left_wpt and obstacle_vehicle_wpt.lane_id * left_wpt.lane_id > 0 \
                and left_wpt.lane_type == carla.LaneType.Driving:
            # this not the real plan path, but just a quick path to check collision
            rx, ry, ryaw = self._collision_check.overtake_collision_path(ego_loc=self.vehicle.get_location(),
                                                                         target_wpt=left_wpt,
                                                                         world=self.vehicle.get_world())
            vehicle_state, _, _ = self.collision_manager(rx, ry, ryaw,
                                                         self._map.get_waypoint(self.vehicle.get_location()),
                                                         True)
            if not vehicle_state:
                print("left overtake is operated")
                self.behavior.overtake_counter = 35
                self.set_destination(left_wpt.transform.location, self.end_waypoint.transform.location, clean=True)
                return vehicle_state

        if (right_turn == carla.LaneChange.Right or right_turn ==
            carla.LaneChange.Both) and right_wpt and obstacle_vehicle_wpt.lane_id * right_wpt.lane_id > 0 \
                and right_wpt.lane_type == carla.LaneType.Driving:
            rx, ry, ryaw = self._collision_check.overtake_collision_path(ego_loc=self.vehicle.get_location(),
                                                                         target_wpt=right_wpt,
                                                                         world=self.vehicle.get_world())
            vehicle_state, _, _ = self.collision_manager(rx, ry, ryaw,
                                                         self._map.get_waypoint(self.vehicle.get_location()),
                                                         True)
            if not vehicle_state:
                print("right overtake is operated")
                self.behavior.overtake_counter = 35
                self.set_destination(right_wpt.transform.location, self.end_waypoint.transform.location, clean=True)
                return vehicle_state

        return True

    def run_step(self, target_speed=None, collision_detector_enabled=True):
        """
        Excute one step of naviation
        :param collision_detector_enabled: whether to enable collision detection
        :param target_speed:  a manual order to achieve certain speed
        :return: control: carla.VehicleControl
        """
        if self.behavior.overtake_counter > 0:
            self.behavior.overtake_counter -= 1

        ego_vehicle_loc = self.vehicle.get_location()
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)

        # 1: Red lights and stops behavior
        if self.traffic_light_manager(ego_vehicle_wp) != 0:
            return self.emergency_stop()

        # 2: generated plan path first
        rx, ry, rk, ryaw = self._local_planner.generate_path()

        # TODO: Hard-coded, revise it later
        if self.get_local_planner().lane_change:
            self._collision_check.time_ahead = 0.5
        else:
            self._collision_check.time_ahead = 1.2

        # 3: collision check
        is_hazard = False
        if collision_detector_enabled:
            is_hazard, obstacle_vehicle, distance = self.collision_manager(rx, ry, ryaw, ego_vehicle_wp)
        car_following_flag = False

        # this flag is used for transition from cut-in joining to back joining
        self.hazard_flag = True if is_hazard else False

        # the case that the vehicle is doing lane change as planned but found vehicle blocking on the other lane
        if is_hazard \
                and self.get_local_planner().lane_change \
                and self.behavior.overtake_counter <= 0 \
                and self._map.get_waypoint(obstacle_vehicle.get_location()).lane_id != ego_vehicle_wp.lane_id:

            reset_target = ego_vehicle_wp.next(get_speed(self.vehicle, True))[0]
            print('destination pushed forward because of potential collision')
            self.set_destination(reset_target.transform.location, self.end_waypoint.transform.location, clean=True)
            rx, ry, rk, ryaw = self._local_planner.generate_path()

        # the case that vehicle is blocing in front and overtake not allowed or it is doing overtaking
        # the second condistion is to prevent successive overtaking
        elif is_hazard and (not self.overtake_allowed or self.behavior.overtake_counter > 0):
            car_following_flag = True

        elif is_hazard and self.overtake_allowed and self.behavior.overtake_counter <= 0:
            obstacle_speed = get_speed(obstacle_vehicle)
            # overtake the vehicle
            if get_speed(self.vehicle) > obstacle_speed:
                car_following_flag = self.overtake_management(obstacle_vehicle)
            else:
                car_following_flag = True

        if not car_following_flag:
            self.car_following_flag = False
        else:
            print("vehicle id %d: car following mode!" % self.vehicle.id)
            self.car_following_flag = True

            if distance < self.behavior.braking_distance:
                return self.emergency_stop()

            target_speed = self.car_following_manager(obstacle_vehicle, distance, target_speed)
            control = self._local_planner.run_step(rx, ry, rk, target_speed=target_speed)
            return control

        # 4. Checking if there's a junction nearby to slow down TODO: This is a very ROUGH WAY for now
        if self.incoming_waypoint.is_junction and (
                self.incoming_direction == RoadOption.LEFT or self.incoming_direction == RoadOption.RIGHT):
            control = self._local_planner.run_step(rx, ry, rk,
                                                   target_speed=min(self.behavior.max_speed, 24))
            return control

        # 5. normal behavior
        control = self._local_planner.run_step(rx, ry, rk,
                                               target_speed=self.behavior.max_speed - self.behavior.speed_lim_dist
                                               if not target_speed else target_speed)

        return control
