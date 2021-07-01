# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


""" This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights,
traffic signs, and has different possible configurations. """

import random
import sys

import numpy as np
import carla

from opencda.core.common.misc import get_speed, positive, cal_distance_angle
from opencda.core.plan.collision_check import CollisionChecker
from opencda.core.plan.local_planner_behavior import LocalPlanner
from opencda.core.plan.global_route_planner import GlobalRoutePlanner
from opencda.core.plan.global_route_planner_dao import GlobalRoutePlannerDAO
from opencda.core.plan.planer_debug_helper import PlanDebugHelper


class BehaviorAgent(object):
    """
    A modulized version of carla BehaviorAgent.
    
    Parameters
    -vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.
    -carla_map : carla.map
        The carla HD map for simulation world.
    -config : dict
        The configuration dictionary of the localization module.
    
    Attributes
    -_ego_pos : carla.position
        Posiion of the ego vehicle. 
    -_ego_speed : float 
        Speed of the ego vehicle. 
    -_map : carla.map
        The HD map of the current simulation world.
    -max_speed : float
        The current speed limit of the ego vehicles.
    -break_distance : float
        The current distance needed for ego vehicle to reach a steady stop.
    -_collision_check : collisionchecker
        A collision check class to estimate the collision with front obstacle.
    -ignore_traffic_light : boolean
        Boolean indicator of whether to ignore traffic light.
    -overtake_allowed : boolean
        Boolean indicator of whether to allow overtake.
    -_local_planner : LocalPlanner
        A carla local planner class for behavior planning.
    -lane_change_allowed : boolean
        Boolean indicator of whether the lane change is allowed.
    -white_list : list
        The white list contains all position of target platoon member for joining.
    -debug_helper : PlanDebugHelper
        The helper class that help with the debug functions.
    """

    def __init__(self, vehicle, carla_map, config_yaml):
        
        self.vehicle = vehicle
        # ego pos(transform) and speed(km/h) retrieved from localization module
        self._ego_pos = None
        self._ego_speed = 0.0
        self._map = carla_map

        # speed related, check yaml file to see the meaning
        self.max_speed = config_yaml['max_speed']
        self.tailgate_speed = config_yaml['tailgate_speed']
        self.speed_lim_dist = config_yaml['speed_lim_dist']
        self.speed_decrease = config_yaml['speed_decrease']
        self.min_speed = 5

        # safety related
        self.safety_time = config_yaml['safety_time']
        self.emergency_param = config_yaml['emergency_param']
        self.break_distance = 0
        self.ttc = 1000
        # collision checker
        self._collision_check = CollisionChecker(time_ahead=config_yaml['collision_time_ahead'])
        self.ignore_traffic_light = config_yaml['ignore_traffic_light']
        self.overtake_allowed = config_yaml['overtake_allowed']
        self.overtake_allowed_origin = config_yaml['overtake_allowed']
        self.overtake_counter = 0
        # used to indicate whether a vehicle is on the planned path
        self.hazard_flag = False

        # route planner related
        self._global_planner = None
        self.start_waypoint = None
        self.end_waypoint = None
        self._sampling_resolution = config_yaml['sample_resolution']

        # intersection agent related
        self.light_state = "Red"
        self.light_id_to_ignore = -1

        # trajectory planner
        self._local_planner = LocalPlanner(self, carla_map, config_yaml['local_planner'])

        # special behavior rlated
        self.car_following_flag = False
        # lane change allowed flag
        self.lane_change_allowed = True
        # destination temp push flag
        self.destination_push_flag = False

        # white list of vehicle managers that the cav does not consider as obstacles
        self.white_list = []
        self.obstacle_vehicles = []

        # debug helper
        self.debug_helper = PlanDebugHelper(self.vehicle.id)

    def update_information(self, ego_pos, ego_speed, objects):
        """
        Update the perception and localization information to the behavior agent.

        Args:
            -ego_pos (carla.Transform): ego position from localization module.
            -ego_speed (float): km/h, ego speed.
            -objects (dictionary): Objects detection results from perception module.
        """
        # update localization information
        self._ego_speed = ego_speed
        self._ego_pos = ego_pos
        self.break_distance = self._ego_speed / 3.6 * self.emergency_param
        # update the localization info to trajectory planner
        self.get_local_planner().update_information(ego_pos, ego_speed)

        # current version only consider about vehicles
        obstacle_vehicles = objects['vehicles']
        self.obstacle_vehicles = self.white_list_match(obstacle_vehicles)

        # update the debug helper
        self.debug_helper.update(ego_speed, self.ttc)

        if self.ignore_traffic_light:
            self.light_state = "Green"
        else:
            # This method also includes stop signs and intersections.
            self.light_state = str(self.vehicle.get_traffic_light_state())

    def add_white_list(self, vm):
        """
        Add vehicle manager to white list.
        """
        self.white_list.append(vm)

    def white_list_match(self, obstacles):
        """
        Match the detected obstacles with the white list. Remove the obstacles that are in white list.
        The white list contains all position of target platoon member for joining.

        Args:
            -obstacles (list):  a list of carla.Vehicle or ObstacleVehicle

        Returns:
            -new_obstacle_list (list): the new list of obstacles.
        """
        new_obstacle_list = []

        for o in obstacles:
            flag = False
            o_x = o.get_location().x
            o_y = o.get_location().y

            o_waypoint = self._map.get_waypoint(o.get_location())
            o_lane_id = o_waypoint.lane_id

            for vm in self.white_list:
                pos = vm.localizer.get_ego_pos()
                vm_x = pos.location.x
                vm_y = pos.location.y

                w_waypoint = self._map.get_waypoint(pos.location)
                w_lane_id = w_waypoint.lane_id

                # if the id is different, then not matched for sure
                if o_lane_id != w_lane_id:
                    continue

                if abs(vm_x - o_x) <= 3.0 and abs(vm_y - o_y) <= 3.0:
                    flag = True
                    break
            if not flag:
                new_obstacle_list.append(o)

        return new_obstacle_list

    def set_destination(self, start_location, end_location, clean=False, end_reset=True, clean_history=False):
        """
        This method creates a list of waypoints from agent's position to destination location
        based on the route returned by the global router.

        Args:
            -end_reset (boolean): Flag to reset the waypoint queue.
            -start_location (carla.location): initial position.
            -end_location (carla.location): final position.
            -clean (boolean): Flag to clean the waypoint queue.
            -clean_history (boolean): Flag to clean the waypoint history.
        """
        if clean:
            self.get_local_planner().waypoints_queue.clear()
            self.get_local_planner().get_trajetory().clear()
            self.get_local_planner()._waypoint_buffer.clear()
        if clean_history:
            self.get_local_planner()._history_buffer.clear()

        self.start_waypoint = self._map.get_waypoint(start_location)

        # make sure the start waypoint is behind the vehicle
        if self._ego_pos:
            cur_loc = self._ego_pos.location
            cur_yaw = self._ego_pos.rotation.yaw
            _, angle = cal_distance_angle(self.start_waypoint.transform.location, cur_loc, cur_yaw)

            while angle > 90:
                self.start_waypoint = self.start_waypoint.next(1)[0]
                _, angle = cal_distance_angle(self.start_waypoint.transform.location, cur_loc, cur_yaw)

        end_waypoint = self._map.get_waypoint(end_location)
        if end_reset:
            self.end_waypoint = end_waypoint

        route_trace = self._trace_route(self.start_waypoint, end_waypoint)

        self._local_planner.set_global_plan(route_trace, clean)

    def get_local_planner(self):
        """
        return the local planner
        """
        return self._local_planner

    def reroute(self, spawn_points):
        """
        This method implements re-routing for vehicles approaching its destination.
        It finds a new target and computes another path to reach it.

        Args:
        -spawn_points (list): list of possible destinations for the agent.
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

        Args:
            -start_waypoint (carla.waypoint): initial position.
            -end_waypoint carla.waypoint: final position.
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

        Args: 
            -waypoint (carla.waypoint): current waypoint of the agent.
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

    def collision_manager(self, rx, ry, ryaw, waypoint, adjacent_check=False):
        """
        This module is in charge of warning in case of a collision.

        Args: 
        -adjacent_check (boolean): whether it is a check for adjacent lane.
        -rx (float): x coordinates of plan path.
        -ry (float): y coordinates of plan path.
        -ryaw (float): yaw angle.
        -waypoint (carla.waypoint): current waypoint of the agent.
        -vehicle_state (string): True if there is a vehicle nearby, False if not.
        -vehicle (carla.vehicle): nearby vehicle.
        Returns:
        -distance (float): distance to nearby vehicle.
        """

        def dist(v):
            return v.get_location().distance(waypoint.transform.location)

        vehicle_state = False
        min_distance = 100000
        target_vehicle = None

        for vehicle in self.obstacle_vehicles:
            collision_free = self._collision_check.collision_circle_check(rx, ry, ryaw, vehicle,
                                                                          self._ego_speed / 3.6,
                                                                          adjacent_check=adjacent_check)
            if not collision_free:
                vehicle_state = True
                distance = dist(vehicle)
                if distance < min_distance:
                    min_distance = distance
                    target_vehicle = vehicle

        return vehicle_state, target_vehicle, min_distance

    def overtake_management(self, obstacle_vehicle):
        """
        Overtake behavior.

        Args:
        -obstacle_vehicle (carla.vehicle): The obstacle vehicle.
        Return: 
        -vehicle_state (boolean): Flag indicating whether the vehicle is in dangerous state.
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
            rx, ry, ryaw = self._collision_check.adjacent_lane_collision_check(ego_loc=self._ego_pos.location,
                                                                               target_wpt=left_wpt,
                                                                               overtake=True,
                                                                               world=self.vehicle.get_world())
            vehicle_state, _, _ = self.collision_manager(rx, ry, ryaw,
                                                         self._map.get_waypoint(self._ego_pos.location),
                                                         True)
            if not vehicle_state:
                print("left overtake is operated")
                self.overtake_counter = 100
                next_wpt = left_wpt.next(self._ego_speed / 3.6 * 6)[0]
                left_wpt = left_wpt.next(5)[0]
                self.set_destination(left_wpt.transform.location, next_wpt.transform.location,
                                     clean=True, end_reset=False)
                return vehicle_state

        if (right_turn == carla.LaneChange.Right or right_turn ==
            carla.LaneChange.Both) and right_wpt and obstacle_vehicle_wpt.lane_id * right_wpt.lane_id > 0 \
                and right_wpt.lane_type == carla.LaneType.Driving:
            rx, ry, ryaw = self._collision_check.adjacent_lane_collision_check(ego_loc=self._ego_pos.location,
                                                                               target_wpt=right_wpt,
                                                                               overtake=True,
                                                                               world=self.vehicle.get_world())
            vehicle_state, _, _ = self.collision_manager(rx, ry, ryaw,
                                                         self._map.get_waypoint(self._ego_pos.location),
                                                         True)
            if not vehicle_state:
                print("right overtake is operated")
                self.overtake_counter = 100
                next_wpt = right_wpt.next(self._ego_speed / 3.6 * 6)[0]
                right_wpt = right_wpt.next(5)[0]
                self.set_destination(right_wpt.transform.location, next_wpt.transform.location,
                                     clean=True, end_reset=False)
                return vehicle_state

        return True

    def lane_change_management(self):
        """
        Identify whether a potential hazard exits if operating lane change.

        Returns:
            vehicle_state (boolean): whether the lane change is dangerous
        """
        ego_wpt = self._map.get_waypoint(self._ego_pos.location)
        ego_lane_id = ego_wpt.lane_id
        target_wpt = None

        # check the closest waypoint on the adjacent lane
        for wpt in self.get_local_planner()._waypoint_buffer:
            if wpt[0].lane_id != ego_lane_id:
                target_wpt = wpt[0]
                break
        if not target_wpt:
            return False

        rx, ry, ryaw = self._collision_check.adjacent_lane_collision_check(ego_loc=self._ego_pos.location,
                                                                           target_wpt=target_wpt,
                                                                           overtake=False,
                                                                           world=self.vehicle.get_world())
        vehicle_state, _, _ = self.collision_manager(rx, ry, ryaw,
                                                     self._map.get_waypoint(self._ego_pos.location),
                                                     adjacent_check=True)
        return not vehicle_state

    def car_following_manager(self, vehicle, distance, target_speed=None):
        """
        Module in charge of car-following behaviors when there's
        someone in front of us.
        
        Args:
            -target_speed (float): The target car following speed.
            -vehicle (carla.vehicle): Leading vehicle to follow.
            -distance (float): distance from leading vehicle.
            -control (carla.VehicleControl): Vehicle control of the next step.
        Returns: 
            -target_speed (float): The target speed for the next step.
            -target_loc (carla.location): The target location for the next step.
        """
        if not target_speed:
            target_speed = self.max_speed - self.speed_lim_dist

        vehicle_speed = get_speed(vehicle)

        delta_v = max(1, (self._ego_speed - vehicle_speed) / 3.6)
        ttc = distance / delta_v if delta_v != 0 else distance / np.nextafter(0., 1.)
        self.ttc = ttc
        # Under safety time distance, slow down.
        if self.safety_time > ttc > 0.0:
            target_speed = min(positive(vehicle_speed - self.speed_decrease),
                               target_speed)
            # print("vehicle id %d: car following decreasing speed mode, target speed %f"
            #       % (self.vehicle.id, target_speed))

        # Actual safety distance area, try to follow the speed of the vehicle in front.
        else:
            target_speed = min(max(self.min_speed, vehicle_speed + 1),
                               target_speed)
            # print("vehicle id %d: car following keep speed mode, target speed %f"
            #       % (self.vehicle.id, target_speed))
        return target_speed

    def run_step(self, target_speed=None, collision_detector_enabled=True, lane_change_allowed=True):
        """
        Execute one step of navigation

        Args: 
        -collision_detector_enabled (boolean): whether to enable collision detection.
        -target_speed (float):  a manual order to achieve certain speed.
        -lane_change_allowed (boolean): whether lane change is allowed. This is passed from platoon behavior agent.
        Returns:
        -control (carla.VehicleControl): Vehicle control of the next step.
        """
        # retrieve ego location
        ego_vehicle_loc = self._ego_pos.location
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)
        # ttc reset to 1000 at the beginning
        self.ttc = 1000

        # simulation ends condition
        if abs(self._ego_pos.location.x - self.end_waypoint.transform.location.x) <= 10 and \
                abs(self._ego_pos.location.y - self.end_waypoint.transform.location.y) <= 10:
            print('Simulation is Over')
            sys.exit(0)

        # when overtake_counter > 0, another overtake/lane change is forbidden
        if self.overtake_counter > 0:
            self.overtake_counter -= 1

        # 1: Traffic light management
        if self.traffic_light_manager(ego_vehicle_wp) != 0:
            return 0, None

        # when the temporary route is finished, we return to the global route
        if len(self.get_local_planner().waypoints_queue) == 0 \
                and len(self.get_local_planner()._waypoint_buffer) <= 2:
            print('Destination Reset!')
            self.overtake_allowed = True and self.overtake_allowed_origin
            self.destination_push_flag = False
            self.set_destination(ego_vehicle_loc, self.end_waypoint.transform.location, clean=True, clean_history=True)

        # 2: Path generation based on the global route
        rx, ry, rk, ryaw = self._local_planner.generate_path()

        # check whether lane change is allowed
        if collision_detector_enabled and \
                self.get_local_planner().lane_id_change and \
                self.overtake_counter <= 0 and \
                not self.destination_push_flag:
            self.lane_change_allowed = lane_change_allowed and self.lane_change_management()

        # 3: Collision check
        is_hazard = False
        if collision_detector_enabled:
            is_hazard, obstacle_vehicle, distance = self.collision_manager(rx, ry, ryaw, ego_vehicle_wp)
        car_following_flag = False

        if not is_hazard:
            self.hazard_flag = False

        # the case that the vehicle is doing lane change as planned but found vehicle blocking on the other lane
        if not self.lane_change_allowed and self.get_local_planner().lane_id_change \
                and not self.destination_push_flag and self.overtake_counter <= 0:
            self.overtake_allowed = False
            reset_target = ego_vehicle_wp.next(self._ego_speed / 3.6 * 3)[0]
            print('Vehicle id: %d :destination pushed forward because of potential collision' % self.vehicle.id)

            self.destination_push_flag = True
            self.set_destination(ego_vehicle_loc, reset_target.transform.location, clean=True,
                                 end_reset=False)

            rx, ry, rk, ryaw = self._local_planner.generate_path()

        # the case that vehicle is blocking in front and overtake not allowed or it is doing overtaking
        # the second condition is to prevent successive overtaking
        elif is_hazard and (not self.overtake_allowed or self.overtake_counter > 0
                            or self.get_local_planner().lane_change):
            car_following_flag = True

        elif is_hazard and self.overtake_allowed and self.overtake_counter <= 0:
            obstacle_speed = get_speed(obstacle_vehicle)
            obstacle_lane_id = self._map.get_waypoint(obstacle_vehicle.get_location()).lane_id
            ego_lane_id = self._map.get_waypoint(self._ego_pos.location).lane_id

            # overtake the obstacle vehicle only when speed is bigger and the lane id is the same
            if ego_lane_id == obstacle_lane_id:
                # this flag is used for transition from cut-in joining to back joining
                self.hazard_flag = is_hazard
                # we only consider overtaking when speed is faster than the front obstacle
                if self._ego_speed >= obstacle_speed - 5:
                    car_following_flag = self.overtake_management(obstacle_vehicle)
                else:
                    car_following_flag = True

        # 4. Car following behavior
        if car_following_flag:

            if distance < self.break_distance:
                return 0, None

            target_speed = self.car_following_manager(obstacle_vehicle, distance, target_speed)
            target_speed, target_loc = self._local_planner.run_step(rx, ry, rk, target_speed=target_speed)
            return target_speed, target_loc

        # 5. Normal behavior
        target_speed, target_loc = self._local_planner.run_step(rx, ry, rk,
                                                                target_speed=self.max_speed - self.speed_lim_dist
                                                                if not target_speed else target_speed)

        return target_speed, target_loc
