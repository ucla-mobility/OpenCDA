# -*- coding: utf-8 -*-

"""Behavior planning module
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


import math
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
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    carla_map : carla.map
        The carla HD map for simulation world.

    config_yaml : dict
        The configuration dictionary of the localization module.

    Attributes
    ----------
    _ego_pos : carla.position
        Posiion of the ego vehicle.

    _ego_speed : float
        Speed of the ego vehicle.

    _map : carla.map
        The HD map of the current simulation world.

    max_speed : float
        The current speed limit of the ego vehicles.

    break_distance : float
        The current distance needed for ego vehicle to reach a steady stop.

    _collision_check : collisionchecker
        A collision check class to estimate the collision with front obstacle.

    ignore_traffic_light : boolean
        Boolean indicator of whether to ignore traffic light.

    overtake_allowed : boolean
        Boolean indicator of whether to allow overtake.

    _local_planner : LocalPlanner
        A carla local planner class for behavior planning.

    lane_change_allowed : boolean
        Boolean indicator of whether the lane change is allowed.

    white_list : list
        The white list contains all position of target
        platoon member for joining.

    obstacle_vehicles : list
        The list contains all obstacle vehicles nearby.

    objects : dict
        The dictionary that contains all kinds of objects nearby.

    debug_helper : PlanDebugHelper
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

        # safety related
        self.safety_time = config_yaml['safety_time']
        self.emergency_param = config_yaml['emergency_param']
        self.break_distance = 0
        self.ttc = 1000
        # collision checker
        self._collision_check = CollisionChecker(
            time_ahead=config_yaml['collision_time_ahead'])
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
        self.stop_sign_wait_count = 0

        # trajectory planner
        self._local_planner = LocalPlanner(
            self, carla_map, config_yaml['local_planner'])

        # special behavior rlated
        self.car_following_flag = False
        # lane change allowed flag
        self.lane_change_allowed = True
        # destination temp push flag
        self.destination_push_flag = 0

        # white list of vehicle managers that the cav does not consider as
        # obstacles
        self.white_list = []
        self.obstacle_vehicles = []
        self.objects = {}

        # debug helper
        self.debug_helper = PlanDebugHelper(self.vehicle.id)

    def update_information(self, ego_pos, ego_speed, objects):
        """
        Update the perception and localization information
        to the behavior agent.

        Parameters
        ----------
        ego_pos : carla.Transform
            Ego position from localization module.

        ego_speed : float
            km/h, ego speed.

        objects : dict
            Objects detection results from perception module.
        """
        # update localization information
        self._ego_speed = ego_speed
        self._ego_pos = ego_pos
        self.break_distance = self._ego_speed / 3.6 * self.emergency_param
        # update the localization info to trajectory planner
        self.get_local_planner().update_information(ego_pos, ego_speed)

        self.objects = objects
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
        Match the detected obstacles with the white list.
        Remove the obstacles that are in white list.
        The white list contains all position of target platoon
        member for joining.

        Parameters
        ----------
        obstacles : list
            A list of carla.Vehicle or ObstacleVehicle

        Returns
        -------
        new_obstacle_list : list
            The new list of obstacles.
        """
        new_obstacle_list = []

        for o in obstacles:
            flag = False
            o_x = o.get_location().x
            o_y = o.get_location().y

            o_waypoint = self._map.get_waypoint(o.get_location())
            o_lane_id = o_waypoint.lane_id

            for vm in self.white_list:
                pos = vm.v2x_manager.get_ego_pos()
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

    def set_destination(
            self,
            start_location,
            end_location,
            clean=False,
            end_reset=True,
            clean_history=False):
        """
        This method creates a list of waypoints from agent's
        position to destination location based on the route returned
        by the global router.

        Parameters
        ----------
        end_reset : boolean
            Flag to reset the waypoint queue.

        start_location : carla.location
            Initial position.

        end_location : carla.location
            Final position.

        clean : boolean
            Flag to clean the waypoint queue.

        clean_history : boolean
            Flag to clean the waypoint history.
        """
        if clean:
            self.get_local_planner().get_waypoints_queue().clear()
            self.get_local_planner().get_trajectory().clear()
            self.get_local_planner().get_waypoint_buffer().clear()
        if clean_history:
            self.get_local_planner().get_history_buffer().clear()

        self.start_waypoint = self._map.get_waypoint(start_location)

        # make sure the start waypoint is behind the vehicle
        if self._ego_pos:
            cur_loc = self._ego_pos.location
            cur_yaw = self._ego_pos.rotation.yaw
            _, angle = cal_distance_angle(
                self.start_waypoint.transform.location, cur_loc, cur_yaw)

            while angle > 90:
                self.start_waypoint = self.start_waypoint.next(1)[0]
                _, angle = cal_distance_angle(
                    self.start_waypoint.transform.location, cur_loc, cur_yaw)

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
        This method implements re-routing for vehicles
        approaching its destination.  It finds a new target and
         computes another path to reach it.

        Parameters
        ----------
        spawn_points : list
            List of possible destinations for the agent.
        """

        print("Target almost reached, setting new destination...")
        random.shuffle(spawn_points)
        new_start = \
            self._local_planner.waypoints_queue[-1][0].transform.location
        destination = spawn_points[0].location if \
            spawn_points[0].location != new_start else spawn_points[1].location
        print("New destination: " + str(destination))

        self.set_destination(new_start, destination)

    def _trace_route(self, start_waypoint, end_waypoint):
        """
        This method sets up a global router and returns the
        optimal route from start_waypoint to end_waypoint.

        Parameters
        ----------
        start_waypoint : carla.waypoint
            Initial position.

        end_waypoint : carla.waypoint
            Final position.
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
        WARNING: What follows is a proxy to avoid having a car brake after
        running a yellow light. This happens because the car is still under
        the influence of the semaphore, even after passing it.
        So, the semaphore id is temporarely saved to ignore it and go around
        this issue, until the car is near a new one.

        Parameters
        ----------
        waypoint : carla.waypoint
            Current waypoint of the agent.

        """

        light_id = self.vehicle.get_traffic_light(
        ).id if self.vehicle.get_traffic_light() is not None else -1

        # this is the case where the vehicle just pass a stop sign, and won't
        # stop at any stop sign in the next 4 seconds.
        if 60 <= self.stop_sign_wait_count < 240:
            self.stop_sign_wait_count += 1
        elif self.stop_sign_wait_count >= 240:
            self.stop_sign_wait_count = 0

        if self.light_state == "Red":
            # when light state is red and light id is -1, it means the vehicle
            # is near a stop sign.
            if light_id == -1:
                # we force the vehicle wait for 2 sceconds in front of the
                # stop sign
                if self.stop_sign_wait_count < 60:
                    self.stop_sign_wait_count += 1
                    # indicate emergent stop needed
                    return 1
                # After pass a stop sign, the vehicle shouldn't stop at
                # the stop sign in the opposite direction
                else:
                    # indicate no need to stop
                    return 0


            if not waypoint.is_junction and (
                    self.light_id_to_ignore != light_id or light_id == -1):
                return 1
            elif waypoint.is_junction and light_id != -1:
                self.light_id_to_ignore = light_id
        if self.light_id_to_ignore != light_id:
            self.light_id_to_ignore = -1
        return 0

    def collision_manager(self, rx, ry, ryaw, waypoint, adjacent_check=False):
        """
        This module is in charge of warning in case of a collision.

        Parameters
        ----------
        rx : float
            x coordinates of plan path.

        ry : float
            y coordinates of plan path.

        ryaw : float
            yaw angle.

        waypoint : carla.waypoint
            current waypoint of the agent.

        adjacent_check : boolean
            Whether it is a check for adjacent lane.
        """

        def dist(v):
            return v.get_location().distance(waypoint.transform.location)

        vehicle_state = False
        min_distance = 100000
        target_vehicle = None

        for vehicle in self.obstacle_vehicles:
            collision_free = self._collision_check.collision_circle_check(
                rx, ry, ryaw, vehicle, self._ego_speed / 3.6, self._map,
                adjacent_check=adjacent_check)
            if not collision_free:
                vehicle_state = True

                # the vehicle length is typical 3 meters,
                # so we need to consider that when calculating the distance
                distance = positive(dist(vehicle) - 3)

                if distance < min_distance:
                    min_distance = distance
                    target_vehicle = vehicle

        return vehicle_state, target_vehicle, min_distance

    def overtake_management(self, obstacle_vehicle):
        """
        Overtake behavior.

        Parameters
        ----------
        obstacle_vehicle : carla.vehicle
            The obstacle vehicle.

        Return
        ------
        vehicle_state : boolean
            Flag indicating whether the vehicle is in dangerous state.
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
            carla.LaneChange.Both) and \
                left_wpt and \
                obstacle_vehicle_wpt.lane_id * left_wpt.lane_id > 0 and \
                left_wpt.lane_type == carla.LaneType.Driving:
            # this not the real plan path, but just a quick path to check
            # collision
            rx, ry, ryaw = self._collision_check.adjacent_lane_collision_check(
                ego_loc=self._ego_pos.location, target_wpt=left_wpt,
                carla_map=self._map,
                overtake=True, world=self.vehicle.get_world())
            vehicle_state, _, _ = self.collision_manager(
                rx, ry, ryaw, self._map.get_waypoint(
                    self._ego_pos.location), True)
            if not vehicle_state:
                print("left overtake is operated")
                self.overtake_counter = 100
                next_wpt_list = left_wpt.next(self._ego_speed / 3.6 * 6)
                if len(next_wpt_list) == 0:
                    return True

                next_wpt = next_wpt_list[0]
                left_wpt = left_wpt.next(5)[0]
                self.set_destination(
                    left_wpt.transform.location,
                    next_wpt.transform.location,
                    clean=True,
                    end_reset=False)
                return vehicle_state

        if (right_turn == carla.LaneChange.Right or right_turn ==
            carla.LaneChange.Both) and \
                right_wpt and \
                obstacle_vehicle_wpt.lane_id * right_wpt.lane_id > 0 \
                and right_wpt.lane_type == carla.LaneType.Driving:
            rx, ry, ryaw = self._collision_check.adjacent_lane_collision_check(
                ego_loc=self._ego_pos.location,
                target_wpt=right_wpt,
                overtake=True,
                carla_map=self._map,
                world=self.vehicle.get_world())

            vehicle_state, _, _ = self.collision_manager(
                rx, ry, ryaw, self._map.get_waypoint(
                    self._ego_pos.location), True)
            if not vehicle_state:
                print("right overtake is operated")
                self.overtake_counter = 100
                next_wpt_list = right_wpt.next(self._ego_speed / 3.6 * 6)
                if len(next_wpt_list) == 0:
                    return True

                next_wpt = next_wpt_list[0]
                right_wpt = right_wpt.next(5)[0]
                self.set_destination(
                    right_wpt.transform.location,
                    next_wpt.transform.location,
                    clean=True,
                    end_reset=False)
                return vehicle_state

        return True

    def lane_change_management(self):
        """
        Identify whether a potential hazard exits if operating lane change.

        Returns
        -------
        vehicle_state : boolean
            Whether the lane change is dangerous.
        """
        ego_wpt = self._map.get_waypoint(self._ego_pos.location)
        ego_lane_id = ego_wpt.lane_id
        target_wpt = None

        # check the closest waypoint on the adjacent lane
        for wpt in self.get_local_planner().get_waypoint_buffer():
            if wpt[0].lane_id != ego_lane_id:
                target_wpt = wpt[0]
                break
        if not target_wpt:
            return True

        rx, ry, ryaw = self._collision_check.adjacent_lane_collision_check(
            ego_loc=self._ego_pos.location,
            target_wpt=target_wpt,
            overtake=False,
            carla_map=self._map,
            world=self.vehicle.get_world())
        vehicle_state, _, _ = self.collision_manager(
            rx, ry, ryaw, self._map.get_waypoint(
                self._ego_pos.location), adjacent_check=True)
        return not vehicle_state

    def car_following_manager(self, vehicle, distance, target_speed=None):
        """
        Module in charge of car-following behaviors when there's
        someone in front of us.

        Parameters
        ----------
        vehicle : carla.vehicle)
            Leading vehicle to follow.

        distance : float
            distance from leading vehicle.

        target_speed : float
            The target car following speed.

        Returns
        -------
        target_speed : float
            The target speed for the next step.

        target_loc : carla.Location
            The target location.
        """
        if not target_speed:
            target_speed = self.max_speed - self.speed_lim_dist

        vehicle_speed = get_speed(vehicle)

        delta_v = max(1, (self._ego_speed - vehicle_speed) / 3.6)
        ttc = distance / delta_v if delta_v != 0 else distance / \
                                                      np.nextafter(0., 1.)
        self.ttc = ttc
        # Under safety time distance, slow down.
        if self.safety_time > ttc > 0.0:
            target_speed = min(positive(vehicle_speed - self.speed_decrease),
                               target_speed)

        # Actual safety distance area, try to follow the speed of the vehicle
        # in front.
        else:
            target_speed = 0 if vehicle_speed == 0 else \
                min(vehicle_speed + 1,
                    target_speed)
        return target_speed

    def is_intersection(self, objects, waypoint_buffer):
        """
        Check the next waypoints is near the intersection. This is done by
        check the distance between the waypoints and the traffic light.

        Parameters
        ----------
        objects : dict
            The dictionary contains all objects info.

        waypoint_buffer : deque
            The waypoint buffer.

        Returns
        -------
        is_junc : boolean
            Whether there is any future waypoint in the junction shortly.
        """
        for tl in objects['traffic_lights']:
            for wpt, _ in waypoint_buffer:
                distance = \
                    tl.get_location().distance(wpt.transform.location)
                if distance < 20:
                    return True
        return False

    def is_close_to_destination(self):
        """
        Check if the current ego vehicle's position is close to destination

        Returns
        -------
        flag : boolean
            It is True if the current ego vehicle's position is close to destination

        """
        flag = abs(self._ego_pos.location.x - self.end_waypoint.transform.location.x) <= 10 and \
               abs(self._ego_pos.location.y - self.end_waypoint.transform.location.y) <= 10
        return flag

    def check_lane_change_permission(self, lane_change_allowed, collision_detector_enabled, rk):
        """
        Check if lane change is allowed.
        Several conditions will influence the result such as the road curvature, collision detector, overtake and push status.
        Please refer to the code for complete conditions.

        Parameters
        ----------
        lane_change_allowed : boolean
            Previous lane change permission.

        collision_detector_enabled : boolean
            True if collision detector is enabled.

        rk : list
            List of planned path points' curvatures.

        Returns
        -------
        lane_change_enabled : boolean
            True if lane change is allowed


        """
        # the lane change is forbidden if driving within a large curve
        if len(rk) > 2 and np.mean(np.abs(np.array(rk))) > 0.04:
            lane_change_allowed = False
        # change the lane change permission only when all of the following conditions are satisfied:
        # * collision detector is enabled : otherwise we won't perform collision check for lane change.
        # * lane id changes and lane lateral changes : makes sure it is indeed a lane change happen in our planned route.
        # * overtake hasn't happened : if previously we have been doing an overtake, then lane change should not be allowed.
        # * destination is not pushed : if we have been doing destination pushed, then lane change should not be allowed.
        lane_change_enabled_flag = collision_detector_enabled and \
               self.get_local_planner().lane_id_change and \
               self.get_local_planner().lane_lateral_change and \
               self.overtake_counter <= 0 and \
               not self.destination_push_flag
        if lane_change_enabled_flag:
            lane_change_allowed = lane_change_allowed and self.lane_change_management()
            if not lane_change_allowed:
                print("lane change not allowed")

        return lane_change_allowed

    def get_push_destination(self, ego_vehicle_wp, is_intersection):
        """
        Get the destination for push operation.

        Parameters
        ----------
        ego_vehicle_wp : carla.waypoint
            Ego vehicle's waypoint.

        is_intersection : boolean
            True if in the intersection.

        Returns
        -------
        reset_target : carla.waypoint
            Temporal push destination.

        """
        waypoint_buffer = self.get_local_planner().get_waypoint_buffer()
        reset_index = len(waypoint_buffer) // 2

        # when it comes to the intersection, we need to use the future
        # waypoint to make sure the next waypoint is at the same lane
        if is_intersection:
            reset_target = waypoint_buffer[reset_index][0].next(
                max(self._ego_speed / 3.6, 10.0))[0]
        else:
            reset_target = \
                ego_vehicle_wp.next(max(self._ego_speed / 3.6 * 3,
                                        10.0))[0]

        print(
            'Vehicle id: %d :destination pushed forward because of '
            'potential collision, reset destination :%f. %f, %f' %
            (self.vehicle.id, reset_target.transform.location.x,
             reset_target.transform.location.y,
             reset_target.transform.location.z))
        return reset_target

    def run_step(
            self,
            target_speed=None,
            collision_detector_enabled=True,
            lane_change_allowed=True):
        """
        Execute one step of navigation.

        Parameters
        __________
        collision_detector_enabled : boolean
            Whether to enable collision detection.

        target_speed : float
            A manual order to achieve certain speed.

        lane_change_allowed : boolean
            Whether lane change is allowed. This is passed from
            platoon behavior agent.

        Returns
        -------
        control : carla.VehicleControl
            Vehicle control of the next step.
        """
        # retrieve ego location
        ego_vehicle_loc = self._ego_pos.location
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)
        waipoint_buffer = self.get_local_planner().get_waypoint_buffer()
        # ttc reset to 1000 at the beginning
        self.ttc = 1000
        # when overtake_counter > 0, another overtake/lane change is forbidden
        if self.overtake_counter > 0:
            self.overtake_counter -= 1

        # we reset destination push flag for every n rounds
        if self.destination_push_flag > 0:
            self.destination_push_flag -= 1

        # use traffic light to detect intersection
        is_intersection = self.is_intersection(self.objects, waipoint_buffer)

        # 0. Simulation ends condition
        if self.is_close_to_destination():
            print('Simulation is Over')
            sys.exit(0)

        # 1. Traffic light management
        if self.traffic_light_manager(ego_vehicle_wp) != 0:
            return 0, None

        # 2. when the temporary route is finished, we return to the global route
        if len(self.get_local_planner().get_waypoints_queue()) == 0 \
                and len(self.get_local_planner().get_waypoint_buffer()) <= 2:
            print('Destination Reset!')
            # in case the vehicle is disabled overtaking function
            # at the beginning
            self.overtake_allowed = True and self.overtake_allowed_origin
            self.lane_change_allowed = True
            self.destination_push_flag = 0
            self.set_destination(
                ego_vehicle_loc,
                self.end_waypoint.transform.location,
                clean=True,
                clean_history=True)

        # intersection behavior. if the car is near a intersection, no overtake is allowed
        if is_intersection:
            self.overtake_allowed = False
        else:
            self.overtake_allowed = True and self.overtake_allowed_origin

        # Path generation based on the global route
        rx, ry, rk, ryaw = self._local_planner.generate_path()


        # check whether lane change is allowed
        self.lane_change_allowed = self.check_lane_change_permission(lane_change_allowed, collision_detector_enabled, rk)

        # 3. Collision check
        is_hazard = False
        if collision_detector_enabled:
            is_hazard, obstacle_vehicle, distance = self.collision_manager(
                rx, ry, ryaw, ego_vehicle_wp)
        car_following_flag = False

        if not is_hazard:
            self.hazard_flag = False

        # 4. push case. Push the car to a temporary destination when original lane change action can't be executed
        # The case that the vehicle is doing lane change as planned
        # but found vehicle blocking on the other lane
        if not self.lane_change_allowed and \
                self.get_local_planner().potential_curved_road \
                and not self.destination_push_flag and \
                self.overtake_counter <= 0:
            self.overtake_allowed = False
            # get push destination based on intersection flag and current waypoint (rule-based)
            reset_target = self.get_push_destination(ego_vehicle_wp, is_intersection)
            # set the flag, so the push operation is not allowed for the next few frames.
            self.destination_push_flag = 90
            self.set_destination(
                ego_vehicle_loc,
                reset_target.transform.location,
                clean=True,
                end_reset=False)
            rx, ry, rk, ryaw = self._local_planner.generate_path()

        # 5. the case that vehicle is blocking in front and overtake not
        # allowed or it is doing overtaking the second condition is to
        # prevent successive overtaking
        elif is_hazard and (not self.overtake_allowed or
                            self.overtake_counter > 0
                            or self.get_local_planner().potential_curved_road):
            car_following_flag = True
        # 6. overtake handeling
        elif is_hazard and self.overtake_allowed and \
                self.overtake_counter <= 0:
            obstacle_speed = get_speed(obstacle_vehicle)
            obstacle_lane_id = self._map.get_waypoint(obstacle_vehicle.get_location()).lane_id
            ego_lane_id = self._map.get_waypoint(
                self._ego_pos.location).lane_id
            # overtake the obstacle vehicle only when speed is bigger and the
            # lane id is the same
            if ego_lane_id == obstacle_lane_id:
                # this flag is used for transition from cut-in joining to back
                # joining
                self.hazard_flag = is_hazard
                # we only consider overtaking when speed is faster than the
                # front obstacle

                if self._ego_speed >= obstacle_speed - 5:
                    car_following_flag = self.overtake_management(obstacle_vehicle)
                else:
                    car_following_flag = True

        # 7. Car following behavior
        if car_following_flag:
            if distance < max(self.break_distance, 3):
                return 0, None

            target_speed = self.car_following_manager(obstacle_vehicle, distance, target_speed)
            target_speed, target_loc = self._local_planner.run_step(
                rx, ry, rk, target_speed=target_speed)
            return target_speed, target_loc

        # 8. Normal behavior
        target_speed, target_loc = self._local_planner.run_step(
            rx, ry, rk, target_speed=self.max_speed - self.speed_lim_dist
            if not target_speed else target_speed)
        return target_speed, target_loc

