# -*- coding: utf-8 -*-
"""
Behavior manager for RL specifically
"""
import carla
from collections import deque

from opencda.core.plan.local_planner_behavior import LocalPlanner
from opencda.core.plan.rl_local_planner_behavior import RLLocalPlanner
from opencda.core.plan.behavior_agent import BehaviorAgent
from opencda.core.plan.plan_utils import RoadOption, AgentState
from opencda.core.common.misc import get_speed, positive, cal_distance_angle


class RLBehaviorAgent(BehaviorAgent):
    """
    RL Behavior Agent that inherits the single vehicle behavior agent.
    
    """

    def __init__(self, vehicle, carla_map, config_yaml):

        super(RLBehaviorAgent, self).__init__(
            vehicle,
            carla_map,
            config_yaml)

        # rl related
        self.node_road_option = None
        self.node_waypoint = None
        self.agent_state = AgentState.IDLE
        self.speed_limit = 0
        self.distance_to_goal = 0.0
        self._navigation_waypoints_queue = deque()
        self.distances = deque()
        self.timeout = -1
        self.timeout_in_seconds = 0
        # note: No need to make buffer size configurable. Just use 100.
        self._buffer_size = 100
        self._navigation_waypoints_buffer = deque(maxlen=100)
        self._route = []
        # self._vehicle_location = None
        # init current waypoint based on ego vehicle position
        self.current_waypoint = self._map.get_waypoint(
            self.vehicle.get_location(), lane_type=carla.LaneType.Driving, project_to_road=True
        )
        self.target_waypoint = self.current_waypoint
        self._min_distance = config_yaml['rl_planner']['min_rl_plan_dist']

        self._rl_planner = RLLocalPlanner(
            self, carla_map, config_yaml['rl_planner'])

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
        self.get_rl_planner().update_information(ego_pos, ego_speed)

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

    def set_destination(self,
                        start_location,
                        end_location,
                        clean=False,
                        end_reset=True,
                        clean_history=False):

        super(RLBehaviorAgent, self).set_destination(start_location,
                                                     end_location,
                                                     clean=False,
                                                     end_reset=True,
                                                     clean_history=False)

        end_waypoint = self._map.get_waypoint(end_location)
        route_trace = self._trace_route(self.start_waypoint, end_waypoint)
        prev_loc = None

        if clean:
            # reset rl variables
            self._route = []
            self.distance_to_goal = 0.0
            self._navigation_waypoints_queue.clear()
            self._navigation_waypoints_buffer.clear()
            self.distances.clear()
        if not clean:
            self._route += route_trace

        for elem in route_trace:
            self._navigation_waypoints_queue.append(elem)
            cur_loc = elem[0].transform.location
            if prev_loc is not None:
                delta = cur_loc.distance(prev_loc)
                self.distance_to_goal += delta
                self.distances.append(delta)
            prev_loc = cur_loc

        # update rl related variables
        self.node_waypoint = self.start_waypoint
        self.node_road_option = RoadOption.LANEFOLLOW
        self.timeout_in_seconds = ((self.distance_to_goal / 1000.0) / 5.0) * 3600.0 + 20.0
        # note: do not see a point to make fps configurable, just use fps=10
        self.timeout = self.timeout_in_seconds * 10

    def apply_rl_action(
            self,
            start_location,
            end_location):
        """
        This method set the route for ego vehicle, from agent's
        position to the destination location based on RL's action.
        """
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

        # one-step route
        route_trace = self._trace_route(self.start_waypoint, end_waypoint)

        # send the new route to rl local planner for single step planning
        # Note: the rl planner automatically reset waypoint queue and buffer each step
        self._rl_planner.set_current_plan(route_trace)

    def set_route(self, route, clean=False):
        """
        This method add a route into planner to trace. If ``clean`` is set true, it will clean current
        route and waypoint queue. For RL, this function only run once when that navigation target was set.

        Parameters
        ----------
        route:list
            Route add to agent.
        clean:bool
            Whether to clean current route. Defaults to False.
        """

        if clean:
            self._navigation_waypoints_queue.clear()
            self._navigation_waypoints_buffer.clear()
            self._route = route
            self.distance_to_goal = 0
            self.distances.clear()
        else:
            self._route.extend(route)

        self.end_waypoint = self._route[-1][0]

        prev_loc = None
        for elem in route:
            self._navigation_waypoints_queue.append(elem)
            cur_loc = elem[0].transform.location
            if prev_loc is not None:
                delta = cur_loc.distance(prev_loc)
                self.distance_to_goal += delta
                self.distances.append(delta)
            prev_loc = cur_loc

        if self.distances:
            cur_resolution = np.average(list(self.distances)[:100])
            self._buffer_size = min(100, int(100 // cur_resolution))
        self.node_waypoint, self.node_road_option = self._navigation_waypoints_queue[0]
        self.timeout_in_seconds = ((self.distance_to_goal / 1000.0) / 5.0) * 3600.0 + 20.0
        # note: do not see a point to make fps configurable, just use fps=10
        self.timeout = self.timeout_in_seconds * self._fps

    def get_route(self):
        """
        Get the current route of the behavioral agent.
        Returns
        -------
        :list
            The current route of the vehicle as a list of waypoints.
        """
        return self._route

    def get_local_planner(self):
        """
        return the local planner.
        """
        return self._local_planner

    def get_rl_planner(self):
        """
        Returns the RL local planner object.
        """
        return self._rl_planner

    def get_waypoints_list(self, waypoint_num):
        """
        Return a list of wapoints from the end of waypoint buffer.
        Parameters
        ----------
        waypoint_num:int
            Number of waypoint to put in the list.
        Returns
        -------
        :List
            A list of waypoints.
        """
        num = 0
        i = 0
        waypoint_list = []
        while num < waypoint_num and i < len(self._navigation_waypoints_buffer):
            waypoint = self._navigation_waypoints_buffer[i][0]
            i += 1
            if len(waypoint_list) == 0:
                waypoint_list.append(waypoint)
                num += 1
            elif waypoint_list[-1].transform.location.distance(waypoint.transform.location) > 1e-4:
                waypoint_list.append(waypoint)
                num += 1
        return waypoint_list

    def get_direction_list(self, waypoint_num):
        """
        Get a list of possible direction in the current location.
        Parameters
        ----------
        waypoint_num:int
            The desired waypoint number to look ahead.

        Returns
        -------
        :list
            A list of direction.
        """
        num = min(waypoint_num, len(self._navigation_waypoints_buffer))
        direction_list = []
        for i in range(num):
            direction = self._navigation_waypoints_buffer[i][1].value
            direction_list.append(direction)
        return direction_list

    def get_current_waypoint(self):
        '''
        Get the CARLA waypoint based on ego position.

        Returns
        -------
        current_wpt: carla.waypoint
            The closest waypoint based on current position.
        '''

        current_location = self._ego_pos.location
        current_wpt = self._map.get_waypoint(current_location)
        return current_wpt

    def get_ego_speed(self):
        '''
        Get the ego vehicle speed in km/h.
        '''
        return self._ego_speed

    def run_step(self):
        """
        Run one step of local planner for RL model. This method updates global navigation
        status to for reward calculation, and apply new action by sending a new target waypoint
        to the rl local planner.
        """
        # ----- update global navigation status -----
        assert self._route is not None
        self.current_waypoint = self._map.get_waypoint(
            self._ego_pos.location, lane_type=carla.LaneType.Driving, project_to_road=True
        )

        # Add waypoints into buffer if empty
        if not self._navigation_waypoints_buffer:
            for i in range(min(self._buffer_size, len(self._navigation_waypoints_queue))):
                if self._navigation_waypoints_queue:
                    self._navigation_waypoints_buffer.append(self._navigation_waypoints_queue.popleft())
                else:
                    break

            # If no waypoints return with current waypoint
            if not self._navigation_waypoints_buffer:
                self.target_waypoint = self.current_waypoint
                self.node_waypoint = self.current_waypoint
                self.target_road_option = RoadOption.VOID
                self.node_road_option = RoadOption.VOID
                self.agent_state = AgentState.VOID
                return

        # Find the most far waypoint within min distance
        max_index = -1
        for i, (waypoint, _) in enumerate(self._navigation_waypoints_buffer):
            cur_dis = waypoint.transform.location.distance(self._ego_pos.location)
            if cur_dis < self._min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self.node_waypoint, self.node_road_option = self._navigation_waypoints_buffer.popleft()
                if self._navigation_waypoints_queue:
                    self._navigation_waypoints_buffer.append(self._navigation_waypoints_queue.popleft())
                if self.distances:
                    self.distance_to_goal -= self.distances.popleft()

        # Update information
        if self._navigation_waypoints_buffer:
            self.target_waypoint, self.target_road_option = self._navigation_waypoints_buffer[0]
        self.agent_state = AgentState.NAVIGATING
        self.speed_limit = self.vehicle.get_speed_limit()

        # ----- apply new action -----
        # Path generation based on the global route
        # rx, ry, rk, ryaw = self._local_planner.generate_path()
        rx, ry, rk, ryaw = self._rl_planner.generate_path()
        # calculate speed and target location (note: delete manual target speed)
        target_speed, target_loc = self._rl_planner.run_step(
            rx, ry, rk, target_speed=self.max_speed - self.speed_lim_dist)

        return target_speed, target_loc

if __name__ == "__main__":
    pass
