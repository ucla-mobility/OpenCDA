# -*- coding: utf-8 -*-
"""
Behavior manager for RL specifically
"""
import carla
from collections import deque

from opencda.core.plan.behavior_agent import BehaviorAgent
from opencda.core.plan.plan_utils import RoadOption, AgentState

class RLBehaviorAgent(BehaviorAgent):
    """
    RL Behavior Agent that inherits the single vehicle behavior agent.
    
    Parameters
    ----------
    
    
    Attributes
    ----------
    
    """
    def __init__(self, vehicle, carla_map, behavior_yaml):
        
        super(RLBehaviorAgent,self).__init__(
            vehicle,
            carla_map,
            behavior_yaml)
        
        # rl related
        self.node_road_option = None
        self.node_waypoint = None
        self.agent_state = AgentState.IDLE
        self.speed_limit = 0
        self.distance_to_goal = 0.0
        self._waypoints_queue = deque()
        self.distances = deque()
        self.timeout = -1
        self.timeout_in_seconds = 0
        # note: No need to make buffer size configurable. Just use 100.
        self._buffer_size = 100
        self._waypoints_buffer = deque(maxlen=100)
        self._route = []
        # self._vehicle_location = None
        # init current waypoint based on ego vehicle position
        self.current_waypoint = self._map.get_waypoint(
            self.vehicle.get_location(), lane_type=carla.LaneType.Driving, project_to_road=True
        )
        self.target_waypoint = self.current_waypoint

        # todo: change it to rl specific module
        self._min_distance = 5
        
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
        return the local planner
        """
        return self._local_planner

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
        while num < waypoint_num and i < len(self._waypoints_buffer):
            waypoint = self._waypoints_buffer[i][0]
            i += 1
            if len(waypoint_list) == 0:
                waypoint_list.append(waypoint)
                num + 1
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
        num = min(waypoint_num, len(self._waypoints_buffer))
        direction_list = []
        for i in range(num):
            direction = self._waypoints_buffer[i][1].value
            direction_list.append(direction)
        return direction_list
    
    def run_step(self):
        """
        Run one step of local planner for RL model. This method will update node, target waypoint,
        road option, and check agent states. The vehicle manager will get the updated info to
        compose navigation dict for carla_env class.
        """
        assert self._route is not None

        self.current_waypoint = self._map.get_waypoint(
            self._ego_pos.location, lane_type=carla.LaneType.Driving, project_to_road=True
        )

        # Add waypoints into buffer if empty
        if not self._waypoints_buffer:
            for i in range(min(self._buffer_size, len(self._waypoints_queue))):
                if self._waypoints_queue:
                    self._waypoints_buffer.append(self._waypoints_queue.popleft())
                else:
                    break

            # If no waypoints return with current waypoint
            if not self._waypoints_buffer:
                self.target_waypoint = self.current_waypoint
                self.node_waypoint = self.current_waypoint
                self.target_road_option = RoadOption.VOID
                self.node_road_option = RoadOption.VOID
                self.agent_state = AgentState.VOID
                return

        # Find the most far waypoint within min distance
        max_index = -1
        for i, (waypoint, _) in enumerate(self._waypoints_buffer):
            cur_dis = waypoint.transform.location.distance(self._ego_pos.location)
            if cur_dis < self._min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self.node_waypoint, self.node_road_option = self._waypoints_buffer.popleft()
                if self._waypoints_queue:
                    self._waypoints_buffer.append(self._waypoints_queue.popleft())
                if self.distances:
                    self.distance_to_goal -= self.distances.popleft()

        # Update information
        if self._waypoints_buffer:
            self.target_waypoint, self.target_road_option = self._waypoints_buffer[0]
        self.agent_state = AgentState.NAVIGATING
        self.speed_limit = self.vehicle.get_speed_limit()
        # note: Only update RL data, vehicle behaviors are regulated by the agent. The "speed limit" is now target speed.

if __name__ == "__main__":
    pass
