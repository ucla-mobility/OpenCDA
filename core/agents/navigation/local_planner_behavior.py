#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains a local planner to perform
low-level waypoint following based on PID controllers. """

from collections import deque
from enum import Enum
import math

import carla
import numpy as np

from core.agents.tools.misc import distance_vehicle, draw_trajetory_points, get_speed, cal_distance_angle
from core.agents.navigation.spline import Spline2D
from customize.controller import compute_pid, CustomizedVehiclePIDController


class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations
    when moving from a segment of lane to other.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6


class LocalPlanner(object):
    """
    LocalPlanner implements the basic behavior of following a trajectory
    of waypoints that is generated on-the-fly.
    The low-level motion of the vehicle is computed by using two PID controllers,
    one is used for the lateral control
    and the other for the longitudinal control (cruise speed).

    When multiple paths are available (intersections)
    this local planner makes a random choice.
    """

    # Minimum distance to target waypoint as a percentage
    # (e.g. within 80% of total distance)

    # FPS used for dt
    FPS = 10

    def __init__(self, agent, buffer_size=5, dynamic_pid=False, debug=False, debug_trajectory=False):
        """
        :param agent: agent that regulates the vehicle
        :param buffer_size: the buffer size for waypoint
        :param dynamic_pid: all pid parameters are dynamic based on surroundings,
        which will require customized function supplied to compute
        """
        # ego _vehicle
        self._vehicle = agent.vehicle
        # leading vehicle
        self._frontal_vehicle = agent.frontal_vehicle
        self._map = agent.vehicle.get_world().get_map()

        self.sampling_radius = None
        self.target_waypoint = None
        self.target_road_option = None

        self._min_distance = None
        self._current_waypoint = None
        self._target_speed = None
        self._next_waypoints = None
        self._vehicle_controller = None
        self._global_plan = None
        self._pid_controller = None

        # global route
        self.waypoints_queue = deque(maxlen=20000)
        self._buffer_size = buffer_size
        # local route
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        # platooning following route
        self._following_buffer = deque(maxlen=1000)

        self._init_controller()
        self.dynamic_pid = dynamic_pid

        # trajectory point buffer
        self._long_plan_debug = []
        self._trajectory_buffer = deque(maxlen=30)
        self._history_buffer = deque(maxlen=3)
        # old positions 
        self._history_wpts = deque(maxlen=6)
        # save whole trajetory for car following
        # self._trajectory_complete_buffer = deque(maxlen=30)
        # debug option
        self.debug = debug
        self.debug_trajectory = debug_trajectory

    def reset_vehicle(self):
        """Reset the ego-vehicle"""
        self._vehicle = None
        print("Resetting ego-vehicle!")

    def _init_controller(self):
        """
        Controller initialization.

        dt -- time difference between physics control in seconds.
        This is can be fixed from server side
        using the arguments -benchmark -fps=F, since dt = 1/F

        target_speed -- desired cruise speed in km/h

        min_distance -- minimum distance to remove waypoint from queue

        lateral_dict -- dictionary of arguments to setup the lateral PID controller
                            {'K_P':, 'K_D':, 'K_I':, 'dt'}

        longitudinal_dict -- dictionary of arguments to setup the longitudinal PID controller
                            {'K_P':, 'K_D':, 'K_I':, 'dt'}
        """
        # Default parameters
        self.args_lat_hw_dict = {
            'K_P': 0.75,
            'K_D': 0.02,
            'K_I': 0.4,
            'dt': 1.0 / self.FPS}
        self.args_lat_city_dict = {
            'K_P': 0.58,
            'K_D': 0.02,
            'K_I': 0.5,
            'dt': 1.0 / self.FPS}
        self.args_long_hw_dict = {
            'K_P': 0.37,
            'K_D': 0.024,
            'K_I': 0.032,
            'dt': 1.0 / self.FPS}
        self.args_long_city_dict = {
            'K_P': 0.15,
            'K_D': 0.05,
            'K_I': 0.07,
            'dt': 1.0 / self.FPS}
        # tune PID 
        # reset PID when change lane 
        # self.args_lat_hw_dict = {
        #     'K_P': 0.75-0.28,
        #     'K_D': 0.02,
        #     'K_I': 0.4-0.1,
        #     'dt': 1.0 / self.FPS}

        # self.args_long_hw_dict = {
        #     'K_P': 0.37-0.13,
        #     'K_D': 0.024,
        #     'K_I': 0.032,
        #     'dt': 1.0 / self.FPS}

        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())

        self._global_plan = False

        self._target_speed = self._vehicle.get_speed_limit()

        self._min_distance = 3

    def set_speed(self, speed):
        """
        Request new target speed.

            :param speed: new target speed in km/h
        """

        self._target_speed = speed

    def set_global_plan(self, current_plan, clean=False):
        """
        Sets new global plan.

            :param clean:
            :param current_plan: list of waypoints in the actual plan
        """
        for elem in current_plan:
            self.waypoints_queue.append(elem)

        if clean:
            self._waypoint_buffer.clear()
            for _ in range(self._buffer_size):
                if self.waypoints_queue:
                    self._waypoint_buffer.append(
                        self.waypoints_queue.popleft())
                else:
                    break

        self._global_plan = True

    def get_incoming_waypoint_and_direction(self, steps=3):
        """
        Returns direction and waypoint at a distance ahead defined by the user.

            :param steps: number of steps to get the incoming waypoint.
        """
        if len(self.waypoints_queue) > steps:
            return self.waypoints_queue[steps]

        else:
            try:
                wpt, direction = self.waypoints_queue[-1]
                return wpt, direction
            except IndexError as i:
                print(i)
                return None, RoadOption.VOID
        return None, RoadOption.VOID

    def get_trajetory(self):
        """
        Get the trajetory
        :return:
        """
        return self._trajectory_buffer

    def generate_trajectory(self, debug=True):
        """
        Generate a smooth trajectory using spline fitting
        :return:
        """
        x = []
        y = []

        # [m] distance of each intepolated points
        ds = 0.1
        # unit time space
        dt = 0.2

        target_speed = self._target_speed

        current_speed = get_speed(self._vehicle)
        current_location = self._vehicle.get_location()
        current_yaw = self._vehicle.get_transform().rotation.yaw
        current_wpt = self._map.get_waypoint(current_location).next(1)[0].transform.location
        current_waypoint = self._map.get_waypoint(current_location)
        future_wpt = self._waypoint_buffer[-1][0]
        previous_wpts = self._history_buffer[0][0] if len(self._history_buffer)>0 else current_waypoint

        # lateral position change
        vec_norm, angle = cal_distance_angle(previous_wpts.transform.location, current_location, current_yaw)
        lateral_diff = abs(vec_norm*math.sin(math.radians(angle)))
        # vehicle boundary 
        boundingbox = self._vehicle.bounding_box
        # vehicle width
        veh_width = 2*abs(boundingbox.location.y - boundingbox.extent.y)  
        # lane width 
        lane_width = current_waypoint.lane_width

        # check lane ID (close steps) 
        is_lanechange = (future_wpt.lane_id != current_waypoint.lane_id)
        # chekc lane ID (lanechange transition steps) 
        is_near_lanechange = (previous_wpts.lane_id != future_wpt.lane_id)
        # check the lateral shift range
        is_lateral_within_range = (veh_width < lateral_diff < (2*lane_width-veh_width))
        # print('The lateral distance: ' + str(is_lateral_within_range))

        index = 0
        for i in range(len(self._history_buffer)):
            prev_wpt = self._history_buffer[i][0].transform.location
            _, angle = cal_distance_angle(prev_wpt, current_location, current_yaw)
            # use older history for lane change 
            if angle > 90 and is_lanechange:
                # print('First of history buffer: ' + str(self._history_buffer[0][0].transform.location))
                x.append(prev_wpt.x)
                y.append(prev_wpt.y)
                index += 1

        # lane change 
        if (is_lanechange or is_near_lanechange): # and (is_lateral_within_range): 
            # change PID 
            print('lane change mode!')
            self.args_lat_hw_dict = {
                'K_P': 0.75-0.32,
                'K_D': 0.02,
                'K_I': 0.4-0.32,
                'dt': 1.0 / self.FPS}

            self.args_long_hw_dict = {
                'K_P': 0.37-0.15,
                'K_D': 0.024,
                'K_I': 0.032-0.01,
                'dt': 1.0 / self.FPS}
            # use veh position instead of waypoints 
            x.append(current_location.x)
            y.append(current_location.y)
        # not lane change
        else: 
            _, angle = cal_distance_angle(current_wpt, current_location, current_yaw)
            if angle < 90:
                # print('current way point is: %f, %f' % (current_wpt.x, current_wpt.y))
                x.append(current_wpt.x)
                y.append(current_wpt.y)
            else:
                # print('current point is: %f, %f' % (current_wpt.x, current_wpt.y))
                x.append(current_location.x)
                y.append(current_location.y)
            
        # used to filter the duplicate points
        prev_x = x[index]
        prev_y = y[index]
        # more waypoints will lead to a more optimized planning path
        for i in range(len(self._waypoint_buffer)):
            cur_x = self._waypoint_buffer[i][0].transform.location.x
            cur_y = self._waypoint_buffer[i][0].transform.location.y
            if abs(prev_x - cur_x) < 0.5 and abs(prev_y - cur_y) < 0.5:
                continue
            prev_x = cur_x
            prev_y = cur_y

            x.append(cur_x)
            y.append(cur_y)

        sp = Spline2D(x, y)
        # s = np.arange(sp.s[index], sp.s[-1], ds) 
        if index >= 2:
            s = np.arange(sp.s[index-2], sp.s[-1], ds) # calculate from previous location
        else:
            s = np.arange(sp.s[index], sp.s[-1], ds) # calculate from current location 
        # calculate interpolation points
        rx, ry, ryaw, rk = [], [], [], []
        # we only need the interpolation points until next waypoint
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            if abs(ix - x[index]) <= ds and abs(iy - y[index]) <= ds:
                continue
            # if abs(ix - x[1]) <= ds and abs(iy - y[1]) <= ds:
            #     break
            rx.append(ix)
            ry.append(iy)

        # debug purpose
        if debug:
            self._long_plan_debug = []
            for i_s in s:
                ix, iy = sp.calc_position(i_s)
                self._long_plan_debug.append(carla.Transform(carla.Location(ix, iy, 0)))

        # sample the trajectory by 0.1 second
        # sample_resolution = (current_speed + target_speed) / 2 / 3.6 * dt
        sample_num = 2.0 // dt

        if sample_num == 0 or len(rx) == 0:
            print('no trajectory')
            self._trajectory_buffer.append((self._waypoint_buffer[0][0],
                                            self._waypoint_buffer[0][1],
                                            target_speed))
        else:
            break_flag = False
            current_speed = current_speed / 3.6
            for i in range(1, int(sample_num) + 1):
                acceleration = min(0.75,
                                   (target_speed / 3.6 - current_speed) / dt)
                # sample_resolution = current_speed * dt + 0.5 * acceleration * dt ** 2
                current_speed += acceleration * dt
                sample_resolution = current_speed * dt + 0.5 * acceleration * dt ** 2

                # print(sample_resolution)
                if int(i * sample_resolution // ds - 1) >= len(rx):
                    sample_x = rx[-1]
                    sample_y = ry[-1]
                    break_flag = True

                else:
                    sample_x = rx[int(i * sample_resolution // ds - 1)]
                    sample_y = ry[int(i * sample_resolution // ds - 1)]

                # check if sampled coordinates are in the past 
                current_z = self._waypoint_buffer[0][0].transform.location.z
                sample_transform = carla.Transform(carla.Location(sample_x, sample_y, current_z + 0.5))
                # _, angle = cal_distance_angle(sample_transform.location, current_location, current_yaw)

                if len(self._history_buffer)>0: 
                    start_location = self._history_buffer[0][0].transform.location
                else: 
                    start_location = current_location
                x_in_range = (start_location.x <= sample_x <= current_location.x) or (current_location.x <= sample_x <= start_location.x)
                y_in_range = (start_location.y <= sample_y <= current_location.y) or (current_location.y <= sample_y <= start_location.y) 

                if (not x_in_range and not y_in_range):
                    self._trajectory_buffer.append((sample_transform,
                                                    self._waypoint_buffer[0][1],
                                                    target_speed,
                                                    i * dt))
                else: 
                    self._trajectory_buffer.append((self._waypoint_buffer[0][0],
                                            self._waypoint_buffer[0][1],
                                            target_speed,
                                            i * dt))
                if break_flag:
                    break

    def pop_buffer(self, vehicle_transform):
        """
        Remove waypoints achieved
        :return:
        """
        max_index = -1

        for i, (waypoint, _) in enumerate(self._waypoint_buffer):
            if distance_vehicle(
                    waypoint, vehicle_transform) < self._min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                if self._history_buffer:
                    prev_wpt = self._history_buffer[-1]
                    incoming_wpt = self._waypoint_buffer.popleft()

                    if abs(prev_wpt[0].transform.location.x - incoming_wpt[0].transform.location.x) > 0.5 or \
                            abs(prev_wpt[0].transform.location.y - incoming_wpt[0].transform.location.y) > 0.5:
                        self._history_buffer.append(incoming_wpt)
                else:
                    self._history_buffer.append(self._waypoint_buffer.popleft())

        if self._following_buffer:
            max_index = -1
            for i, (waypoint, _) in enumerate(self._following_buffer):
                if distance_vehicle(
                        waypoint, vehicle_transform) < self._min_distance:
                    max_index = i
            if max_index >= 0:
                for i in range(max_index + 1):
                    self._following_buffer.popleft()

        if self._trajectory_buffer:
            max_index = -1
            for i, (waypoint, _, _, _) in enumerate(self._trajectory_buffer):
                if distance_vehicle(
                        waypoint, vehicle_transform) < max(self._min_distance - 1, 1):
                    max_index = i
            if max_index >= 0:
                for i in range(max_index + 1):
                    self._trajectory_buffer.popleft()
            # print('Trajectory buffer size : %d' % len(self._trajectory_buffer))

    def run_step(self, target_speed=None, target_waypoint=None, target_road_option=None,
                 trajectory=None, following=False):
        """
        Execute one step of local planning which involves
        running the longitudinal and lateral PID controllers to
        follow the smooth waypoints trajectory.

            :param following:
            :param trajectory:
            :param target_road_option:
            :param target_waypoint:
            :param target_speed: desired speed
            :return: control
        """

        # record history position
        if self._waypoint_buffer[-1] not in self._history_wpts:
            # add history trajectory points
            self._history_wpts.append(self._waypoint_buffer[-1])

        if target_speed is not None:
            self._target_speed = target_speed
        else:
            self._target_speed = self._vehicle.get_speed_limit()

        if len(self.waypoints_queue) == 0:
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            control.manual_gear_shift = False
            return control

        # Buffering the waypoints. Always keep the waypoint buffer alive in case of dissolving
        if len(self._waypoint_buffer) < 4:
            for i in range(self._buffer_size):
                if self.waypoints_queue:
                    self._waypoint_buffer.append(
                        self.waypoints_queue.popleft())
                else:
                    break
            # self._trajectory_buffer.clear()
            # self.generate_trajectory(self.debug_trajectory)

        # trajectory generation
        if not trajectory and len(self._trajectory_buffer) < 15 and not following:
            self._trajectory_buffer.clear()
            self.generate_trajectory(self.debug_trajectory)
        elif trajectory:
            self._trajectory_buffer = trajectory.copy()
            # self._trajectory_complete_buffer = trajectory.copy()

        # Current vehicle waypoint
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())

        # Target waypoint
        if not target_waypoint or not target_road_option:
            self.target_waypoint, self.target_road_option, self._target_speed, dt = self._trajectory_buffer[0]
        else:
            self._following_buffer.append((target_waypoint, target_road_option))
            self.target_waypoint, self.target_road_option = self._following_buffer[0]

        if self.dynamic_pid:
            args_lat, args_long = compute_pid(self)

        elif self._target_speed > 50:
            args_lat = self.args_lat_hw_dict
            args_long = self.args_long_hw_dict

        else:
            args_lat = self.args_lat_city_dict
            args_long = self.args_long_city_dict

        self._pid_controller = CustomizedVehiclePIDController(self._vehicle,
                                                              args_lateral=args_lat,
                                                              args_longitudinal=args_long)

        control = self._pid_controller.run_step(self._target_speed, self.target_waypoint.transform.location
        if hasattr(self.target_waypoint, 'is_junction')
        else self.target_waypoint.location)

        # Purge the queue of obsolete waypoints
        vehicle_transform = self._vehicle.get_transform()
        self.pop_buffer(vehicle_transform)

        if self.debug_trajectory:
            draw_trajetory_points(self._vehicle.get_world(),
                                  self._long_plan_debug,
                                  color=carla.Color(0, 255, 0),
                                  size=0.05,
                                  lt=0.2)
            draw_trajetory_points(self._vehicle.get_world(),
                                  self._trajectory_buffer, z=0.1, lt=0.1)


        if self.debug:
            draw_trajetory_points(self._vehicle.get_world(),
                                  # self._waypoint_buffer,
                                  self._history_buffer,
                                  z=0.1,
                                  size=0.1,
                                  color=carla.Color(0, 0, 255),
                                  lt=0.3)
            # draw_trajetory_points(self._vehicle.get_world(),
            #                       self._history_wpts,
            #                       z=0.1,
            #                       size=0.1,
            #                       color=carla.Color(255, 0, 0),
            #                       lt=0.2)
            # if len(self._history_wpts)>=2:
            #     wpt_t = self._history_wpts[-3][0].transform
            #     self._vehicle.get_world().debug.draw_point(wpt_t.location + carla.Location(0.25), 0.1, carla.Color(255, 0, 0), 0.2, False)

        return control
