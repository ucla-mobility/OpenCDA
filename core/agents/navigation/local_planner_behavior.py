# -*- coding: utf-8 -*-
""" This module contains a local planner to perform
low-level waypoint following based on PID controllers. """

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from collections import deque
from enum import Enum
import statistics
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

    def __init__(self, agent, buffer_size=5, dynamic_pid=False, debug=False, debug_trajectory=False, update_freq=15):
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
        self.update_freq = update_freq

        # lane change flag
        self.lane_change = False

        # controller param
        self.max_throttle = 1.0
        self.max_break = 1.0

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

    def set_controller_longitudinal(self, max_throttle, max_break):
        """
        Change the parameters of controller
        :return:
        """
        self.max_throttle = max_throttle
        self.max_break = max_break

    def generate_path(self):
        """
        Generate the smooth path using cubic spline
        :return: rx, ry, ryaw, rk: list of planned path points' x,y coordinates, yaw angle and curvature
        """

        # used to save all key spline node
        x = []
        y = []

        # [m] distance of each interpolated points
        ds = 0.1

        # retrieve current location, yaw angle
        current_location = self._vehicle.get_location()
        current_yaw = self._vehicle.get_transform().rotation.yaw

        # retrieve the corresponding waypoint of the current location
        current_wpt = self._map.get_waypoint(current_location).next(1)[0]
        current_wpt_loc = current_wpt.transform.location

        # retrieve the future and past waypoint to check whether a lane change is gonna operated
        future_wpt = self._waypoint_buffer[-1][0]
        previous_wpt = self._history_buffer[0][0] if len(self._history_buffer) > 0 else current_wpt

        # check lateral offset from previous waypoint to current waypoint
        vec_norm, angle = cal_distance_angle(previous_wpt.transform.location,
                                             future_wpt.transform.location,
                                             future_wpt.transform.rotation.yaw)
        # distance in the lateral direction
        lateral_diff = abs(vec_norm * math.sin(math.radians(angle - 1 if angle > 90 else angle + 1)))

        boundingbox = self._vehicle.bounding_box
        veh_width = 2 * abs(boundingbox.location.y - boundingbox.extent.y)
        lane_width = current_wpt.lane_width

        is_lateral_within_range = veh_width < lateral_diff < 2 * lane_width
        # check if the vehicle is in lane change based on lane id and lateral offset
        self.lane_change = (future_wpt.lane_id != current_wpt.lane_id
                            or previous_wpt.lane_id != future_wpt.lane_id) \
                            or is_lateral_within_range

        _, angle = cal_distance_angle(self._waypoint_buffer[0][0].transform.location, current_location, current_yaw)

        # we consider history waypoint to generate trajectory
        index = 0
        for i in range(len(self._history_buffer)):
            prev_wpt = self._history_buffer[i][0].transform.location
            _, angle = cal_distance_angle(prev_wpt, current_location, current_yaw)
            # make sure the history waypoint is already passed by
            if angle > 90 and not self.lane_change:
                x.append(prev_wpt.x)
                y.append(prev_wpt.y)
                index += 1
            if self.lane_change:
                x.append(prev_wpt.x)
                y.append(prev_wpt.y)
                index += 1

        # to make sure the vehicle is stable during lane change, we don't include any current position
        if self.lane_change:
            _, angle = cal_distance_angle(self._waypoint_buffer[0][0].transform.location,
                                          current_location, current_yaw)
            print('lane change')
            pass
        else:
            _, angle = cal_distance_angle(current_wpt_loc, current_location, current_yaw)
            # we prefer to use waypoint as the current position for path generation if the waypoint is
            # in front of us. This is because waypoint always sits in the center
            if angle < 90:
                x.append(current_wpt_loc.x)
                y.append(current_wpt_loc.y)
            else:
                x.append(current_location.x)
                y.append(current_location.y)

        # used to filter the waypoints that are too close
        prev_x = 0 if self.lane_change else x[index]
        prev_y = 0 if self.lane_change else y[index]
        for i in range(len(self._waypoint_buffer)):
            cur_x = self._waypoint_buffer[i][0].transform.location.x
            cur_y = self._waypoint_buffer[i][0].transform.location.y
            if abs(prev_x - cur_x) < 0.5 and abs(prev_y - cur_y) < 0.5:
                continue
            prev_x = cur_x
            prev_y = cur_y

            x.append(cur_x)
            y.append(cur_y)

        # Cubic Spline Interpolation calculation
        sp = Spline2D(x, y)

        diff_x = current_location.x - sp.sx.y[0]
        diff_y = current_location.y - sp.sy.y[0]
        diff_s = np.hypot(diff_x, diff_y)

        # we only need the interpolation points after current position
        s = np.arange(diff_s, sp.s[-1], ds)

        # calculate interpolation points
        rx, ry, ryaw, rk = [], [], [], []
        # we only need the interpolation points until next waypoint
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            if abs(ix - x[index]) <= ds and abs(iy - y[index]) <= ds:
                continue
            rx.append(ix)
            ry.append(iy)
            rk.append(sp.calc_curvature(i_s))
            ryaw.append(sp.calc_yaw(i_s))

        # debug purpose
        self._long_plan_debug = []
        s = np.arange(sp.s[0], sp.s[-1], ds)
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            self._long_plan_debug.append(carla.Transform(carla.Location(ix, iy, 0)))

        return rx, ry, rk, ryaw

    def generate_trajectory(self, rx, ry, rk):
        """
        Sampling the generated path and assign speed to each point
        :param rx: x coordinates of planning path
        :param ry: y coordinates of planning path
        :param rk: curvature of planning path
        :param debug: whether to draw the whole plan path
        :return:
        """
        # unit distance for interpolation points
        ds = 0.1
        # unit time space TODO: Make this dynamic to map a linear relationship with speed
        dt = 0.25

        target_speed = self._target_speed
        current_speed = get_speed(self._vehicle)

        # sample the trajectory by 0.1 second
        sample_num = 2.0 // dt

        break_flag = False
        current_speed = current_speed / 3.6
        sample_resolution = 0

        # use mean curvature to constrain the speed
        mean_k = abs(abs(statistics.mean(rk)))
        # v^2 <= a_lat_max / curvature, we assume 3.6 is the maximum lateral acceleration
        target_speed = min(target_speed, np.sqrt(7.2 / mean_k) * 3.6)
        print('current speed %f and target speed is %f' % (current_speed * 3.6, target_speed))
        # TODO: This may need to be tuned more(for instance, use history speed)
        acceleration = max(min(4.5,
                               (target_speed / 3.6 - current_speed) / dt), -3.5)

        for i in range(1, int(sample_num) + 1):
            sample_resolution += current_speed * dt + 0.5 * acceleration * dt ** 2
            current_speed += acceleration * dt

            # print(sample_resolution)
            if int(sample_resolution // ds - 1) >= len(rx):
                sample_x = rx[-1]
                sample_y = ry[-1]
                break_flag = True

            else:
                sample_x = rx[max(0, int(sample_resolution // ds - 1))]
                sample_y = ry[max(0, int(sample_resolution // ds - 1))]

            self._trajectory_buffer.append((carla.Transform(carla.Location(sample_x, sample_y,
                                                                           self._waypoint_buffer[0][
                                                                               0].transform.location.z + 0.5)),
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

    def run_step(self, rx, ry, rk, target_speed=None, trajectory=None, following=False):
        """
        Execute one step of local planning which involves
        running the longitudinal and lateral PID controllers to
        follow the smooth waypoints trajectory.

            :param rx: generated path x coordinates
            :param ry: generated path y coordinates
            :param rk: generated path curvatures
            :param following: whether the vehicle is under following status
            :param trajectory: pre-generated trajectory only for following vehicles in the platooning
            :param target_speed: desired speed
            :return: control
        """

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

        # Buffering the waypoints. Always keep the waypoint buffer alive
        if len(self._waypoint_buffer) < 5:
            for i in range(self._buffer_size):
                if self.waypoints_queue:
                    self._waypoint_buffer.append(
                        self.waypoints_queue.popleft())
                else:
                    break

        # we will generate the trajectory only if it is not a following vehicle in the platooning
        if not trajectory and len(self._trajectory_buffer) < self.update_freq and not following:
            self._trajectory_buffer.clear()
            self.generate_trajectory(rx, ry, rk)
        elif trajectory:
            self._trajectory_buffer = trajectory.copy()

        # Current vehicle waypoint
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())

        # Target waypoint
        self.target_waypoint, self.target_road_option, self._target_speed, dt = \
            self._trajectory_buffer[min(1, len(self._trajectory_buffer) - 1)]

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
                                                              args_longitudinal=args_long,
                                                              max_brake=self.max_break,
                                                              max_throttle=self.max_throttle)

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
                                  self._trajectory_buffer, z=0.1, lt=0.05)

        if self.debug:
            draw_trajetory_points(self._vehicle.get_world(),
                                  self._waypoint_buffer,
                                  z=0.1,
                                  size=0.1,
                                  color=carla.Color(0, 0, 255),
                                  lt=0.2)
            draw_trajetory_points(self._vehicle.get_world(),
                                  self._history_buffer,
                                  z=0.1,
                                  size=0.1,
                                  color=carla.Color(255, 0, 255),
                                  lt=0.2)

        return control
