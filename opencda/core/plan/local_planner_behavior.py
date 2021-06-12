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

from opencda.core.common.misc import distance_vehicle, draw_trajetory_points, cal_distance_angle
from opencda.core.plan.spline import Spline2D


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

    def __init__(self, agent, carla_map, config_yaml):
        """
        :param agent: agent that regulates the vehicle
        :param config_yaml: local planner configuration file
        """
        self._vehicle = agent.vehicle
        self._map = carla_map

        self._ego_pos = None
        self._ego_speed = None

        # waypoint pop out thresholding
        self._min_distance = config_yaml['min_dist']
        self._buffer_size = config_yaml['buffer_size']
        # TODO: Redudant, remove later
        self._current_waypoint = None

        # TODO: pid controller should be outside
        self._pid_controller = None

        # global route
        self.waypoints_queue = deque(maxlen=20000)
        # waypoint route
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        # trajectory buffer
        self._long_plan_debug = []
        self._trajectory_buffer = deque(maxlen=30)
        self._history_buffer = deque(maxlen=3)
        self.trajectory_update_freq = config_yaml['trajectory_update_freq']

        # used to identify whether lane change is operated
        self.lane_change = False
        # In some corner cases, the id is not changed but we regard it as lane change due to large lateral diff
        self.lane_id_change = False

        # debug option
        self.debug = config_yaml['debug']
        self.debug_trajectory = config_yaml['debug_trajectory']

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

    def update_information(self, ego_pos, ego_speed):
        """
        Update the ego position and speed for trajectory planner.
        Args:
            ego_pos (carla.Transform): Ego position from localization module.
            ego_speed (float): Ego speed(km/h) from localization module.

        Returns:

        """
        self._ego_pos = ego_pos
        self._ego_speed = ego_speed

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

    def get_trajetory(self):
        """
        Get the trajetory
        :return:
        """
        return self._trajectory_buffer

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

        # retrieve current location, yaw angle todo: this should comes from self._egopos
        current_location = self._ego_pos.location
        current_yaw = self._ego_pos.rotation.yaw

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
        self.lane_id_change = (future_wpt.lane_id != current_wpt.lane_id or previous_wpt.lane_id != future_wpt.lane_id)
        self.lane_change = self.lane_id_change or is_lateral_within_range

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
            # if the vehicle starts lane change at the very start
            if len(x) == 0 or len(y) == 0:
                x.append(current_location.x)
                y.append(current_location.y)
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
        prev_x = x[max(0, index - 1)] if self.lane_change else x[index]
        prev_y = y[max(0, index - 1)] if self.lane_change else y[index]
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
        self._long_plan_debug = []
        # we only need the interpolation points until next waypoint
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            if abs(ix - x[index]) <= ds and abs(iy - y[index]) <= ds:
                continue
            self._long_plan_debug.append(carla.Transform(carla.Location(ix, iy, 0)))
            rx.append(ix)
            ry.append(iy)
            rk.append(max(min(sp.calc_curvature(i_s), 0.2), -0.2))
            ryaw.append(sp.calc_yaw(i_s))

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
        current_speed = self._ego_speed

        # sample the trajectory by 0.1 second
        sample_num = 2.0 // dt

        break_flag = False
        current_speed = current_speed / 3.6
        sample_resolution = 0

        # use mean curvature to constrain the speed

        mean_k = 0.0001 if len(rk) < 2 else abs(statistics.mean(rk))
        # v^2 <= a_lat_max / curvature, we assume 3.6 is the maximum lateral acceleration
        target_speed = min(target_speed, np.sqrt(5.0 / (mean_k + 10e-6)) * 3.6)
        print('Vehicle Id:%d, current speed %f and target speed is %f' % (self._vehicle.id,
                                                                          current_speed * 3.6, target_speed))

        # TODO: This may need to be tuned more(for instance, use history speed to check acceleration)
        if self._pid_controller:
            max_acc = 3.5 if self._pid_controller.max_throttle >= 0.9 else 2.5
        else:
            max_acc = 3.5
        # todo: hard-coded, need to be tuned
        acceleration = max(min(max_acc,
                               (target_speed / 3.6 - current_speed) / dt), -6.5)

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
            :return: next trajectory point's target speed and waypoint
        """

        self._target_speed = target_speed

        # Buffering the waypoints. Always keep the waypoint buffer alive todo:remove the hard coded
        if len(self._waypoint_buffer) < 9:
            for i in range(self._buffer_size - len(self._waypoint_buffer)):
                if self.waypoints_queue:
                    self._waypoint_buffer.append(
                        self.waypoints_queue.popleft())
                else:
                    break

        # we will generate the trajectory only if it is not a following vehicle in the platooning
        if not trajectory and len(self._trajectory_buffer) < self.trajectory_update_freq and not following:
            self._trajectory_buffer.clear()
            self.generate_trajectory(rx, ry, rk)
        elif trajectory:
            self._trajectory_buffer = trajectory.copy()

        # Current vehicle waypoint
        self._current_waypoint = self._map.get_waypoint(self._ego_pos.location)

        # Target waypoint TODO: dt is never used
        self.target_waypoint, self.target_road_option, self._target_speed, dt = \
            self._trajectory_buffer[min(1, len(self._trajectory_buffer) - 1)]

        # Purge the queue of obsolete waypoints
        vehicle_transform = self._ego_pos
        self.pop_buffer(vehicle_transform)

        if self.debug_trajectory:
            draw_trajetory_points(self._vehicle.get_world(),
                                  self._long_plan_debug,
                                  color=carla.Color(0, 255, 0),
                                  size=0.05,
                                  lt=0.1)
            draw_trajetory_points(self._vehicle.get_world(),
                                  self._trajectory_buffer, z=0.1, lt=0.1)

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

        return self._target_speed, self.target_waypoint.transform.location \
            if hasattr(self.target_waypoint, 'is_junction') else self.target_waypoint.location
