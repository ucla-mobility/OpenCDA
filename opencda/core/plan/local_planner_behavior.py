# -*- coding: utf-8 -*-
""" This module contains a local planner to perform
low-level waypoint following based on PID controllers. """

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License:  TDG-Attribution-NonCommercial-NoDistrib

from collections import deque
from enum import Enum
import statistics
import math

import carla
import numpy as np

from opencda.core.common.misc import distance_vehicle, draw_trajetory_points, \
    cal_distance_angle, compute_distance
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
    LocalPlanner implements the basic behavior of following a trajectory of
    waypoints that is generated on-the-fly.The low-level motion of the vehicle
    is computed by using lateral and longitudinal PID controllers. When
    multiple paths are available (intersections) this local planner makes
    a random choice.

    Parameters
    ----------
    agent : carla.agent
        The carla.agent that applying vehicle contorl.

    carla_map : carla.map
        The HD map of the current simulation world.

    config : dict
        The configuration dictionary of the trajectory planning module.

    Attributes
    ----------
    _vehicle : carla.vehicle
        The caral vehicle objcet.

    _ego_pos : carla.position
        The current position of the ego vehicle.

    _ego_speed : float
        The current speed of the ego vehicle.

    waypoints_queue : deque
        The waypoint deque of the current plan.

    _waypoint_buffer : deque
        A buffer deque to store waypoints of the next steps.

    _long_plan_debug : list
        A list that stores the waypoints of global plan for debug purposes.

    _trajectory_buffer : deque
        A deque buffer that stores the current trajectory.

    _history_buffer : deque
        A deque buffer that stores the trajectory history of the ego vehicle.

    potential_curved_road : boolean
        A indicator used to identify whether the road is potentially curved by using lane_id and lane's lateral change

    lane_id_change : boolean
        In some corner cases, the id is not changed but we regard it
         as lane change due to large lateral diff.

    """

    # Minimum distance to target waypoint as a percentage
    # (e.g. within 80% of total distance)

    def __init__(self, agent, carla_map, config_yaml):
        self._vehicle = agent.vehicle
        self._map = carla_map

        self._ego_pos = None
        self._ego_speed = None

        # waypoint pop out thresholding
        self._min_distance = config_yaml['min_dist']
        self._buffer_size = config_yaml['buffer_size']

        # global route
        self.waypoints_queue = deque(maxlen=20000)
        # waypoint route
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        # trajectory buffer
        self._long_plan_debug = []
        self._trajectory_buffer = deque(maxlen=30)
        self._history_buffer = deque(maxlen=3)
        self.trajectory_update_freq = config_yaml['trajectory_update_freq']
        self.waypoint_update_freq = config_yaml['waypoint_update_freq']

        # trajectory sampling rate
        self.dt = config_yaml['trajectory_dt']

        # used to identify whether road is potentially curved
        self.potential_curved_road = False
        # In some corner cases, the id is not changed but we regard it as lane
        # change due to large lateral diff
        self.lane_id_change = False
        self.lane_lateral_change = False

        # debug option
        self.debug = config_yaml['debug']
        self.debug_trajectory = config_yaml['debug_trajectory']

    def set_global_plan(self, current_plan, clean=False):
        """
        Sets new global plan.

        Parameters
        ----------
        current_plan : list
            List of waypoints in the actual plan.

        clean : boolean
            Indicator of whether to clear the global plan.

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

        Parameters
        ----------
        ego_pos : carla.Transform
            Ego position from localization module.

        ego_speed : float
            Ego speed(km/h) from localization module.

        """
        self._ego_pos = ego_pos
        self._ego_speed = ego_speed

    def get_trajectory(self):
        """
        Get the trajetory

        Returns :
        ----------
        self._trajectory_buffer : deque
            Trajectory buffer.

        """
        return self._trajectory_buffer

    def get_waypoint_buffer(self):
        """
        Get the _waypoint_buffer.

        Returns
        -------
        self._waypoint_buffer : deque
            A buffer deque to store waypoints of the next steps.

        """
        return self._waypoint_buffer

    def get_waypoints_queue(self):
        """
        Get the waypoints_queue.
        Returns
        -------
        self.waypoints_queue : deque
            The waypoint deque of the current plan.

        """
        return self.waypoints_queue

    def get_history_buffer(self):
        """
        Get the _history_buffer

        Returns
        -------
        self._history_buffer : deque
            A deque buffer that stores the trajectory history of the ego vehicle.

        """
        return self._history_buffer

    def generate_path(self):
        """
        Generate the smooth path using cubic spline.

        Returns :
        ----------
        rx : list
            List of planned path points' x coordinates.

        ry : list
            List of planned path points' y coordinates.

        ryaw : list
            List of planned path points' yaw angles.

        rk : list
            List of planned path points' curvatures.

        """

        # used to save all key spline node
        x = []
        y = []

        # pop out the waypoints that may damage driving performance
        self.buffer_filter()

        # [m] distance of each interpolated points
        ds = 0.1

        # retrieve current location, yaw angle
        current_location = self._ego_pos.location
        current_yaw = self._ego_pos.rotation.yaw

        # retrieve the corresponding waypoint of the current location
        current_wpt = self._map.get_waypoint(current_location).next(1)[0]
        current_wpt_loc = current_wpt.transform.location

        # retrieve the future and past waypoint to check whether a lane change
        # is gonna operated
        future_wpt = self._waypoint_buffer[-1][0]
        previous_wpt = self._history_buffer[0][0] if len(
            self._history_buffer) > 0 else current_wpt

        # check lateral offset from previous waypoint to current waypoint
        vec_norm, angle = cal_distance_angle(previous_wpt.transform.location,
                                             future_wpt.transform.location,
                                             future_wpt.transform.rotation.yaw)
        # distance in the lateral direction
        lateral_diff = abs(
            vec_norm *
            math.sin(
                math.radians(
                    angle - 1 if angle > 90 else angle + 1)))

        boundingbox = self._vehicle.bounding_box
        veh_width = 2 * abs(boundingbox.location.y - boundingbox.extent.y)
        lane_width = current_wpt.lane_width

        self.lane_lateral_change = veh_width < lateral_diff
        # check if the vehicle is in lane change based on lane id and lateral
        # offset
        self.lane_id_change = (
                future_wpt.lane_id != current_wpt.lane_id or
                previous_wpt.lane_id != future_wpt.lane_id)
        self.potential_curved_road = self.lane_id_change or self.lane_lateral_change

        _, angle = cal_distance_angle(
            self._waypoint_buffer[0][0].transform.location,
            current_location,
            current_yaw)

        # we consider history waypoint to generate trajectory
        index = 0
        for i in range(len(self._history_buffer)):
            prev_wpt = self._history_buffer[i][0].transform.location
            _, angle = cal_distance_angle(
                prev_wpt, current_location, current_yaw)
            # make sure the history waypoint is already passed by
            if angle > 90 and not self.potential_curved_road:
                x.append(prev_wpt.x)
                y.append(prev_wpt.y)
                index += 1
            if self.potential_curved_road:
                x.append(prev_wpt.x)
                y.append(prev_wpt.y)
                index += 1

        # to make sure the vehicle is stable during lane change, we don't
        # include any current position
        if self.potential_curved_road:
            _, angle = cal_distance_angle(
                self._waypoint_buffer[0][0].transform.location,
                current_location,
                current_yaw)
            # if the vehicle starts lane change at the very start
            if len(x) == 0 or len(y) == 0:
                x.append(current_location.x)
                y.append(current_location.y)
        else:
            _, angle = cal_distance_angle(
                current_wpt_loc, current_location, current_yaw)
            # we prefer to use waypoint as the current position for path
            # generation if the waypoint is in front of us.
            # This is because waypoint always sits in the center
            if angle < 90:
                x.append(current_wpt_loc.x)
                y.append(current_wpt_loc.y)
            else:
                x.append(current_location.x)
                y.append(current_location.y)

        # used to filter the waypoints that are too close
        index = max(0, index - 1) if self.potential_curved_road else index
        prev_x = x[index]
        prev_y = y[index]
        for i in range(len(self._waypoint_buffer)):
            cur_x = self._waypoint_buffer[i][0].transform.location.x
            cur_y = self._waypoint_buffer[i][0].transform.location.y
            if abs(prev_x - cur_x) < 0.5 and abs(prev_y - cur_y) < 0.5:
                continue
            prev_x = cur_x
            prev_y = cur_y

            x.append(cur_x)
            y.append(cur_y)

        # calculate interpolation points
        rx, ry, ryaw, rk = [], [], [], []

        # Cubic Spline Interpolation calculation
        if len(x) < 2 or len(y) < 2:
            return rx, ry, rk, ryaw

        sp = Spline2D(x, y)

        diff_x = current_location.x - sp.sx.y[0]
        diff_y = current_location.y - sp.sy.y[0]
        diff_s = np.hypot(diff_x, diff_y)

        # we only need the interpolation points after current position
        s = np.arange(diff_s, sp.s[-1], ds)

        self._long_plan_debug = []
        # we only need the interpolation points until next waypoint
        for (i, i_s) in enumerate(s):
            ix, iy = sp.calc_position(i_s)
            if abs(ix - x[index]) <= ds and abs(iy - y[index]) <= ds:
                continue
            if i <= len(s) // 2:
                self._long_plan_debug.append(
                    carla.Transform(carla.Location(ix, iy, 0)))
            rx.append(ix)
            ry.append(iy)
            rk.append(max(min(sp.calc_curvature(i_s), 0.2), -0.2))
            ryaw.append(sp.calc_yaw(i_s))

        return rx, ry, rk, ryaw

    def generate_trajectory(self, rx, ry, rk):
        """
        Sampling the generated path and assign speed to each point.

        Parameters
        ----------
        rx : list
            List of planned path points' x coordinates.

        ry : list
            List of planned path points' y coordinates.

        rk : list
            List of planned path points' curvatures.

        debug : boolean
            whether to draw the whole plan path

        """
        # unit distance for interpolation points
        ds = 0.1
        # unit sampling resolution
        dt = self.dt

        target_speed = self._target_speed
        current_speed = self._ego_speed

        # sample the trajectory by 0.1 second
        sample_num = 2.0 // dt

        break_flag = False
        current_speed = current_speed / 3.6
        sample_resolution = 0

        # use mean curvature to constrain the speed

        mean_k = 0.0001 if len(rk) < 2 else abs(statistics.mean(rk))
        # v^2 <= a_lat_max / curvature, we assume 3.6 is the maximum lateral
        # acceleration
        target_speed = min(target_speed, np.sqrt(5.0 / (mean_k + 10e-6)) * 3.6)

        max_acc = 3.5
        # todo: hard-coded, need to be tuned
        acceleration = max(
            min(max_acc, (target_speed / 3.6 - current_speed) / dt), -6.5)

        for i in range(1, int(sample_num) + 1):
            sample_resolution += current_speed * dt + \
                                 0.5 * acceleration * dt ** 2
            current_speed += acceleration * dt

            # print(sample_resolution)
            if int(sample_resolution // ds - 1) >= len(rx):
                sample_x = rx[-1]
                sample_y = ry[-1]
                break_flag = True

            else:
                sample_x = rx[max(0, int(sample_resolution // ds - 1))]
                sample_y = ry[max(0, int(sample_resolution // ds - 1))]

            self._trajectory_buffer.append(
                (carla.Transform(
                    carla.Location(
                        sample_x,
                        sample_y,
                        self._waypoint_buffer[0][0].transform.location.z +
                        0.5)),
                 target_speed))
            if break_flag:
                break

    def buffer_filter(self):
        """
        Remove the waypoints in the global route plan which has dramatic
        change of yaw angle. Such waypoint can cause bad vehicle dynnamics.
        """
        prev_wpt = None

        tmp = self._waypoint_buffer.copy()

        for i, (waypoint, _) in enumerate(tmp):

            # we only need to examine the waypoint nearby
            if i >= 3:
                break
            # we need to find the right index for origin buffer, since
            # it may remove several elements already
            j = i - (len(tmp) - len(self._waypoint_buffer))

            # check if the current waypoint is behind the vehicle.
            # if so, remove such waypoint.
            _, angle = cal_distance_angle(
                waypoint.transform.location,
                self._ego_pos.location, self._ego_pos.rotation.yaw)

            if angle > 90:
                # print('delete waypoint!')
                del self._waypoint_buffer[j]
                continue

            if prev_wpt is None:
                prev_wpt = waypoint
                continue

            # avoid the situation that the next goal state is on the
            # neighbor lane and it is too close to the current location,
            # which will cause large steering angel for lane change.
            if prev_wpt.lane_id != waypoint.lane_id and \
                    len(self._waypoint_buffer) >= 2:
                dist = compute_distance(waypoint.transform.location,
                                        prev_wpt.transform.location)

                if dist <= 4.5:
                    del self._waypoint_buffer[j]

            prev_wpt = waypoint

    def pop_buffer(self, vehicle_transform):
        """
        Remove waypoints the ego vehicle has achieved.
        
        Parameters
        ----------
        vehicle_transform : carla.position
            The position of vehicle.
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

                    if abs(
                            prev_wpt[0].transform.location.x -
                            incoming_wpt[0].transform.location.x) > 4.5 or abs(
                        prev_wpt[0].transform.location.y -
                        incoming_wpt[0].transform.location.y) > 4.5:
                        self._history_buffer.append(incoming_wpt)
                else:
                    self._history_buffer.append(
                        self._waypoint_buffer.popleft())

        if self._trajectory_buffer:
            max_index = -1
            for i, (waypoint, _,) in enumerate(self._trajectory_buffer):
                if distance_vehicle(
                        waypoint, vehicle_transform) < \
                        max(self._min_distance - 1, 1):
                    max_index = i
            if max_index >= 0:
                for i in range(max_index + 1):
                    self._trajectory_buffer.popleft()

    def run_step(
            self,
            rx,
            ry,
            rk,
            target_speed=None,
            trajectory=None,
            following=False):
        """
        Execute one step of local planning which involves
        running the longitudinal and lateral PID controllers to
        follow the smooth waypoints trajectory.

        Parameters
        ----------
        rx : list
            List of planned path points' x coordinates.

        ry : list
            List of planned path points' y coordinates.

        ryaw : list
            List of planned path points' yaw angles.

        rk : list
            List of planned path points' curvatures.

        following : boolean
            Indicator of whether the vehicle is under following status.

        trajectory : list
            Pre-generated car-following trajectory only for platoon members.

        target_speed : float
            The ego vehicle's desired speed.

        Returns
        -------
        speed : float
            Next trajectory point's target speed。

        waypoint : carla.waypoint
            Next trajectory point's waypoint.

        """

        self._target_speed = target_speed

        # Buffering the waypoints. Always keep the waypoint buffer alive
        if len(self._waypoint_buffer) < self.waypoint_update_freq:
            for i in range(self._buffer_size - len(self._waypoint_buffer)):
                if self.waypoints_queue:
                    self._waypoint_buffer.append(
                        self.waypoints_queue.popleft())
                else:
                    break

        # we will generate the trajectory only if it is not a following vehicle
        # in the platooning
        if not trajectory and len(
                self._trajectory_buffer) < self.trajectory_update_freq and \
                not following:
            self._trajectory_buffer.clear()
            # if no spline points provided, return 0 and none target wpt
            if len(rx) == 0:
                return 0, None
            self.generate_trajectory(rx, ry, rk)
        elif trajectory:
            self._trajectory_buffer = trajectory.copy()

        # Target waypoint
        self.target_waypoint, self._target_speed = \
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
            # draw_trajetory_points(self._vehicle.get_world(),
            # self._trajectory_buffer, size=0.1, arrow_size=0.2, z=0.1, lt=0.1)

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

        return self._target_speed, \
               self.target_waypoint.transform.location if hasattr(
                   self.target_waypoint,
                   'is_junction') else self.target_waypoint.location
