# -*- coding: utf-8 -*-
"""
PID Control Class
"""

# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from collections import deque

import math
import numpy as np

import carla


class Controller:
    """
    PID Controller implementation.

    Parameters
    ----------
    args : dict
        The configuration dictionary parsed from yaml file.

    Attributes
    ----------
    _lon_ebuffer : deque
        A deque buffer that stores longitudinal control errors.

    _lat_ebuffer : deque
        A deque buffer that stores latitudinal control errors.

    current_transform : carla.transform
        Current ego vehicle transformation in CARLA world.

    current_speed : float
        Current ego vehicle speed.

    past_steering : float
        Sterring angle from previous control step.

    """

    def __init__(self, args):

        # longitudinal related
        self.max_brake = args['max_brake']
        self.max_throttle = args['max_throttle']

        self._lon_k_p = args['lon']['k_p']
        self._lon_k_d = args['lon']['k_d']
        self._lon_k_i = args['lon']['k_i']

        self._lon_ebuffer = deque(maxlen=10)

        # lateral related
        self.max_steering = args['max_steering']

        self._lat_k_p = args['lat']['k_p']
        self._lat_k_d = args['lat']['k_d']
        self._lat_k_i = args['lat']['k_i']

        self._lat_ebuffer = deque(maxlen=10)

        # simulation time-step
        self.dt = args['dt']

        # current speed and localization retrieved from sensing layer
        self.current_transform = None
        self.current_speed = 0.
        # past steering
        self.past_steering = 0.

        self.dynamic = args['dynamic']

    def dynamic_pid(self):
        """
        Compute kp, kd, ki based on current speed.
        """
        pass

    def update_info(self, ego_pos, ego_spd):
        """
        Update ego position and speed to controller.

        Parameters
        ----------
        ego_pos : carla.location
            Position of the ego vehicle.

        ego_spd : float
            Speed of the ego vehicle

        Returns
        -------

        """

        self.current_transform = ego_pos
        self.current_speed = ego_spd
        if self.dynamic:
            self.dynamic_pid()

    def lon_run_step(self, target_speed):
        """

        Parameters
        ----------
        target_speed : float
            Target speed of the ego vehicle.

        Returns
        -------
        acceleration : float
            Desired acceleration value for the current step
            to achieve target speed.

        """
        error = target_speed - self.current_speed
        self._lat_ebuffer.append(error)

        if len(self._lat_ebuffer) >= 2:
            _de = (self._lat_ebuffer[-1] - self._lat_ebuffer[-2]) / self.dt
            _ie = sum(self._lat_ebuffer) * self.dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._lat_k_p * error) +
                       (self._lat_k_d * _de) +
                       (self._lat_k_i * _ie),
                       -1.0, 1.0)

    def lat_run_step(self, target_location):
        """
        Generate the throttle command based on current speed and target speed

        Parameters
        ----------
        target_location : carla.location
            Target location.

        Returns
        -------
        current_steering : float
        Desired steering angle value for the current step to
        achieve target location.

        """
        v_begin = self.current_transform.location
        v_end = v_begin + carla.Location(
            x=math.cos(
                math.radians(
                    self.current_transform.rotation.yaw)), y=math.sin(
                math.radians(
                    self.current_transform.rotation.yaw)))
        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([target_location.x -
                          v_begin.x, target_location.y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(
            w_vec, v_vec) / (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)),
                                 -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)

        if _cross[2] < 0:
            _dot *= -1.0

        self._lon_ebuffer.append(_dot)
        if len(self._lon_ebuffer) >= 2:
            _de = (self._lon_ebuffer[-1] - self._lon_ebuffer[-2]) / self.dt
            _ie = sum(self._lon_ebuffer) * self.dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._lat_k_p * _dot) + (self._lat_k_d *
                       _de) + (self._lat_k_i * _ie), -1.0, 1.0)

    def run_step(self, target_speed, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint at a given target_speed.

        Parameters
        ----------
        target_speed : float
            Target speed of the ego vehicle.

        waypoint : carla.loaction
            Target location.

        Returns
        -------
        control : carla.VehicleControl
            Desired vehicle control command for the current step.

        """
        # control class for carla vehicle
        control = carla.VehicleControl()

        # emergency stop
        if target_speed == 0 or waypoint is None:
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            return control

        acceleration = self.lon_run_step(target_speed)
        current_steering = self.lat_run_step(waypoint)

        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throttle)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        # Steering regulation: changes cannot happen abruptly, can't steer too
        # much.
        if current_steering > self.past_steering + 0.2:
            current_steering = self.past_steering + 0.2
        elif current_steering < self.past_steering - 0.2:
            current_steering = self.past_steering - 0.2

        if current_steering >= 0:
            steering = min(self.max_steering, current_steering)
        else:
            steering = max(-self.max_steering, current_steering)

        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = steering
        return control