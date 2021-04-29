# -*- coding: utf-8 -*-
"""
PID Control Class
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import math
import numpy as np

from collections import deque

import carla

from core.common.misc import get_speed


class VehiclePIDController:
    """
    VehiclePIDController is the combination of two PID controllers
    (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """

    def __init__(self, args):
        """
        Construct class
        :param args: the parameters for pid controller
        """

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
        Compute kp, kd, ki based on current speed
        :return:
        """
        pass

    def update_info(self, vehicle):
        """
        TODO: Use Localization module later
        :param vehicle:
        :return:
        """
        self.current_transform = vehicle.get_transform()
        self.current_speed = get_speed(vehicle)
        self.past_steering = vehicle.get_control().steer
        if self.dynamic:
            self.dynamic_pid()

    def lon_run_step(self, target_speed):
        """
        Generate the throttle command based on current speed and target speed
        :param target_speed: target speed in km/h
        :return: throttle scalar
        """
        error = target_speed - self.current_speed
        self._lat_ebuffer.append(error)

        if len(self._lat_ebuffer) >= 2:
            _de = (self._lat_ebuffer[-1] - self._lat_ebuffer[-2]) / self.dt
            _ie = sum(self._lat_ebuffer) * self.dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._lat_k_p * error) + (self._lat_k_d * _de) + (self._lat_k_i * _ie), -1.0, 1.0)

    def lat_run_step(self, target_location):
        """
        Generate the throttle command based on current speed and target speed
        :param target_location: target waypoint
        :return: steering scalar
        """
        v_begin = self.current_transform.location
        v_end = v_begin + carla.Location(x=math.cos(math.radians(self.current_transform.rotation.yaw)),
                                         y=math.sin(math.radians(self.current_transform.rotation.yaw)))
        v_vec = np.array([v_end.x - v_begin.x, v_end.y - v_begin.y, 0.0])
        w_vec = np.array([target_location.x -
                          v_begin.x, target_location.y -
                          v_begin.y, 0.0])
        _dot = math.acos(np.clip(np.dot(w_vec, v_vec) /
                                 (np.linalg.norm(w_vec) * np.linalg.norm(v_vec)), -1.0, 1.0))
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

        return np.clip((self._lat_k_p * _dot) + (self._lat_k_d * _de) + (self._lat_k_i * _ie), -1.0, 1.0)

    def run_step(self, target_speed, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint
        at a given target_speed.

            :param target_speed: desired vehicle speed
            :param waypoint: target location encoded as a waypoint
            :return: control command
        """

        acceleration = self.lon_run_step(target_speed)
        current_steering = self.lat_run_step(waypoint)

        # control class for carla vehicle
        control = carla.VehicleControl()

        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throttle)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        # Steering regulation: changes cannot happen abruptly, can't steer too much.
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
