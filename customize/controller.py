# -*- coding: utf-8 -*-

"""Customized class to replace the default controlling algorithm
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from core.actuation.pid_controller import VehiclePIDController, PIDLongitudinalController, PIDLateralController


def compute_pid(local_planner):
    """
    Compute K_D, K_I, K_P and dt dynamically based on the surroundings
    :param local_planner:
    :return: a dictionary that contains PID parameters
    """
    args_dynamic_lat = dict()
    args_dynamic_lon = dict()
    return args_dynamic_lat, args_dynamic_lon


class CustomizedVehiclePIDController(VehiclePIDController):
    """
    Customized Vehicle PIDController. Implement your algorithm to replace the naive one
    """

    def __init__(self, vehicle, args_lateral, args_longitudinal,
                 max_throttle=1.0, max_brake=1.0, max_steering=0.8):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller
        using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal
        PID controller using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        """
        super(CustomizedVehiclePIDController, self).__init__(vehicle, args_lateral, args_longitudinal,
                                                             max_throttle, max_brake, max_steering)
        self._lon_controller = CustomizedPIDLongitudinalController(self._vehicle, **args_longitudinal)
        self._lat_controller = CustomizedPIDLateralController(self._vehicle, **args_lateral)


class CustomizedPIDLongitudinalController(PIDLongitudinalController):
    """
    Customized PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        super(CustomizedPIDLongitudinalController, self).__init__(vehicle, K_P, K_D, K_I, dt)


class CustomizedPIDLateralController(PIDLateralController):
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_D=0.0, K_I=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        super(CustomizedPIDLateralController, self).__init__(vehicle, K_P, K_D, K_I, dt)
