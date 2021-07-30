# -*- coding: utf-8 -*-
"""
Use Extended Kalman Filter on GPS + IMU for better localization.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import numpy as np


class ExtentedKalmanFilter(object):
    """
    Extended Kalman Filter implementation for gps and imu.

    Parameters
    -dt : float
        The step time for kalman filter calculation.

    Attributes
    -Q : numpy.array
        predict state covariance.
    -R : numpy.array
        Observation x,y position covariance.
    -time_step : float
        The step time for kalman filter calculation.
    -xEst : numpy.array
        Estimated x values.
    -PEst : numpy.array
        The estimated P values.
    """

    def __init__(self, dt):

        self.Q = np.diag([
            0.2,  # variance of location on x-axis
            0.2,  # variance of location on y-axis
            np.deg2rad(0.1),  # variance of yaw angle
            0.001  # variance of velocity
        ]) ** 2  # predict state covariance

        self.R = np.diag([0.5, 0.5, 0.2]) ** 2

        self.time_step = dt

        self.xEst = np.zeros((4, 1))
        self.PEst = np.eye(4)

    def motion_model(self, x, u):
        """
        Predict current position and yaw based on previous result
        (X = F * X_prev + B * u).

        Args:
            -x (np.array): [x_prev, y_prev, yaw_prev, v_prev], shape: (4, 1).
            -u (np.array): [v_current, imu_yaw_rate], shape:(2, 1).

        Returns:
          x (np.array): predicted state.
        """
        F = np.array([[1.0, 0, 0, 0],
                      [0, 1.0, 0, 0],
                      [0, 0, 1.0, 0],
                      [0, 0, 0, 0]])

        B = np.array([[self.time_step * math.cos(x[2, 0]), 0],
                      [self.time_step * math.sin(x[2, 0]), 0],
                      [0.0, self.time_step],
                      [1.0, 0.0]])

        x = F @ x + B @ u

        return x

    def jacob_f(self, x, u):
        """
        Jacobian of Motion Model motion model

        Args:
            -x (np.array): Input X array.
        Returns:
            -jF (np.array):  Jacobian of Motion Model motion model.

        """
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -self.time_step * v * math.sin(yaw),
             self.time_step * math.cos(yaw)],
            [0.0, 1.0, self.time_step * v * math.cos(yaw),
             self.time_step * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        return jF

    def observation_model(self, x):
        """
        Project the state.array to sensor measurement.array.

        Args:
            -x (np.array): [x, y, yaw, v], shape: (4. 1).

        Returns:
            -z (np.array): predicted measurement.

        """
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]
        ])

        z = H @ x

        return z

    def run_step_init(self, x, y, heading, velocity):
        """
        Initalization for states.

        Args:
            -x (float): The X coordinate.
            -y (float): Tehe y coordinate.
            -heading (float): The heading direction.
            -velocity (float): The velocity.

        """
        self.xEst[0] = x
        self.xEst[1] = y
        self.xEst[2] = heading
        self.xEst[3] = velocity

    def run_step(self, x, y, heading, velocity, yaw_rate_imu):
        """
        Apply EKF on current measurement and previous prediction.

        Args:
            -x (float): x(esu) coordinate from
             gnss sensor at current timestamp.
            -y (float): y(esu) coordinate from
             gnss sensor at current timestamp.
            -heading (float): heading direction at current timestamp.
            -velocity (float): current speed.
            -yaw_rate_imu (float): yaw rate rad/s from IMU sensor.
        Returns:
            - xEST (np.array): The corrected x, y, heading,
              and velocity information.
        """

        # gps observation
        z = np.array([x, y, heading]).reshape(3, 1)
        # velocity and imu yaw rate
        u = np.array([velocity, yaw_rate_imu]).reshape(2, 1)

        # EKF starts
        xPred = self.motion_model(self.xEst, u)
        jF = self.jacob_f(self.xEst, u)
        PPred = jF @ self.PEst @ jF.T + self.Q

        # Jacobian of Observation Model
        jH = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]])
        zPred = self.observation_model(xPred)
        y = z - zPred
        S = jH @ PPred @ jH.T + self.R
        K = PPred @ jH.T @ np.linalg.inv(S)
        self.xEst = xPred + K @ y
        self.PEst = (np.eye(len(self.xEst)) - K @ jH) @ PPred

        return self.xEst[0][0], \
            self.xEst[1][0], \
            self.xEst[2][0], \
            self.xEst[3][0]
