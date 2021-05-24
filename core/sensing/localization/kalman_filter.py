# -*- coding: utf-8 -*-
"""
Use Kalman Filter on GPS + IMU for better localization.
Reference: https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>, Xin Xia<x35xia@g.ucla.edu>
# License: MIT

import math
import numpy as np

from functools import reduce


class KalmanFilter(object):
    """
    Kalman Filter implementation for gps + imu
    """

    def __init__(self):
        """
        Construct class
        """

        self.Q = np.diag([
            0.1,  # variance of location on x-axis
            0.2,  # variance of location on y-axis
            np.deg2rad(0.1),  # variance of yaw angle
            0.001  # variance of velocity
        ]) ** 2  # predict state covariance

        self.R = np.diag([1.0, 2.0, 0.1]) ** 2  # Observation x,y position covariance

        self.time_step = 0.05

        self.xEst = np.zeros((4, 1))
        self.PEst = np.eye(4)

    def motion_model(self, x, u):
        """
        Predict current position and yaw based on previous result.
        X = F * X_prev + B * u
        Args:
            x (np.array): [x_prev, y_prev, yaw_prev, v_prev], shape: (4, 1).
            u (np.array): [v_current, imu_yaw_rate], shape:(2, 1).

        Returns:
          np.array: predicted state.
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

    def observation_model(self, x):
        """
        Project the state matrix to sensor measurement matrix.
        Args:
            x (np.array): [x, y, yaw, v], shape: (4. 1).

        Returns:
            np.array: predicted measurement.

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
        Initial state filling.
        Args:
            x ():
            y ():
            heading ():
            velocity ():

        Returns:

        """
        self.xEst[0] = x
        self.xEst[1] = y
        self.xEst[2] = heading
        self.xEst[3] = velocity

    def run_step(self, x, y, heading, velocity, yaw_rate_imu):
        """
        Apply KF on current measurement and previous prediction
        :param x: x(esu) coordinate from gnss sensor at current timestamp
        :param y: y(esu) coordinate from gnss sensor at current timestamp
        :param heading: heading direction at current timestamp
        :param velocity: current speed
        :param yaw_rate_imu: yaw rate rad/s from IMU sensor
        :return: corrected x, y, heading
        """
        # gps observation
        z = np.array([x, y, heading]).reshape(3, 1)
        # velocity and imu yaw rate
        u = np.array([velocity, yaw_rate_imu]).reshape(2, 1)

        # state prediction
        xPred = self.motion_model(self.xEst, u)
        # sensor measurement prediction
        zPred = self.observation_model(xPred)
        y = z - zPred

        # projection matrix
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]
        ])

        # prediction matrix
        F = np.array([[1.0, 0, 0, 0],
                      [0, 1.0, 0, 0],
                      [0, 0, 1.0, 0],
                      [0, 0, 0, 0]])

        PPred = F @ self.PEst @ F.T + self.Q
        S = np.linalg.inv(H @ PPred @ H.T + self.R)
        K = PPred @ H.T @ S

        self.xEst = xPred + K @ y
        self.PEst = K @ H @ PPred

        return self.xEst[0][0], self.xEst[1][0], self.xEst[2][0], self.xEst[3][0]
