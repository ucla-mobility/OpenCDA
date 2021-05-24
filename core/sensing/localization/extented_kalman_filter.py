# -*- coding: utf-8 -*-
"""
Use Extended Kalman Filter on GPS + IMU for better localization.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>, credit to Kartik Madhira <kartikmadhira1@gmail.com>
# License: MIT

import math
import numpy as np


class ExtentedKalmanFilter(object):
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

        self.R = np.diag([1.0, 2.0, 0.5]) ** 2  # Observation x,y position covariance

        self.time_step = 0.05

        self.xEst = np.zeros((4, 1))
        self.PEst = np.eye(4)

    def motion_model(self, x, u):
        """
        X_pred = FX + Bu
        :param x: previous [x, y, yaw in rad, v]
        :param u: current [velocity, imu yaw rate]
        :return:
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
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}
        so
        dx/dyaw = -v*dt*sin(yaw)
        dx/dv = dt*cos(yaw)
        dy/dyaw = v*dt*cos(yaw)
        dy/dv = dt*sin(yaw)
        """
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -self.time_step * v * math.sin(yaw), self.time_step * math.cos(yaw)],
            [0.0, 1.0, self.time_step * v * math.cos(yaw), self.time_step * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        return jF

    def observation_model(self, x):
        """
        projection from prediction to sensor measurements.
        :param x:
        :return:
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
        Initalization for states
        :param x:
        :param y:
        :param heading:
        :param velocity:
        :return:
        """
        self.xEst[0] = x
        self.xEst[1] = y
        self.xEst[2] = heading
        self.xEst[3] = velocity

    def run_step(self, x, y, heading, velocity, yaw_rate_imu):
        """
        Apply EKF on current measurement and previous prediction
        :param x: x(esu) coordinate from gnss sensor at current timestamp
        :param y: y(esu) coordinate from gnss sensor at current timestamp
        :param heading: heading direction at current timestamp
        :param velocity: current speed
        :param yaw_rate_imu: yaw rate rad/s from IMU sensor
        :return: corrected x, y, heading, velocity
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

        return self.xEst[0][0], self.xEst[1][0], self.xEst[2][0], self.xEst[3][0]
