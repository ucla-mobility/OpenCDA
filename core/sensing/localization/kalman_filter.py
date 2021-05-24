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
        # state at previous timestamp
        self.state_prev = np.zeros((3, 1))
        # covariance matrix at previous timstamp
        self.P_prev = np.identity(3)

        # time step todo: currently hardcoded
        self.time_step = 0.05

        # system error matrix
        self.Q = np.diag([
            0.20,  # variance of location on x-axis
            0.20,  # variance of location on y-axis
            np.deg2rad(1.0),  # variance of yaw angle
        ])  # predict state covariance

        # noise matrix for measurement
        self.R = np.diag([1.0, 2.0, 0.5]) ** 2  # Observation x,y position covariance

    def run_step_init(self, x, y, heading):
        """
        Initialization run step
        :param x:
        :param y:
        :param heading:
        :return:
        """
        self.state_prev[0] = x
        self.state_prev[1] = y
        self.state_prev[2] = heading

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
        # Identity transformation matrix between sensor measurement and target
        H = np.identity(3)
        z = np.array([x, y, np.deg2rad(heading)]).reshape(3, 1)

        # velocity on east and south computation
        ve = velocity * np.cos(np.rad2deg(self.state_prev[2][0]))
        vn = velocity * np.sin(np.rad2deg(self.state_prev[2][0]))

        # system input vector
        u = np.array([ve, vn, yaw_rate_imu]).reshape(3, 1)
        # control matrix
        B = np.identity(3)
        # prediction matrix
        A = np.identity(3)

        # KF algorithm begins
        state_predict = np.matmul(A, self.state_prev) + np.matmul(B, u) * self.time_step
        inovation = z - np.matmul(H, state_predict)

        if inovation[2] > 1.5 * np.pi:
            inovation[2] = inovation[2] - 2 * np.pi
        elif inovation[2] < -1.5 * np.pi:
            inovation[2] = inovation[2] + 2 * np.pi

        # practical setting
        # Q = self.Q * self.time_step + self.time_step ^ 2 / 2 * (self.Q.dot(A.transpose()) +
        #                                                         A.dot(self.Q))
        Q = self.Q
        P = reduce(np.dot, [A, self.P_prev, A.transpose()]) + Q
        inv = np.linalg.inv(reduce(np.dot, [H, P, H.transpose()]) + self.R)
        K = reduce(np.dot, [P, H.transpose(), inv])
        state_final = state_predict + K.dot(inovation)

        if state_final[2] > np.pi:
            state_final[2] = state_final[2] - np.pi
        elif state_final[2] < -np.pi:
            state_final[2] = state_final[2] + np.pi

        self.P_prev = P - reduce(np.dot, [K, H, P])
        self.state_prev = state_final

        return state_final[0][0], state_final[1][0], state_final[2][0]
