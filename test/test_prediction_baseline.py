# -*- coding: utf-8 -*-
"""
Unit test for physics-based prediction baseline.
"""
# Author: Hao Xiang <haxiang@g.ucla.edu>
# License: MIT

import os
import sys
import unittest
import pdb
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
from opencda.core.sensing.prediction.physics import TrajectoryData, ConstantVelocityHeading, \
    ConstantAccelerationHeading, ConstantSpeedYawRate, ConstantManitudeAccelAndYawRate, PhysicsOracle


class TestPredictionBaseline(unittest.TestCase):
    def setUp(self):
        self.observed_traj = [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [6, 6]]
        self.observed_length = len(self.observed_traj)
        self.predict_length = 10
        self.yaw = np.pi / 4
        self.yaw_rate = 0.0
        self.dt = 0.1

        self.v = (np.array(self.observed_traj[-1]) - np.array(self.observed_traj[-2])) / self.dt
        past_v = (np.array(self.observed_traj[-2]) - np.array(self.observed_traj[-3])) / self.dt
        self.a = (self.v - past_v) / self.dt

    def test_constant_velocity_heading(self):
        model = ConstantVelocityHeading(self.observed_length, self.predict_length, self.dt)
        kinematics_data = (self.observed_traj, self.v, self.a, self.yaw, self.yaw_rate)
        preds = model(kinematics_data)

        gt = np.array(
            [[8, 8], [10, 10], [12, 12], [14, 14], [16, 16], [18, 18], [20, 20], [22, 22], [24, 24], [26, 26]])

        def dist(x, y):
            return np.linalg.norm(x - y)

        assert dist(preds, gt) < 1e-10
        return

    def test_constant_acceleration_heading(self):
        model = ConstantAccelerationHeading(self.observed_length, self.predict_length, self.dt)
        kinematics_data = (self.observed_traj, self.v, self.a, self.yaw, self.yaw_rate)
        preds = model(kinematics_data)
        gt = np.array(
            [[8, 8], [10, 10], [12, 12], [14, 14], [16, 16], [18, 18], [20, 20], [22, 22], [24, 24], [26, 26]])
        displacement = [0.5, 2, 4.5, 8, 12.5, 18, 24.5, 32, 40.5, 50.0]
        displacement = np.array([[x, x] for x in displacement])
        gt = gt + displacement

        def dist(x, y):
            return np.linalg.norm(x - y)

        assert dist(preds, gt) < 1e-10
        return

    def test_constant_speed_yaw_rate(self):
        model = ConstantSpeedYawRate(self.observed_length, self.predict_length, self.dt)
        kinematics_data = (self.observed_traj, self.v, self.a, self.yaw, self.yaw_rate)
        preds = model(kinematics_data)
        gt = np.array(
            [[8, 8], [10, 10], [12, 12], [14, 14], [16, 16], [18, 18], [20, 20], [22, 22], [24, 24], [26, 26]])

        def dist(x, y):
            return np.linalg.norm(x - y)

        assert dist(preds, gt) < 1e-10
        return

    def test_constant_magnitude_accel_and_yaw_rate(self):
        model = ConstantManitudeAccelAndYawRate(self.observed_length, self.predict_length, self.dt)
        kinematics_data = (self.observed_traj, self.v, self.a, self.yaw, self.yaw_rate)
        preds = model(kinematics_data)
        gt = np.array([[8., 8.],
                       [11., 11.],
                       [15., 15.],
                       [20., 20.],
                       [26., 26.],
                       [33., 33.],
                       [41., 41.],
                       [50., 50.],
                       [60., 60.],
                       [71., 71.]])

        def dist(x, y):
            return np.linalg.norm(x - y)

        assert dist(preds, gt) < 1e-10
        return

    def test_oracle(self):
        model = PhysicsOracle(self.observed_length, self.predict_length, self.dt)
        kinematics_data = (self.observed_traj, self.v, self.a, self.yaw, self.yaw_rate)
        gt = np.array([[8., 8.],
                       [11., 11.],
                       [15., 15.],
                       [20., 20.],
                       [26., 26.],
                       [33., 33.],
                       [41., 41.],
                       [50., 50.],
                       [60., 60.],
                       [71., 71.]])
        preds = model(kinematics_data, gt)

        def dist(x, y):
            return np.linalg.norm(x - y)

        assert dist(preds, gt) < 1e-10
        return


if __name__ == '__main__':
    unittest.main()
