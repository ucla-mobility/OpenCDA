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

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
from opencda.core.sensing.prediction.physics import TrajectoryData, ConstantVelocityHeading


class TestPredictionBaseline(unittest.TestCase):
    def setUp(self):
        self.observed_traj = np.array([[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5]])
        self.observed_length = len(self.observed_traj)
        self.predict_length = 10
        self.yaw = np.pi / 4
        self.dt = 0.1

        def norm(x):
            return np.sqrt(x[0] ** 2 + x[1] ** 2)

        self.v = norm(self.observed_traj[-1, :] - self.observed_traj[-2, :]) / self.dt

    def test_constant_velocity_heading(self):
        model = ConstantVelocityHeading(self.observed_length, self.predict_length, self.dt)
        preds = model(self.observed_traj, self.v, self.yaw)
        gt = np.array([[6, 6], [7, 7], [8, 8], [9, 9], [10, 10], [11, 11, ], [12, 12], [13, 13], [14, 14], [15, 15]])

        def dist(x, y):
            return np.linalg.norm(x - y)

        return dist(preds, gt) < 1e-10


if __name__ == '__main__':
    unittest.main()
