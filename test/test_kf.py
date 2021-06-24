# -*- coding: utf-8 -*-
"""
Unit test for Kalman Filter
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import os
import sys
import unittest

import numpy as np

# temporary solution for relative imports in case opencda is not installed
# if opencda is installed, no need to use the following line
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

import mocked_carla as mcarla
from opencda.core.sensing.localization.kalman_filter import KalmanFilter
from opencda.core.sensing.localization.coordinate_transform import geo_to_transform


class testKalmanFilter(unittest.TestCase):
    def setUp(self):
        self.dt = 0.25
        self.kf = KalmanFilter(self.dt)
        self.kf.run_step_init(10, 10, 90, 20)

    def test_parameters(self):
        assert (hasattr(self.kf, 'Q') and
                self.kf.Q.shape == (4, 4))
        assert (hasattr(self.kf, 'R') and
                self.kf.R.shape == (3, 3))
        assert (hasattr(self.kf, 'time_step') and
                self.kf.time_step == self.dt)
        assert (hasattr(self.kf, 'xEst') and
                self.kf.xEst.shape == (4, 1))
        assert (hasattr(self.kf, 'PEst') and
                self.kf.PEst.shape == (4, 4))

    def test_run_step(self):
        assert isinstance(self.kf.run_step(10, 10, 10, 10, 3)[0], float)
        assert isinstance(self.kf.run_step(10, 10, 10, 10, 3)[1], float)
        assert isinstance(self.kf.run_step(10, 10, 10, 10, 3)[2], float)
        assert isinstance(self.kf.run_step(10, 10, 10, 10, 3)[3], float)

    def test_geo_to_transform(self):
        assert isinstance(geo_to_transform(100, 70, 10, 10, 10, 10)[0], float)
        assert isinstance(geo_to_transform(100, 70, 10, 10, 10, 10)[1], float)
        assert isinstance(geo_to_transform(100, 70, 10.0, 10, 10, 10.0)[2], float)


if __name__ == '__main__':
    unittest.main()