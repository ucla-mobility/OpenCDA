# -*- coding: utf-8 -*-
"""
Unit test for ML Manager.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import os
import sys
import unittest

import cv2
import numpy as np

# temporary solution for relative imports in case opencda is not installed
# if opencda is installed, no need to use the following line
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

from opencda.customize.ml_libs.ml_manager import MLManager


class TestMlManager(unittest.TestCase):
    def setUp(self):
        current_path = os.path.dirname(os.path.realpath(__file__))
        self.data = cv2.imread(os.path.join(current_path, 'data/test.jpg'))
        self.ml_manager = MLManager()

    def test_parameters(self):
        assert self.ml_manager.object_detector

    def test_draw_2d_bbx(self):
        results = self.ml_manager.object_detector(self.data)
        assert len(results) == 1
        assert self.ml_manager.draw_2d_box(results, self.data, 0).shape == self.data.shape
