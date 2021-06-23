# -*- coding: utf-8 -*-
"""
Unit test for Localization DebugHelper.
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
from opencda.core.application.platooning.platoon_debug_helper import PlatoonDebugHelper


class TestPlanDebugHelper(unittest.TestCase):
    def setUp(self):
        self.actor_id = 10
        self.platoon_debug_helper = PlatoonDebugHelper(actor_id=self.actor_id)
        self.platoon_debug_helper.count = 100

    def test_parameters(self):
        assert isinstance(self.platoon_debug_helper.speed_list[0], list)
        assert isinstance(self.platoon_debug_helper.acc_list[0], list)
        assert isinstance(self.platoon_debug_helper.ttc_list[0], list)
        assert isinstance(self.platoon_debug_helper.time_gap_list[0], list)
        assert isinstance(self.platoon_debug_helper.dist_gap_list[0], list)

    def test_update(self):
        self.platoon_debug_helper.update(90, 2, 0.8, 10)

        assert self.platoon_debug_helper.count == 101
        assert len(self.platoon_debug_helper.speed_list) == 1
        assert len(self.platoon_debug_helper.acc_list) == 1
        assert len(self.platoon_debug_helper.ttc_list) == 1
        assert len(self.platoon_debug_helper.time_gap_list) == 1
        assert len(self.platoon_debug_helper.dist_gap_list) == 1

    def test_evaluate(self):
        self.platoon_debug_helper.update(90, 2, 0.8, 10)
        figure, txt = self.platoon_debug_helper.evaluate()
        assert figure and isinstance(txt, str)