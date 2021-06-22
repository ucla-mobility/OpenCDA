# -*- coding: utf-8 -*-
"""
Controller interface
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT
from collections import deque

import math
import importlib

import numpy as np
import carla


class ControlManager(object):
    """
    Interface to select different types of controller.
    """

    def __init__(self, control_config):
        """
        Construct class
        Args:
            control_config(dict): Controller params.
        """
        controller_type = control_config['type']
        controller = getattr(importlib.import_module("opencda.core.actuation.%s" % controller_type), 'Controller')
        self.controller = controller(control_config['args'])

    def update_info(self, ego_pos, ego_speed):
        self.controller.update_info(ego_pos, ego_speed)

    def run_step(self, target_speed, waypoint):
        control_command = self.controller.run_step(target_speed, waypoint)
        return control_command
