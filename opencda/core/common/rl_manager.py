# -*- coding: utf-8 -*-

""" Manager for calculating navigation status of RL ego vehcile
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from collections import deque
import weakref

import carla
import numpy as np

from opencda.core.application.platooning.platooning_plugin \
    import PlatooningPlugin
from opencda.core.common.misc import compute_distance

class RLManager(object):
