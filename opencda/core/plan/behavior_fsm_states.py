# -*- coding: utf-8 -*-

"""Finite State Machine
"""

# Author: Xu Han <>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import random
import sys

import numpy as np
import networkx as nx
from enum import Enum


class BehaviorSuperStates(Enum):
    """
    The finite state machine superstates for generic behavioral planning.
    These classes are used to indicate the current status
    of the CAV during platooning.

    Attributes
    ----------
    SEARCHING : int
        The vehicle is not in any platoon and currently searching one to join.
    """
    LANE_FOLLOWING = 0
    INTERSECTION = 1
    OVERTAKING = 2
    OBSTACLE_AVOIDANCE = 3

class BehaviorStates(Enum):
    """
    The finite state machine states for generic behavioral planning.
    These classes are used to indicate the current status
    of the CAV during platooning.

    Attributes
    ----------
    SEARCHING : int
        The vehicle is not in any platoon and currently searching one to join.
    """
    # LANE_FOLLOWING
    GO_STRAIGHT = 0
    PREPARE_LANE_CHANGE_LEFT = 1
    PREPARE_LANE_CHANGE_RIGHT = 2
    LANE_CHANGE_LEFT = 3
    LANE_CHANGE_RIGHT = 4
    CAR_FOLLOWING = 5

    # INTERSECTION
    # GO_STRAIGHT = 0
    # CAR_FOLLOWING = 5
    TURN_LEFT = 6
    TURN_RIGHT = 7
    STOP = 8

    # OVERTAKING
    # GO_STRAIGHT = 0
    # PREPARE_LANE_CHANGE_LEFT = 1
    # PREPARE_LANE_CHANGE_RIGHT = 2
    # LANE_CHANGE_LEFT = 3
    # LANE_CHANGE_RIGHT = 4

    # OBSTACLE_AVOIDANCE
    SLOW_DOWN = 9
    EMERGENCY_STOP = 10
    AVOIDANCE_LANE_CHANGE_LEFT = 11
    AVOIDANCE_LANE_CHANGE_RIGHT = 12
