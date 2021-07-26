# -*- coding: utf-8 -*-

"""Finite State Machine
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from enum import Enum


class FSM(Enum):
    """
    The finite state machine class for platooning.
    These classes are used to indicate the current status
    of the CAV during platooning.

    Attributes
    ----------
    SEARCHING : int
        The vehicle is not in any platoon and currently searching one to join.
    OPEN_GAP : int
        The platoon member is increasing the gap for other vehicle to merge.
    MOVE_TO_POINT : int
        The merging vehicle is moving to the meeting points for joining.
    JOINING : int
        The merging vehicle is operating the joining maneuver(lane change).
    MAINTINING : int
        The platoon member is following the leader and maintain the time gap.
    BACK_JOINING : int
        The merging vehicle is in back-join state.
    CUT_IN_TO_BACK : int
        The merging vehicle abandons cut-in-join and switch to back join.
    JOINING_FINISHED : int
        Indicate the joining finished and the
        vehicle will switch to maintaining state.
    LEADING_MODE : int
        The vehicle is the platoon leader.
    ABONDON:
        Current joining is abandoned.
    DISABLE:
        V2X is not available and thus won't join any platoon.
    """
    SEARCHING = 0
    OPEN_GAP = 1
    MOVE_TO_POINT = 2
    JOINING = 3
    MAINTINING = 4
    BACK_JOINING = 5
    CUT_IN_TO_BACK = 6
    FRONT_JOINING = 7
    JOINING_FINISHED = 8
    LEADING_MODE = 9
    ABONDON = 10
    DISABLE = 11
