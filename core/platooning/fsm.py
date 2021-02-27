# -*- coding: utf-8 -*-

"""Finite State Machine
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from enum import Enum


class FSM(Enum):
    SEARCHING = 0
    OPEN_GAP = 1
    MOVE_TO_POINT = 2
    JOINING = 3
    MAINTINING = 4
    BACK_JOINING =5
    CUT_IN_TO_BACK = 6
