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
    LEAVE_NEGOTIATION = 5
    DISENGAGE = 6
