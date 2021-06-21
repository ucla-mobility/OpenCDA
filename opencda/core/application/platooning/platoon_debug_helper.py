# -*- coding: utf-8 -*-
"""
Analysis + visualization functions for platooning
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import matplotlib.pyplot as plt

from opencda.core.plan.planer_debug_helper import PlanDebugHelper


class PlatoonDebugHelper(PlanDebugHelper):
    """This class aims to save statistics for platoon behaviour
    Attributes:
        time_gap_list (list): The list containing intra-time-gap(s) of all time-steps
        dist_gap_list(list): The list containing distance gap(s) of all time-steps
    """
    def __init__(self, actor_id):
        super(PlatoonDebugHelper, self).__init__(actor_id)
        
        self.time_gap_list = [[]]
        self.dist_gap_list = [[]]
        
    def update(self, ego_speed, ttc, time_gap=None, dist_gap=None):
        super().update(ego_speed, ttc)
        self.time_gap_list[0].append(time_gap)
        self.dist_gap_list[0].append(dist_gap)