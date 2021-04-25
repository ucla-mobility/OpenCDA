# -*- coding: utf-8 -*-

"""Customized class to replace the default local behavior planner(mainly for adding trajectory)
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from core.plan.local_planner_behavior import LocalPlanner


class CustomizedLocalPlanner(LocalPlanner):
    """
    Customized Local Planner to implement trajectory method
    """

    def __init__(self, agent, buffer_size=5, dynamic_pid=False, debug=False, debug_trajectory=False, update_freq=15):
        """
        :param agent: agent that regulates the vehicle
        :param buffer_size: the buffer size for waypoint
        :param dynamic_pid: all pid parameters are dynamic based on surroundings,
        which will require customized function supplied to compute
        """
        super(CustomizedLocalPlanner, self).__init__(agent, buffer_size, dynamic_pid,
                                                     debug, debug_trajectory, update_freq)
