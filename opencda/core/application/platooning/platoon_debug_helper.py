# -*- coding: utf-8 -*-
"""
Analysis + visualization functions for platooning
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from opencda.core.plan.planer_debug_helper \
    import PlanDebugHelper


class PlatoonDebugHelper(PlanDebugHelper):
    """This class aims to save statistics for platoon behaviour

    Parameters
    ----------
    actor_id : int
        The actor ID of the selected vehcile.

    Attributes
    ----------
    time_gap_list : list
        The list containing intra-time-gap(s) of all time-steps.

    dist_gap_list : list
        The list containing distance gap(s) of all time-steps.
    """

    def __init__(self, actor_id):
        super(PlatoonDebugHelper, self).__init__(actor_id)

        self.time_gap_list = [[]]
        self.dist_gap_list = [[]]

    def update(self, ego_speed, ttc, time_gap=None, dist_gap=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        ego_speed : float
            Ego vehcile speed.

        ttc : float
            Ego vehicle time-to-collision.

        time_gap : float
            Ego vehicle time gap with the front vehicle.

        dist_gap : float
            Ego vehicle distance gap with front vehicle.
        """
        super().update(ego_speed, ttc)
        # at the very beginning, the vehicle speed is 0, which causes
        # an infinite time gap.  So we need to filter out the first
        # 100 data points.
        if self.count > 100:
            self.time_gap_list[0].append(time_gap)
            self.dist_gap_list[0].append(dist_gap)
