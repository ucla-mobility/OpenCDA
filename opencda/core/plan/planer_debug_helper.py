# -*- coding: utf-8 -*-
"""
Analysis + Visualization functions for planning
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License:  TDG-Attribution-NonCommercial-NoDistrib
import warnings

import numpy as np
import matplotlib.pyplot as plt

import opencda.core.plan.drive_profile_plotting as open_plt


class PlanDebugHelper(object):
    """
    This class aims to save statistics for planner behaviour.

    Parameters:
    -actor_id : int
        The actor ID of the target vehicle for bebuging.

    Attributes
    -speed_list : list
        The list containing speed info(m/s) of all time-steps.
    -acc_list : list
        The list containing acceleration info(m^2/s) of all time-steps.
    -ttc_list : list
        The list containing ttc info(s) for all time-steps.
    -count : int
        Used to count how many simulation steps have been executed.

    """

    def __init__(self, actor_id):
        self.actor_id = actor_id
        self.speed_list = [[]]
        self.acc_list = [[]]
        self.ttc_list = [[]]

        self.count = 0

    def update(self, ego_speed, ttc):
        """
        Update the speed info.
        Args:
            -ego_speed (float): Ego speed in km/h.
            -ttc (flot): Time to collision in seconds.

        """
        self.count += 1
        # at the very beginning, the vehicle is in a spawn state, so we should
        # filter out the first 100 data points.
        if self.count > 100:
            self.speed_list[0].append(ego_speed / 3.6)
            if len(self.speed_list[0]) <= 1:
                self.acc_list[0].append(0)
            else:
                # todo: time-step hardcoded
                self.acc_list[0].append(
                    (self.speed_list[0][-1] - self.speed_list[0][-2]) / 0.05)
            self.ttc_list[0].append(ttc)

    def evaluate(self):
        """
        Evaluate the target vehicle and visulize the plot.
        Returns:
            -figure (matplotlib.pyplot.figure): The target vehicle's planning
             profile (velocity, acceleration, and ttc).
            -perform_txt (txt file): The target vehicle's planning profile
            as text files.

        """
        warnings.filterwarnings('ignore')
        # draw speed, acc and ttc plotting
        figure = plt.figure()
        plt.subplot(311)
        open_plt.draw_velocity_profile_single_plot(self.speed_list)

        plt.subplot(312)
        open_plt.draw_acceleration_profile_single_plot(self.acc_list)

        plt.subplot(313)
        open_plt.draw_ttc_profile_single_plot(self.ttc_list)

        figure.suptitle('planning profile of actor id %d' % self.actor_id)

        # calculate the statistics
        spd_avg = np.mean(np.array(self.speed_list[0]))
        spd_std = np.std(np.array(self.speed_list[0]))

        acc_avg = np.mean(np.array(self.acc_list[0]))
        acc_std = np.std(np.array(self.acc_list[0]))

        ttc_array = np.array(self.ttc_list[0])
        ttc_array = ttc_array[ttc_array < 1000]
        ttc_avg = np.mean(ttc_array)
        ttc_std = np.std(ttc_array)

        perform_txt = 'Speed average: %f (m/s), ' \
                      'Speed std: %f (m/s) \n' % (spd_avg, spd_std)

        perform_txt += 'Acceleration average: %f (m/s), ' \
                       'Acceleration std: %f (m/s) \n' % (acc_avg, acc_std)

        perform_txt += 'TTC average: %f (m/s), ' \
                       'TTC std: %f (m/s) \n' % (ttc_avg, ttc_std)

        return figure, perform_txt
