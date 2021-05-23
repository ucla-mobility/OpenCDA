# -*- coding: utf-8 -*-
"""
Visualization tools for localization
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import numpy as np
import matplotlib.pyplot as plt


class DebugHelper(object):
    """This class aims to help users debugging their localization algorithms.

    Users can apply this class to draw the x, y coordinate trajectory, yaw angle
     and vehicle speed from GNSS raw measurements, Kalman filter(or any other filter),
     and the groundtruth measurements. Error plotting is also enabled.

    Attributes:
        show_animation (bool):
        show_plotting (bool):
        scale(float):
    """

    def __init__(self, config_yaml):
        """

        Args:
            config_yaml (dict):
        """
        self.show_animation = config_yaml['show_animation']
        self.show_plotting = config_yaml['show_plotting']
        self.scale = config_yaml['scale']

        # off-line plotting
        self.gnss_x = []
        self.gnss_y = []
        self.gnss_yaw = []
        self.gnss_spd = []

        self.filter_x = []
        self.filter_y = []
        self.filter_yaw = []
        self.filter_spd = []

        self.gt_x = []
        self.gt_y = []
        self.gt_yaw = []
        self.gt_spd = []

        # the error between filter results and gt
        self.error_x = []
        self.error_y = []
        self.error_yaw = []
        self.error_spd = []

        # online animation
        # filtered x y coordinates
        self.hxEst = np.zeros((2, 1))
        # gt x y coordinates
        self.hTrue =  np.zeros((2, 1))
        # gnss x y coordinates
        self.hz = np.zeros((2, 1))

    def run_step(self, gnss_x, gnss_y, gnss_yaw, gnss_spd,
                 filter_x, filter_y, filter_yaw, filter_spd,
                 gt_x, gt_y, gt_yaw, gt_spd):
        """
        Run a single step for DebugHelper to save and animate(optional) the localization data.
        Args:
            gnss_x (float):
            gnss_y (float):
            gnss_yaw (float):
            gnss_spd (float):
            filter_x (float):
            filter_y (float):
            filter_yaw (float):
            filter_spd (float):
            gt_x (float):
            gt_y (float):
            gt_yaw (float):
            gt_spd ()float:

        Returns:

        """
        self.gnss_x.append(gnss_x)
        self.gnss_y.append(gnss_y)
        self.gnss_yaw.append(gnss_yaw)
        self.gnss_spd.append(gnss_spd)

        self.filter_x.append(filter_x)
        self.filter_y.append(filter_y)
        self.filter_yaw.append(filter_yaw)
        self.filter_spd.append(filter_spd)

        self.gt_x.append(gt_x)
        self.gt_y.append(gt_y)
        self.gt_yaw.append(gt_yaw)
        self.gt_spd.append(gt_spd)

        if self.show_animation:
            xEst = np.array([filter_x, filter_y]).reshape(2, 1)
            zTrue = np.array([gt_x, gt_y]).reshape(2, 1)
            z = np.array([gt_x, gt_y]).reshape(2, 1)

            self.hxEst = np.hstack((self.hxEst, xEst))
            self.hz = np.hstack((self.hz, z))
            self.hTrue = np.hstack((self.hTrue, zTrue))

            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(self.hTrue[0, :].flatten() * self.scale,
                     self.hTrue[1, :].flatten() * self.scale, "-b")
            plt.plot(self.hz[0, :] * self.scale, self.hz[1, :] * self.scale, ".g")
            plt.plot(self.hxEst[0, :].flatten() * self.scale,
                     self.hxEst[1, :].flatten() * self.scale, "-r")

            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)
