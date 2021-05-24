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
        x_scale(float):
        y_scale(float):
    """

    def __init__(self, config_yaml, actor_id):
        """

        Args:
            config_yaml (dict):
            actor_id(int):
        """
        self.show_animation = config_yaml['show_animation']
        self.show_plotting = config_yaml['show_plotting']
        self.x_scale = config_yaml['x_scale']
        self.y_scale = config_yaml['y_scale']

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

        # online animation
        # filtered x y coordinates
        self.hxEst = np.zeros((2, 1))
        # gt x y coordinates
        self.hTrue = np.zeros((2, 1))
        # gnss x y coordinates
        self.hz = np.zeros((2, 1))

        self.actor_id = actor_id

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
        self.gnss_spd.append(gnss_spd / 3.6)

        self.filter_x.append(filter_x)
        self.filter_y.append(filter_y)
        self.filter_yaw.append(filter_yaw)
        self.filter_spd.append(filter_spd / 3.6)

        self.gt_x.append(gt_x)
        self.gt_y.append(gt_y)
        self.gt_yaw.append(gt_yaw)
        self.gt_spd.append(gt_spd / 3.6)

        if self.show_animation:
            xEst = np.array([filter_x, filter_y]).reshape(2, 1)
            zTrue = np.array([gt_x, gt_y]).reshape(2, 1)
            z = np.array([gnss_x, gnss_y]).reshape(2, 1)

            self.hxEst = np.hstack((self.hxEst, xEst))
            self.hz = np.hstack((self.hz, z))
            self.hTrue = np.hstack((self.hTrue, zTrue))

            plt.cla()
            plt.title('actor id %d animation' % self.actor_id)
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [plt.close() if event.key == 'escape' else None])

            plt.plot(self.hTrue[0, 1:].flatten() * self.x_scale,
                     self.hTrue[1, 1:].flatten() * self.y_scale, "-b")
            plt.plot(self.hz[0, 1:] * self.x_scale, self.hz[1, 1:] * self.y_scale, ".g")
            plt.plot(self.hxEst[0, 1:].flatten() * self.x_scale,
                     self.hxEst[1, 1:].flatten() * self.y_scale, "-r")

            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

    def plot(self):
        """
        Plot the localization related data points.
        Args:

        Returns:

        """

        figure, axis = plt.subplots(3, 2)

        # x, y coordinates
        axis[0, 0].plot(self.gnss_x, self.gnss_y, ".g", label='gnss')
        axis[0, 0].plot(self.gt_x, self.gt_y, ".b", label='gt')
        axis[0, 0].plot(self.filter_x, self.filter_y, ".r", label='filter')
        axis[0, 0].legend()
        axis[0, 0].set_title("x-y coordinates plotting")

        # yaw angle
        axis[0, 1].plot(np.arange(len(self.gnss_yaw)), self.gnss_yaw, ".g", label='gnss')
        axis[0, 1].plot(np.arange(len(self.gt_yaw)), self.gt_yaw, ".b", label='gt')
        axis[0, 1].plot(np.arange(len(self.filter_yaw)), self.filter_yaw, ".r", label='filter')
        axis[0, 1].legend()
        axis[0, 1].set_title("yaw angle(degree) plotting")

        # speed
        axis[1, 0].plot(np.arange(len(self.gnss_spd)), self.gnss_spd, ".g", label='gnss')
        axis[1, 0].plot(np.arange(len(self.gt_spd)), self.gt_spd, ".b", label='gt')
        axis[1, 0].plot(np.arange(len(self.filter_spd)), self.filter_spd, ".r", label='filter')
        axis[1, 0].legend()
        axis[1, 0].set_title("speed(m/s) plotting")

        # error curve on x
        axis[1, 1].plot(np.arange(len(self.gnss_x)), np.array(self.gt_x) - np.array(self.gnss_x),
                        "-g", label='gnss')
        axis[1, 1].plot(np.arange(len(self.filter_x)), np.array(self.gt_x) - np.array(self.filter_x),
                        "-r", label='filter')
        axis[1, 1].legend()
        axis[1, 1].set_title("error curve on x coordinates")

        # error curve on y
        axis[2, 0].plot(np.arange(len(self.gnss_y)), np.array(self.gt_y) - np.array(self.gnss_y),
                        "-g", label='gnss')
        axis[2, 0].plot(np.arange(len(self.filter_y)), np.array(self.gt_y) - np.array(self.filter_y),
                        "-r", label='filter')
        axis[2, 0].legend()
        axis[2, 0].set_title("error curve on y coordinates")

        # error curve on yaw
        axis[2, 1].plot(np.arange(len(self.gnss_yaw)), np.array(self.gt_yaw) - np.array(self.gnss_yaw),
                        "-g", label='gnss')
        axis[2, 1].plot(np.arange(len(self.filter_yaw)), np.array(self.gt_yaw) - np.array(self.filter_yaw),
                        "-r", label='filter')
        axis[2, 1].legend()
        axis[2, 1].set_title("error curve on yaw angle")

        figure.suptitle('localization plotting of actor id %d' % self.actor_id)
        plt.show()

        print("--------------Localization Module Performance on Actor %d" % self.actor_id)
        x_error_mean = np.mean(np.abs(np.array(self.gt_x) - np.array(self.gnss_x)))
        y_error_mean = np.mean(np.abs(np.array(self.gt_y) - np.array(self.gnss_y)))
        yaw_error_mean = np.mean(np.abs(np.array(self.gt_yaw) - np.array(self.gnss_yaw)))
        print('mean error for gnss x: %f m, gnss y: %f m, gnss yaw: %f degree'
              % (x_error_mean, y_error_mean, yaw_error_mean))

        x_error_mean = np.mean(np.abs(np.array(self.gt_x) - np.array(self.filter_x)))
        y_error_mean = np.mean(np.abs(np.array(self.gt_y) - np.array(self.filter_y)))
        yaw_error_mean = np.mean(np.abs(np.array(self.gt_yaw) - np.array(self.filter_yaw)))
        print('mean error for filtered x: %f m, filtered y: %f m filter yaw: %f degree'
              % (x_error_mean, y_error_mean, yaw_error_mean))
