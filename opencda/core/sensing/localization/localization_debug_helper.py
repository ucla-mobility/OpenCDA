# -*- coding: utf-8 -*-
"""
Visualization tools for localization
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import numpy as np
import matplotlib

import matplotlib.pyplot as plt


class LocDebugHelper(object):
    """
    This class aims to help users debugging their localization algorithms.
    Users can apply this class to draw the x, y coordinate
    trajectory, yaw angle and vehicle speed from GNSS raw measurements,
    Kalman filter, and the groundtruth measurements.
    Error plotting is also enabled.

    Attributes
        show_animation : boolean
            Indicator of whether to visulize animtion.
        x_scale : float
            The scale of x coordinates.
        y_scale : float
            The scale of y coordinates.
        gnss_x : list
            The list of recorded gnss x coordinates.
        gnss_y : list
            The list of recorded gnss y coordinates.
        gnss_yaw : list
            The list of recorded gnss yaw angles.
        gnss_speed : list
            The list of recorded gnss speed values.
        filter_x : list
            The list of filtered x coordinates.
        filter_y : list
            The list of filtered y coordinates.
        filter_yaw : list
            The list of filtered yaw angles.
        filter_speed : list
            The list of filtered speed values.
        gt_x : list
            The list of ground truth x coordinates.
        gt_y : list
            The list of ground truth y coordinates.
        gt_yaw : list
            The list of ground truth yaw angles.
        gt_speed : list
            The list of ground truth speed values.
        hxEst : list
            The filtered x y coordinates.
        hTrue : list
            The true x y coordinates.
        hz : list
            The gnss detected x y coordinates.
        actor_id : int
            The list of ground truth speed values.
    """

    def __init__(self, config_yaml, actor_id):

        self.show_animation = config_yaml['show_animation']
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
        Run a single step for DebugHelper to save and animate(optional)
        the localization data.

        Args:
            -gnss_x (float): GNSS detected x coordinate.
            -gnss_y (float): GNSS detected y coordinate.
            -gnss_yaw (float): GNSS detected yaw angle.
            -gnss_spd (float): GNSS detected speed value.
            -filter_x (float): Filtered x coordinates.
            -filter_y (float): Filtered y coordinates.
            -filter_yaw (float): Filtered yaw angle.
            -filter_spd (float): Filtered speed value.
            -gt_x (float): The ground truth x coordinate.
            -gt_y (float): The ground truth y coordinate.
            -gt_yaw (float): The ground truth yaw angle.
            -gt_spd (float): The ground truth speed value.

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
            # call backend setting here to solve the conflict between cv2 pyqt5
            # and pyplot qtagg
            try:
                matplotlib.use('TkAgg')
            except ImportError:
                pass
            xEst = np.array([filter_x, filter_y]).reshape(2, 1)
            zTrue = np.array([gt_x, gt_y]).reshape(2, 1)
            z = np.array([gnss_x, gnss_y]).reshape(2, 1)

            self.hxEst = np.hstack((self.hxEst, xEst))
            self.hz = np.hstack((self.hz, z))
            self.hTrue = np.hstack((self.hTrue, zTrue))

            plt.cla()
            plt.title('actor id %d localization trajectory' % self.actor_id)
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event', lambda event: [
                    plt.close() if event.key == 'escape' else None])

            plt.plot(self.hTrue[0, 1:].flatten() * self.x_scale,
                     self.hTrue[1, 1:].flatten() * self.y_scale, "-b",
                     label='groundtruth')
            plt.plot(self.hz[0, 1:] *
                     self.x_scale, self.hz[1, 1:] *
                     self.y_scale, ".g", label='gnss noise data')
            plt.plot(self.hxEst[0, 1:].flatten() * self.x_scale,
                     self.hxEst[1, 1:].flatten() * self.y_scale, "-r",
                     label='kf result')

            plt.axis("equal")
            plt.grid(True)
            plt.legend()
            plt.pause(0.001)

    def evaluate(self):
        """
        Plot the localization related data points.

        Returns:
            -figures(matplotlib.pyplot.plot): The plot of
            localization related figures.
            -perform_txt(txt file): The localization related
            datas saved as text file.

        """
        figure, axis = plt.subplots(3, 2)
        figure.set_size_inches(16, 12)
        # x, y coordinates
        axis[0, 0].plot(self.gnss_x, self.gnss_y, ".g", label='gnss')
        axis[0, 0].plot(self.gt_x, self.gt_y, ".b", label='gt')
        axis[0, 0].plot(self.filter_x, self.filter_y, ".r", label='filter')
        axis[0, 0].legend()
        axis[0, 0].set_title("x-y coordinates plotting")

        # yaw angle
        axis[0, 1].plot(np.arange(len(self.gnss_yaw)),
                        self.gnss_yaw, ".g", label='gnss')
        axis[0, 1].plot(np.arange(len(self.gt_yaw)),
                        self.gt_yaw, ".b", label='gt')
        axis[0, 1].plot(np.arange(len(self.filter_yaw)),
                        self.filter_yaw, ".r", label='filter')
        axis[0, 1].legend()
        axis[0, 1].set_title("yaw angle(degree) plotting")

        # speed
        axis[1, 0].plot(np.arange(len(self.gnss_spd)),
                        self.gnss_spd, ".g", label='gnss')
        axis[1, 0].plot(np.arange(len(self.gt_spd)),
                        self.gt_spd, ".b", label='gt')
        axis[1, 0].plot(np.arange(len(self.filter_spd)),
                        self.filter_spd, ".r", label='filter')
        axis[1, 0].legend()
        axis[1, 0].set_title("speed(m/s) plotting")

        # error curve on x
        axis[1, 1].plot(np.arange(len(self.gnss_x)), np.array(
            self.gt_x) - np.array(self.gnss_x), "-g", label='gnss')
        axis[1, 1].plot(np.arange(len(self.filter_x)), np.array(
            self.gt_x) - np.array(self.filter_x), "-r", label='filter')
        axis[1, 1].legend()
        axis[1, 1].set_title("error curve on x coordinates")

        # error curve on y
        axis[2, 0].plot(np.arange(len(self.gnss_y)), np.array(
            self.gt_y) - np.array(self.gnss_y), "-g", label='gnss')
        axis[2, 0].plot(np.arange(len(self.filter_y)), np.array(
            self.gt_y) - np.array(self.filter_y), "-r", label='filter')
        axis[2, 0].legend()
        axis[2, 0].set_title("error curve on y coordinates")

        # error curve on yaw
        axis[2, 1].plot(np.arange(len(self.gnss_yaw)), np.array(
            self.gt_yaw) - np.array(self.gnss_yaw), "-g", label='gnss')
        axis[2, 1].plot(np.arange(len(self.filter_yaw)), np.array(
            self.gt_yaw) - np.array(self.filter_yaw), "-r", label='filter')
        axis[2, 1].legend()
        axis[2, 1].set_title("error curve on yaw angle")

        figure.suptitle('localization plotting of actor id %d' % self.actor_id)

        x_error_mean = np.mean(
            np.abs(
                np.array(
                    self.gt_x) -
                np.array(
                    self.gnss_x)))
        y_error_mean = np.mean(
            np.abs(
                np.array(
                    self.gt_y) -
                np.array(
                    self.gnss_y)))
        yaw_error_mean = np.mean(
            np.abs(
                np.array(
                    self.gt_yaw) -
                np.array(
                    self.gnss_yaw)))

        perform_txt = 'mean error for GNSS raw data on x-axis: %f (meter), ' \
                      'mean error for GNSS raw data on y-axis: %f (meter),' \
                      'mean error for GNSS raw data on yaw : %f (degree) \n'\
                      % (x_error_mean,
                         y_error_mean,
                         yaw_error_mean)

        x_error_mean = np.mean(
            np.abs(
                np.array(
                    self.gt_x) -
                np.array(
                    self.filter_x)))
        y_error_mean = np.mean(
            np.abs(
                np.array(
                    self.gt_y) -
                np.array(
                    self.filter_y)))
        yaw_error_mean = np.mean(
            np.abs(
                np.array(
                    self.gt_yaw) -
                np.array(
                    self.filter_yaw)))

        perform_txt += 'mean error after data fusion on x-axis: %f (meter), ' \
                       'mean error after data fusion  on y-axis: %f (meter),' \
                       'mean error after data fusion yaw : %f (degree) \n' \
                       % (x_error_mean,
                          y_error_mean,
                          yaw_error_mean)

        return figure, perform_txt
