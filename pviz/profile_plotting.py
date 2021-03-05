# -*- coding: utf-8 -*-

"""Tools to plot velocity, acceleration, curvation....
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import math
import numpy as np
import matplotlib.pyplot as plt
import pylab


def draw_velocity_profile_separately(velocity_list, ids):
    """
    draw velocity profile for all vehicle
    :param velocity_list: a list of velocity list
    :param ids: a list of string recording each vehicle's id
    :return:
    """
    cols = 3
    rows = math.ceil(len(velocity_list) / cols)

    fig, axs = plt.subplots(rows + 1, cols)

    for i in range(len(velocity_list)):
        col = i % cols
        row = i // cols

        axs[row, col].plot(velocity_list[i])
        axs[row, col].set_title('ID: %d' % ids[i])

    for ax in axs.flat:
        ax.set(xlabel='time per 0.1 s', ylabel='speed km/h')

    for ax in axs.flat:
        ax.label_outer()

    plt.show()


def draw_velocity_profile_single_plot(velocity_list, ids):
    """
    Draw velocity profiles in a single plot
    :param velocity_list:
    :param ids:
    :return:
    """

    for i, v in enumerate(velocity_list):
        x_s = np.arange(len(v)) * 0.05
        label = 'Leading Vehicle, id: %d' % ids[i] if i == 0 else 'Following Vehicle, id: %d' % ids[i]
        pylab.plot(x_s, v, label=label)

    pylab.legend(loc=0)
    pylab.show()


def draw_intergap_profile_separately(gap_list, ids):
    """
    draw velocity profile for all vehicle
    :param gap_list: a list of velocity list
    :param ids: a list of string recording each vehicle's id
    :return:
    """
    cols = 3
    rows = math.ceil(len(gap_list) / cols)

    fig, axs = plt.subplots(rows + 1, cols)

    for i in range(len(gap_list)):
        col = i % cols
        row = i // cols

        axs[row, col].plot(gap_list[i])
        axs[row, col].set_title('ID: %d' % ids[i])

    for ax in axs.flat:
        ax.set(xlabel='time per 0.1 s', ylabel='time gap')

    for ax in axs.flat:
        ax.label_outer()

    plt.show()


if __name__ == '__main__':
    velocity_list = [[23, 25, 25, 44, 66], [44, 55, 25, 22, 33]]
    ids = [23, 45]
    draw_velocity_profile_single_plot(velocity_list, ids)
