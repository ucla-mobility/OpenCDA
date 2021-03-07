# -*- coding: utf-8 -*-

"""Tools to plot velocity, acceleration, curvation....
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import json
import math
import matplotlib.pyplot as plt
import numpy as np
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


def draw_velocity_profile_single_plot(velocity_list, ids, title):
    """
    Draw velocity profiles in a single plot
    :param title:
    :param velocity_list:
    :param ids:
    :return:
    """

    for i, v in enumerate(velocity_list):
        x_s = np.arange(len(v)) * 0.05
        label = 'Vehicle, id: %d' % ids[i]
        pylab.plot(x_s, v, label=label)

    pylab.title(title)
    pylab.legend(loc=0)
    pylab.show()


def draw_intergap_profile_singel_plot(gap_list, ids, title):
    """
    Draw inter gap profiles in a single plot
    :param title:
    :param gap_list:
    :param ids:
    :return:
    """
    # this is used to find the merging vehicle position since its inter gap length is always smaller
    max_len = max(len(gap_list[0]), len(gap_list[-1]))

    for i, v in enumerate(gap_list):
        if len(v) < max_len:
            x_s = np.arange(max_len - len(v), max_len) * 0.05
        else:
            x_s = np.arange(len(v)) * 0.05

        label = 'Vehicle, id: %d' % (ids[i] + 1)
        pylab.plot(x_s, v, label=label)

    pylab.title(title)
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


def dump_data(data):
    """
    Dump data to json file
    :param data: dictionary containing all stats
    :return:
    """
    with open("platooning_2.json", "w") as outfile:
        json.dump(data, outfile)


if __name__ == '__main__':
    velocity_list = [[23, 25, 25, 44, 66], [44, 55, 25, 22, 33]]
    ids = [23, 45]
    draw_velocity_profile_single_plot(velocity_list, ids)
