# -*- coding: utf-8 -*-

"""Tools to plot velocity, acceleration, and curvation.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License:  TDG-Attribution-NonCommercial-NoDistrib

import matplotlib.pyplot as plt
import numpy as np


def draw_velocity_profile_single_plot(velocity_list):
    """
    Draw velocity profiles in a single plot.

    Parameters
    ----------
    velocity_list : list
         The vehicle velocity profile saved in a list.
    """

    for i, v in enumerate(velocity_list):
        x_s = np.arange(len(v)) * 0.05
        plt.plot(x_s, v)

    plt.ylim([0, 34])

    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")
    fig = plt.gcf()
    fig.set_size_inches(11, 5)


def draw_acceleration_profile_single_plot(acceleration):
    """
    Draw velocity profiles in a single plot.

    Parameters
    ----------
    acceleration : list
        The vehicle acceleration profile saved in a list.

    """

    for i, v in enumerate(acceleration):
        x_s = np.arange(len(v)) * 0.05
        plt.plot(x_s, v)

    plt.ylim([-8, 8])

    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m^2/s)")
    fig = plt.gcf()
    fig.set_size_inches(11, 5)


def draw_ttc_profile_single_plot(ttc_list):
    """
    Draw ttc.

    Parameters
    ----------
    ttc_list : list
        The vehicle time to collision profile saved in a list.
    """
    # this is used to find the merging vehicle position since its inter gap
    # length is always smaller

    for i, v in enumerate(ttc_list):
        x_s = np.arange(len(v)) * 0.05
        plt.plot(x_s, v)

    plt.xlabel("Time (s)")
    plt.ylabel("TTC (s)")
    plt.ylim([0, 30])
    fig = plt.gcf()
    fig.set_size_inches(11, 5)


def draw_time_gap_profile_singel_plot(gap_list):
    """
    Draw inter gap profiles in a single plot.

    Parameters
    __________
    gap_list : list
        The vehicle front time gap profile saved in a list.

    """

    for i, v in enumerate(gap_list):
        x_s = np.arange(len(v)) * 0.05
        plt.plot(x_s, v)

    plt.xlabel("Time (s)")
    plt.ylabel("Time Gap (s)")
    plt.ylim([0.0, 1.8])
    fig = plt.gcf()
    fig.set_size_inches(11, 5)


def draw_dist_gap_profile_singel_plot(gap_list):
    """
    Draw distance gap profiles in a single plot.

    Parameters
    __________
    gap_list : list
        The vehicle front distance gap profile saved in a list.
    """
    for i, v in enumerate(gap_list):
        x_s = np.arange(len(v)) * 0.05
        plt.plot(x_s, v)

    plt.xlabel("Time (s)")
    plt.ylabel("Distance Gap (m)")
    plt.ylim([5, 35])
    fig = plt.gcf()
    fig.set_size_inches(11, 5)


def draw_sub_plot(
        velocity_list,
        acceleration_list,
        time_gap_list,
        distance_gap_list,
        ttc_list):
    """
    This is a specific function that draws 4 in 1 images
    for trajectory following task.

    Parameters
    ----------
    velocity_list : list
        The vehicle velocity profile saved in a list.

    distance_gap_list : list
        The vehicle distance gap profile saved in a list.

    time_gap_list : list
        The vehicle time gap profile saved in a list.

    acceleration_list : list
        The vehicle acceleration profile saved in a list.

    ttc_list : list
        The ttc list.

    """
    fig = plt.figure()
    plt.subplot(511)
    draw_velocity_profile_single_plot(velocity_list)

    plt.subplot(512)
    draw_acceleration_profile_single_plot(acceleration_list)

    plt.subplot(513)
    draw_time_gap_profile_singel_plot(time_gap_list)

    plt.subplot(514)
    draw_dist_gap_profile_singel_plot(distance_gap_list)

    plt.subplot(515)
    draw_dist_gap_profile_singel_plot(distance_gap_list)

    label = []
    for i in range(1, len(velocity_list) + 1):
        label.append('Leading Vehicle, id: %d' %
                     int(i - 1) if i == 1 else 'Following Vehicle, id: %d' %
                     int(i - 1))

    fig.legend(label, loc='upper right')

    return fig
