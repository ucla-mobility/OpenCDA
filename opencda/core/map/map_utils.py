# -*- coding: utf-8 -*-

"""HDMap utilities
"""


# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla
import numpy as np
from enum import IntEnum


class InterpolationMethod(IntEnum):
    INTER_METER = 0  # fixed interpolation at a given step in meters
    INTER_ENSURE_LEN = 1  # ensure we always get the same number of elements


def lateral_shift(transform, shift):
    transform.rotation.yaw += 90
    return transform.location + shift * transform.get_forward_vector()


def list_loc2array(list_location):
    """
    Convert list of carla location to np.array
    Parameters
    ----------
    list_location : list
        List of carla locations.

    Returns
    -------
    loc_array : np.array
        Numpy array of shape (N, 3)
    """
    loc_array = np.zeros((len(list_location), 3))
    for (i, carla_location) in enumerate(list_location):
        loc_array[i, 0] = carla_location.x
        loc_array[i, 1] = carla_location.y
        loc_array[i, 2] = carla_location.z

    return loc_array


def list_wpt2array(list_wpt):
    """
    Convert list of carla transform to np.array
    Parameters
    ----------
    list_wpt : list
        List of carla waypoint.

    Returns
    -------
    loc_array : np.array
        Numpy array of shape (N, 3)
    """
    loc_array = np.zeros((len(list_wpt), 3))
    for (i, carla_wpt) in enumerate(list_wpt):
        loc_array[i, 0] = carla_wpt.transform.location.x
        loc_array[i, 1] = carla_wpt.transform.location.y
        loc_array[i, 2] = carla_wpt.transform.location.z

    return loc_array


def convert_tl_status(status):
    """
    Convert carla.TrafficLightState to str.
    Parameters
    ----------
    status : carla.TrafficLightState

    Returns
    -------
    status_str : str
    """
    if status == carla.TrafficLightState.Red:
        return 'red'
    elif status == carla.TrafficLightState.Green:
        return 'green'
    elif status == carla.TrafficLightState.Yellow:
        return 'yellow'
    else:
        return 'normal'
