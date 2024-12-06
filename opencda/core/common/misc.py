#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" Module with auxiliary functions. """

import math
import importlib

import numpy as np
import carla


def draw_trajetory_points(world, waypoints, z=0.25,
                          color=carla.Color(255, 0, 0),
                          lt=5, size=0.1, arrow_size=0.1):
    """
    Draw a list of trajectory points

    Parameters
    ----------
    size : float
        Time step between updating visualized waypoint.

    lt : int
        Number of waypoints being visualized.

    color : carla.Color
        The trajectory color.

    world : carla.world
        The simulation world.

    waypoints : list
        The waypoints of the current plan.

    z : float
        The height of the visualized waypoint.
    """
    for i in range(len(waypoints)):
        wpt = waypoints[i]
        if isinstance(wpt, tuple) or isinstance(wpt, list):
            wpt = wpt[0]
        if hasattr(wpt, 'is_junction'):
            wpt_t = wpt.transform
        else:
            wpt_t = wpt

        world.debug.draw_point(
            wpt_t.location,
            size=size,
            color=color,
            life_time=lt)


def draw_waypoints(world, waypoints, z=0.5):
    """
    Draw a list of waypoints at a certain height given in z.

    Parameters
    ----------
    world : carla.world
        The simulation world.

    waypoints : list
        List or iterable container with the waypoints to draw.

    z: float
        Height in meters.
    """
    for wpt in waypoints:
        wpt_t = wpt.transform
        begin = wpt_t.location + carla.Location(z=z)
        world.debug.draw_point(begin, size=0.1, life_time=1.0)


def get_speed(vehicle, meters=False):
    """
    Compute speed of a vehicle in Km/h.

    Parameters
    ----------
    meters : bool
        Whether to use m/s (True) or km/h (False).

    vehicle : carla.vehicle
        The vehicle for which speed is calculated.

    Returns
    -------
    speed : float
        The vehicle speed.
    """
    vel = vehicle.get_velocity()
    vel_meter_per_second = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
    return vel_meter_per_second if meters else 3.6 * vel_meter_per_second


def get_acc(vehicle, meters=False):
    """
    Compute acceleration of a vehicle.

    Parameters
    ----------
    meters : bool
        Whether to use m/s^2 (True) or km/h^2 (False).

    vehicle : carla.vehicle
        The vehicle for which speed is calculated.

    Returns
    -------
    acceleration : float
        The vehicle speed.
    """
    acc = vehicle.get_acceleration()
    acc_meter_per_second = math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2)

    return acc_meter_per_second if meters else 3.6 * acc_meter_per_second


def cal_distance_angle(target_location, current_location, orientation):
    """
    Calculate the vehicle current relative distance to target location.

    Parameters
    ----------
    target_location : carla.Location
        The target location.

    current_location : carla.Location
        The current location .

    orientation : carla.Rotation
        Orientation of the reference object.

    Returns
    -------
    distance : float
        The measured distance from current location to target location.

    d_angle : float)
        The measured rotation (angle) froM current location
        to target location.
    """
    target_vector = np.array([target_location.x -
                              current_location.x, target_location.y -
                              current_location.y])
    norm_target = np.linalg.norm(target_vector) + 1e-10

    forward_vector = np.array(
        [math.cos(math.radians(orientation)),
         math.sin(math.radians(orientation))])
    d_angle = math.degrees(
        math.acos(
            np.clip(
                np.dot(
                    forward_vector, target_vector) / norm_target, -1., 1.)))

    return norm_target, d_angle


def distance_vehicle(waypoint, vehicle_transform):
    """
    Returns the 2D distance from a waypoint to a vehicle

    Parameters
    ----------
    waypoint : carla.Waypoint
        Actual waypoint.

    vehicle_transform : carla.transform
        Transform of the target vehicle.
    """
    loc = vehicle_transform.location
    if hasattr(waypoint, 'is_junction'):
        x = waypoint.transform.location.x - loc.x
        y = waypoint.transform.location.y - loc.y
    else:
        x = waypoint.location.x - loc.x
        y = waypoint.location.y - loc.y

    return math.sqrt(x * x + y * y)


def vector(location_1, location_2):
    """
    Returns the unit vector from location_1 to location_2.

    Parameters
    ----------
    location_1 : carla.location
        Start location of the vector.

    location_2 : carla.location
        End location of the vector.
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]


def compute_distance(location_1, location_2):
    """
    Euclidean distance between 3D points.

    Parameters
    ----------
    location_1 : carla.Location
        Start point of the measurement.

    location_2 : carla.Location
        End point of the measurement.
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
    return norm


def positive(num):
    """
    Return the given number if positive, else 0
    """
    return num if num > 0.0 else 0.0


def get_speed_sumo(sumo2carla_ids, carla_id):
    """
    Get the speed of the vehicles controlled by sumo.

    Parameters
    ----------
    sumo2carla_ids : dict
        Sumo-carla mapping dictionary.

    carla_id : int
        Carla actor id.

    Returns
    -------
    speed : float
        The speed retrieved from the sumo server, -1 if the carla_id not
        found.
    """
    # python will only import this once and then save it in cache. so the
    # efficiency won't affected during the loop.
    traci = importlib.import_module("traci")

    for key, value in sumo2carla_ids.items():
        if int(value) == carla_id:
            vehicle_speed = traci.vehicle.getSpeed(key)
            return vehicle_speed

    return -1

TOWN_DICTIONARY = {
    '2021_08_20_21_48_35': 'Town06',
    '2021_08_18_19_48_05': 'Town06',
    '2021_08_20_21_10_24': 'Town06',
    '2021_08_21_09_28_12': 'Town06',
    '2021_08_22_07_52_02': 'Town05',
    '2021_08_22_09_08_29': 'Town05',
    '2021_08_22_21_41_24': 'Town05',
    '2021_08_23_12_58_19': 'Town05',
    '2021_08_23_15_19_19': 'Town04',
    '2021_08_23_16_06_26': 'Town04',
    '2021_08_23_17_22_47': 'Town04',
    '2021_08_23_21_07_10': 'Town10HD',
    '2021_08_23_21_47_19': 'Town10HD',
    '2021_08_24_07_45_41': 'Town10HD',
    '2021_08_24_11_37_54': 'Town07',
    '2021_08_24_20_09_18': 'Town04',
    '2021_08_24_20_49_54': 'Town04',
    '2021_08_24_21_29_28': 'Town04',
    '2021_08_16_22_26_54': 'Town06',
    '2021_08_18_09_02_56': 'Town06',
    '2021_08_18_18_33_56': 'Town06',
    '2021_08_18_21_38_28': 'Town06',
    '2021_08_18_22_16_12': 'Town06',
    '2021_08_18_23_23_19': 'Town06',
    '2021_08_19_15_07_39': 'Town06',
    '2021_08_20_16_20_46': 'Town06',
    '2021_08_20_20_39_00': 'Town06',
    '2021_08_20_21_00_19': 'Town06',
    '2021_08_21_09_09_41': 'Town06',
    '2021_08_21_15_41_04': 'Town05',
    '2021_08_21_16_08_42': 'Town05',
    '2021_08_21_17_00_32': 'Town05',
    '2021_08_21_21_35_56': 'Town05',
    '2021_08_21_22_21_37': 'Town05',
    '2021_08_22_06_43_37': 'Town05',
    '2021_08_22_07_24_12': 'Town05',
    '2021_08_22_08_39_02': 'Town05',
    '2021_08_22_09_43_53': 'Town05',
    '2021_08_22_10_10_40': 'Town05',
    '2021_08_22_10_46_58': 'Town06',
    '2021_08_22_11_29_38': 'Town06',
    '2021_08_22_22_30_58': 'Town05',
    '2021_08_23_10_47_16': 'Town04',
    '2021_08_23_11_06_41': 'Town05',
    '2021_08_23_11_22_46': 'Town04',
    '2021_08_23_12_13_48': 'Town05',
    '2021_08_23_13_10_47': 'Town05',
    '2021_08_23_16_42_39': 'Town04',
    '2021_08_23_17_07_55': 'Town04',
    '2021_08_23_19_27_57': 'Town10HD',
    '2021_08_23_20_47_11': 'Town10HD',
    '2021_08_23_22_31_01': 'Town10HD',
    '2021_08_23_23_08_17': 'Town10HD',
    '2021_08_24_09_25_42': 'Town07',
    '2021_08_24_09_58_32': 'Town07',
    '2021_08_24_12_19_30': 'Town07',
    '2021_09_09_13_20_58': 'Town03',
    '2021_09_09_19_27_35': 'Town01',
    '2021_09_10_12_07_11': 'Town04',
    '2021_09_09_23_21_21': 'Town03',
    '2021_08_21_17_30_41': 'Town05',
    '2021_08_22_13_37_16': 'Town06',
    '2021_08_22_22_01_17': 'Town05',
    '2021_08_23_10_51_24': 'Town05',
    '2021_08_23_13_17_21': 'Town05',
    '2021_08_23_19_42_07': 'Town10HD',
    '2021_09_09_22_21_11': 'Town02',
    '2021_09_11_00_33_16': 'Town10HD',
    '2021_08_18_19_11_02': 'Town06'
}
