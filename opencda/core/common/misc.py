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

    orientation : float
        Orientation (i.e. yaw angle, "transform.rotation.yaw") of the reference object.

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