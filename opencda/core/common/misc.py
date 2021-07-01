#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" Module with auxiliary functions. """

import math
import numpy as np
import carla


def draw_trajetory_points(world, waypoints, z=0.25, color=carla.Color(255, 0, 0), lt=5, size=0.1, arrow_size=0.1):
    """
    Draw a list of trajetory points

    Args
        -size (float): Time step between updating visulized waypoint (dfault=0.1).
        -lt (int): Number of waypoints being visulized (dfault=5).
        -color (carla.color): The trajectory color (default=carla.Color(255, 0, 0)).
        -world (carla.world): The simulation world.
        -waypoints (list): The waypoints of the current plan.
        -z (float): The height of the visulized waypoint (dfault=0.25). 
    """
    for i in range(len(waypoints)):
        wpt = waypoints[i]
        if isinstance(wpt, tuple) or isinstance(wpt, list):
            wpt = wpt[0]
        if hasattr(wpt, 'is_junction'):
            wpt_t = wpt.transform
        else:
            wpt_t = wpt

        world.debug.draw_arrow(wpt_t.location, wpt_t.location + wpt_t.get_forward_vector(),
                               thickness=size, arrow_size=arrow_size, color=color, life_time=lt)
        # world.debug.draw_point(wpt_t.location + carla.Location(z), size, color, lt, False)


def draw_waypoints(world, waypoints, z=0.5):
    """
    Draw a list of waypoints at a certain height given in z.

    Args
        -world: carla.world object
        -waypoints: list or iterable container with the waypoints to draw
        -z: height in meters
    """
    for wpt in waypoints:
        wpt_t = wpt.transform
        begin = wpt_t.location + carla.Location(z=z)
        angle = math.radians(wpt_t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=1.0)


def get_speed(vehicle, meters=False):
    """
    Compute speed of a vehicle in Km/h.
    
    Args
        -meters (boolean): whether to use m/s (True) or km/h (False).
        -vehicle (carla.vehicle): the vehicle for which speed is calculated.
    Returns
        -speed (flaot): The vehicle speed.
    """
    vel = vehicle.get_velocity()
    vel_meter_per_second = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
    return vel_meter_per_second if meters else 3.6 * vel_meter_per_second


def get_acc(vehicle, meters=False):
    """
    Compute acceleration of a vehicle.

    Args
        -meters (boolean): whether to use m/s^2 (True) or km/h^2 (False).
        -vehicle (carla.vehicle): the vehicle for which speed is calculated.
    Returns
        -acceleration (flaot): The vehicle speed.
    """
    acc = vehicle.get_acceleration()
    acc_meter_per_second = math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2)

    return acc_meter_per_second if meters else 3.6 * acc_meter_per_second


def is_within_distance_ahead(target_transform, current_transform, max_distance):
    """
    Check if a target object is within a certain distance in front of a reference object.

    Args
        -target_transform (carla.transform): location of the target object.
        -current_transform (carla.transform): location of the reference object.
        -orientation (carla.rotation): orientation of the reference object.
        -max_distance (float): maximum allowed distance.
    Returns
        -detection result (boolen): True if target object is within max_distance ahead of the reference object.
    """
    target_vector = np.array([target_transform.location.x - current_transform.location.x,
                              target_transform.location.y - current_transform.location.y])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    fwd = current_transform.get_forward_vector()
    forward_vector = np.array([fwd.x, fwd.y])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return d_angle < 90.0


def cal_distance_angle(target_location, current_location, orientation):
    """
    Calculate the vehicle current relative distance to target location.

    Args
        -target_location (carla.location): The target location to calcualte distance.
        -current_location (carla.location): The current location as origin for distance calculation.
        -orientation (carla.rotation): orientation of the reference object.
    Returns
        -distance (float): The measured distance from current location to target location.
        -d_angle (float): The measureed rotation (angle) fron current location to target location.
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector) + 1e-10

    forward_vector = np.array(
        [math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return norm_target, d_angle


def is_within_distance(target_location, current_location, orientation, max_distance, d_angle_th_up, d_angle_th_low=0):
    """
    Check if a target object is within a certain distance from a reference object.
    A vehicle in front would be something around 0 deg, while one behind around 180 deg.

    Args
        -target_location (carla.location): location of the target object.
        -current_location (carla.location): location of the reference object.
        -orientation (carla.rotation): orientation of the reference object.
        -max_distance (float): maximum allowed distance.
        -d_angle_th_up (carla.rotation): upper thereshold for angle.
        -d_angle_th_low (carla.rotation): low thereshold for angle (optional, default is 0).
    Returns
        - detect result (boolean): True if target object is within max_distance ahead of the reference object.
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    forward_vector = np.array(
        [math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return d_angle_th_low < d_angle < d_angle_th_up


def compute_magnitude_angle(target_location, current_location, orientation):
    """
    Compute relative angle and distance between a target_location and a current_location.

    Args
        -target_location (carla.location): location of the target object.
        -current_location (carla.location): location of the reference object.
        -orientation (carla.rotation): orientation of the reference object.
    Returns
        -compute result (tuple): A tuple composed by the distance to the object and the angle between both objects(i.e., (dist, angle)).
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return (norm_target, d_angle)


def distance_vehicle(waypoint, vehicle_transform):
    """
    Returns the 2D distance from a waypoint to a vehicle

    Args
        -waypoint (carla.waypoint): actual waypoint
        -vehicle_transform (carla.transform): transform of the target vehicle
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
    
    Args
        -location_1 (carla.location): Start location of the vector.
        -location_2 (carla.location): End location of the vector.
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]


def compute_distance(location_1, location_2):
    """
    Euclidean distance between 3D points.

    Args
        -location_1 (3D points): Start point of the measurement.
        -location_2 (3D points): End point of the measurement.
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
