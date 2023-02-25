# -*- coding: utf-8 -*-

"""HDMap utilities
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla
import numpy as np
import uuid
import math
from enum import IntEnum

LABEL_TO_CARLA = {'building': carla.CityObjectLabel.Buildings,
                  'terrain': carla.CityObjectLabel.Terrain,
                  'sidewalk': carla.CityObjectLabel.Sidewalks}


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


def x_to_world_transformation(transform):
    """
    Get the transformation matrix from x(it can be vehicle or sensor)
    coordinates to world coordinate.

    Parameters
    ----------
    transform : carla.Transform
        The transform that contains location and rotation

    Returns
    -------
    matrix : np.ndarray
        The transformation matrx.

    """
    rotation = transform.rotation
    location = transform.location

    # used for rotation matrix
    c_y = np.cos(np.radians(rotation.yaw))
    s_y = np.sin(np.radians(rotation.yaw))
    c_r = np.cos(np.radians(rotation.roll))
    s_r = np.sin(np.radians(rotation.roll))
    c_p = np.cos(np.radians(rotation.pitch))
    s_p = np.sin(np.radians(rotation.pitch))

    matrix = np.identity(4)
    # translation matrix
    matrix[0, 3] = location.x
    matrix[1, 3] = location.y
    matrix[2, 3] = location.z

    # rotation matrix
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r

    return matrix


def world_to_sensor(cords, sensor_transform):
    """
    Transform coordinates from world reference to sensor reference.

    Parameters
    ----------
    cords : np.ndarray
        Coordinates under world reference, shape: (4, n).

    sensor_transform : carla.Transform
        Sensor position in the world.

    Returns
    -------
    sensor_cords : np.ndarray
        Coordinates in the sensor reference.

    """
    sensor_world_matrix = x_to_world_transformation(sensor_transform)
    world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
    sensor_cords = np.dot(world_sensor_matrix, cords)

    return sensor_cords


def exclude_off_road_agents(static_bev, dynamic_bev):
    dynamic_bev[static_bev == 0] = 0
    return dynamic_bev


def retrieve_city_object_info(world, label_list):
    """
    A general function to retrieve object bbx in carla world except vehicle,
    lane, crosswalk and road.

    Parameters
    ----------
    world : carla.World
        Carla world object
    label_list : list of str
        The label that users want to retrieve.

    Returns
    -------
    A dictionary with information of the retrieved objects.
    """
    city_object_info = {}
    for label_name in label_list:
        object_ins = world.get_level_bbs(LABEL_TO_CARLA[label_name])

        obj_info = {}

        for obj in object_ins:
            obj_id = uuid.uuid4().hex[:6].upper()

            obj_transform = carla.Transform(obj.location,
                                            obj.rotation)
            obj_loc = [obj.location.x,
                       obj.location.y,
                       obj.location.z, ]
            obj_yaw = obj.rotation.yaw

            # calculate 4 corners
            bb = obj.extent
            corners = [
                carla.Location(x=-bb.x, y=-bb.y),
                carla.Location(x=-bb.x, y=bb.y),
                carla.Location(x=bb.x, y=bb.y),
                carla.Location(x=bb.x, y=-bb.y)
            ]

            obj_transform.transform(corners)
            corners_reformat = [[x.x, x.y, x.z] for x in corners]

            obj_info[obj_id] = {'location': obj_loc,
                                'yaw': obj_yaw,
                                'corners': corners_reformat}

        city_object_info.update({label_name: obj_info})

    return city_object_info


def obj_in_range(center, radius, obj_info_dict):
    """
    Retrieve the object in range.

    Parameters
    ----------
    center : carla.Transform
        The ego position

    radius : float
        Valid radius.

    obj_info_dict : dict

    Returns
    -------
    A dictionary that contains objects in range.
    """
    final_objs = {}

    for obj_category, obj_contents in obj_info_dict.items():
        cur_objs = {}
        for obj_id, obj_info in obj_contents.items():
            corners = obj_info['corners']
            for corner in corners:
                distance = math.sqrt((corner[0] - center.location.x) ** 2 + \
                                     (corner[1] - center.location.y) ** 2)
                if distance < radius:
                    cur_objs.update({obj_id: obj_info})
                    break
        final_objs.update({obj_category: cur_objs})

    return final_objs
