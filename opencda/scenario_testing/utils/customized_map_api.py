# -*- coding: utf-8 -*-

"""Loading world from customized map
"""
# Author: Xu Han <hanxu417@ucla.edu>, Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os
import sys

import carla


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def load_customized_world(xodr_path, client):
    """
    Load .xodr file and return the carla world object

    Parameters
    ----------
    xodr_path : str
        path to the xodr file

    client : carla.client
        The created CARLA simulation client.
    """
    if os.path.exists(xodr_path):
        with open(xodr_path) as od_file:
            try:
                data = od_file.read()
            except OSError:
                print('file could not be readed.')
                sys.exit()
        print('load opendrive map %r.' % os.path.basename(xodr_path))
        vertex_distance = 2.0  # in meters
        max_road_length = 500.0  # in meters
        wall_height = 1.0  # in meters
        extra_width = 0.6  # in meters
        world = client.generate_opendrive_world(
            data, carla.OpendriveGenerationParameters(
                vertex_distance=vertex_distance,
                max_road_length=max_road_length,
                wall_height=wall_height,
                additional_width=extra_width,
                smooth_junctions=False,
                enable_mesh_visibility=True))
        return world
    else:
        print('file not found.')
        return None


def spawn_helper_2lanefree(carla_version, coefficient):
    """
    A helper function to locate the valid spawn point on the merge lane.

    Parameters
    ----------
    carla_version : str
        The CARLA simulator version. We need this as the map for 0.9.11
        and 0.9.12 are a little different

    coefficient : float
        A single scalar indicating where is the  spawn point, eg. 0.5
        represents the spawn position is in the middle of the merge lane.

    Returns
    -------
    transform_point : carla.transform
        The desired spawn points.
    """
    if carla_version == '0.9.12':
        coefficient += 0.06

    transform_point = carla.Transform(carla.Location(x=-1202.0827,
                                                     y=458.2501,
                                                     z=0.3),
                                      carla.Rotation(yaw=-20.4866))

    begin_point = carla.Transform(carla.Location(x=-16.7102,
                                                 y=15.3622,
                                                 z=0.3),
                                  carla.Rotation(yaw=-20.4866))

    transform_point.location.x = transform_point.location.x + coefficient * \
                                 (begin_point.location.x -
                                  transform_point.location.x)
    transform_point.location.y = transform_point.location.y + coefficient * \
                                 (begin_point.location.y -
                                  transform_point.location.y)

    return transform_point


def spawn_helper_2lanefree_complete(carla_version, coefficient):
    """
    A helper function to locate the valid spawn point on the merge lane.

    Parameters
    ----------
    carla_version : str
        The CARLA simulator version. We need this as the map for 0.9.11
        and 0.9.12 are a little different

    coefficient : float
        A single scalar indicating where is the  spawn point, eg. 0.5
        represents the spawn position is in the middle of the merge lane.

    Returns
    -------
    transform_point : carla.transform
        The desired spawn points.
    """

    if carla_version == '0.9.12':
        coefficient += 0.06

    start_point_x = -1202.19
    start_point_y = 456.34

    merge_point_x = -31.65
    merge_point_y = 19.52
    merge_point_yaw = -20.48

    spawn_x = start_point_x + coefficient * (merge_point_x - start_point_x)
    spawn_y = start_point_y + coefficient * (merge_point_y - start_point_y)

    transform_point = carla.Transform(carla.Location(spawn_x, spawn_y, 0.3),
                                      carla.Rotation(yaw=merge_point_yaw))

    return transform_point
