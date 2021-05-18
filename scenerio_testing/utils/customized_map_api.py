# -*- coding: utf-8 -*-

"""Loading world from customized map
"""
# Author: Xu Han <hanxu417@ucla.edu>
# License: MIT

import os
import sys

import carla


def load_customized_world(xodr_path, client):
    """
    Load .xodr file and return the carla world object
    :param xodr_path: path to the xodr file
    :param client: created client
    :return:
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


def spawn_helper_2lanefree(carla_map, coefficient):
    """
    A helper function to locate the valid spawn point on the merge lane.
    :param carla_map: the 2lanefreeway map
    :param coefficient: a single scalar indicating where is the spawn point, eg. 0.5 represents the spawn position
    is in the middle of the merge lane
    :return: carla transform
    """

    all_deafault_spawn = carla_map.get_spawn_points()
    transform_point = all_deafault_spawn[11]
    transform_point.location.x = transform_point.location.x + \
                             coefficient * (all_deafault_spawn[2].location.x - all_deafault_spawn[11].location.x)
    transform_point.location.y = transform_point.location.y + \
                             coefficient * (all_deafault_spawn[2].location.y - all_deafault_spawn[11].location.y)

    return transform_point


def spawn_helper_2lanefree_complete(carla_map, coefficient):
    """
    A helper function to locate the valid spawn point on the merge lane.
    :param carla_map: the 2lanefreeway map
    :param coefficient: a single scalar indicating where is the spawn point, eg. 0.5 represents the spawn position
    is in the middle of the merge lane
    :return: carla transform
    """

    all_deafault_spawn = carla_map.get_spawn_points()
    transform_point = all_deafault_spawn[13]
    transform_point.location.x = transform_point.location.x + \
                             coefficient * (all_deafault_spawn[1].location.x - all_deafault_spawn[13].location.x)
    transform_point.location.y = transform_point.location.y + \
                             coefficient * (all_deafault_spawn[1].location.y - all_deafault_spawn[13].location.y)

    return transform_point