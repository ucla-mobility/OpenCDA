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
