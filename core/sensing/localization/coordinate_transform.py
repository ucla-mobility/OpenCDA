# -*- coding: utf-8 -*-
"""
Functions to transfer coordinates under different coordinate system
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT
import numpy as np
import pymap3d as pm


def geo_to_transform(lat, lon, alt, lat_0, lon_0, alt_0):
    """
    Convert WG84 to ENU. The origin of the ENU should pass the geo reference.
    :param lat: current latitude
    :param lon: current longitude
    :param alt: current altitude
    :param lat_0: geo_ref latitude
    :param lon_0: geo_ref longitude
    :param alt_0: geo_ref altitude
    :return:
    """
    north_curvature_radius = 6371848.628169

    ell = pm.Ellipsoid('wgs84')
    enu_cordinates = pm.geodetic2enu(lat, lon, alt, lat_0, lon_0, alt_0, ell=ell)

    # combining these two methods can get position with less error
    delta_north = np.deg2rad(lat - lat_0) * north_curvature_radius

    # carla is ESU
    return enu_cordinates[0], -delta_north, enu_cordinates[2]