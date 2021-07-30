# -*- coding: utf-8 -*-
"""
Functions to transfer coordinates under different coordinate system
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
import numpy as np


def geo_to_transform(lat, lon, alt, lat_0, lon_0, alt_0):
    """
    Convert WG84 to ENU. The origin of the ENU should pass the geo reference.
    Note this function is a writen by reversing the
    official API transform_to_geo.

    Parameters
    ----------
    lat : float
        current latitude.

    lon : float
        current longitude.

    alt : float
        current altitude.

    lat_0 : float)
        geo_ref latitude.

    lon_0 : float
        geo_ref longitude.

    alt_0 : float
        geo_ref altitude.

    Returns
    -------
    x : float
        The transformed x coordinate.

    y : float
        The transformed y coordinate.

    z : float
        The transformed z coordinate.
    """
    EARTH_RADIUS_EQUA = 6378137.0
    scale = np.cos(np.deg2rad(lat_0))

    mx = lon * np.pi * EARTH_RADIUS_EQUA * scale / 180
    mx_0 = scale * np.deg2rad(lon_0) * EARTH_RADIUS_EQUA
    x = mx - mx_0

    my = np.log(np.tan((lat + 90) * np.pi / 360)) * EARTH_RADIUS_EQUA * scale
    my_0 = scale * EARTH_RADIUS_EQUA * \
        np.log(np.tan((90 + lat_0) * np.pi / 360))
    y = -(my - my_0)

    z = alt - alt_0

    return x, y, z
