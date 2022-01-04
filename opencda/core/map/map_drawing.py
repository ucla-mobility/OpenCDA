# -*- coding: utf-8 -*-

"""Rasterization drawing functions
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import numpy as np
import cv2

# sub-pixel drawing precision constants
CV2_SUB_VALUES = {"shift": 9, "lineType": cv2.LINE_AA}
CV2_SHIFT_VALUE = 2 ** CV2_SUB_VALUES["shift"]
INTERPOLATION_POINTS = 20

ROAD_COLOR = (17, 17, 31)


def cv2_subpixel(coords: np.ndarray) -> np.ndarray:
    """
    Cast coordinates to numpy.int but keep fractional part by previously multiplying by 2**CV2_SHIFT
    cv2 calls will use shift to restore original values with higher precision

    Args:
        coords (np.ndarray): XY coords as float

    Returns:
        np.ndarray: XY coords as int for cv2 shift draw
    """
    coords = coords * CV2_SHIFT_VALUE
    coords = coords.astype(np.int)
    return coords


def draw_road(lane_area_list, image):
    """
    Draw poly for road.

    Parameters
    ----------
    lane_area_list : list
        List of lane coordinates

    image : np.ndarray
        image to be drawn

    Returns
    -------
    drawed image.
    """

    for lane_area in lane_area_list:
        lane_area = lane_area.reshape(-1, 2)
        cv2.fillPoly(image, [lane_area], ROAD_COLOR,
                     **CV2_SUB_VALUES)
    return image
