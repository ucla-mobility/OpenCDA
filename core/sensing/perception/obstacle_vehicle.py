# -*- coding: utf-8 -*-
"""
Obstacle vehicle class to save object detection.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla
import numpy as np


def is_vehicle_cococlass(label):
    """
    Check whether the label belongs to the vehicle class according to coco dataset.
    Args:
        label(int):

    Returns:
        is_vehicle: bool
            whether this label belongs to the vehicle class
    """
    vehicle_class_array = np.array([2, 3, 4, 6, 8], dtype=np.int)
    return True if 0 in (label - vehicle_class_array) else False


class BoundingBox(object):
    """
    Bounding box class for obstacle vehicle.
    """

    def __init__(self, corners):
        """
        Construct class.
        Args:
            corners (nd.nparray): Eight corners of the bounding box. shape:(8, 3)
        """
        center_x = np.mean(corners[:, 0])
        center_y = np.mean(corners[:, 1])
        center_z = np.mean(corners[:, 2])

        extent_x = (np.max(corners[:, 0]) - np.min(corners[:, 0])) / 2
        extent_y = (np.max(corners[:, 1]) - np.min(corners[:, 1])) / 2
        extent_z = (np.max(corners[:, 2]) - np.min(corners[:, 2])) / 2

        self.location = carla.Location(x=center_x, y=center_y, z=center_z)
        self.extent = carla.Vector3D(x=extent_x, y=extent_y, z=extent_z)


class ObstacleVehicle(object):
    """
    A class for obstacle vehicle. The attributes are designed to match with carla.Vehicle class
    """

    def __init__(self, corners, o3d_bbx):
        """
        Construct class.
        Args:
            corners (nd.nparray): Eight corners of the bounding box. shape:(8, 3).
            o3d_bbx (open3d.AlignedBoundingBox): The bounding box object in Open3d. This is mainly used for
            visualization.
        """
        self.bounding_box = BoundingBox(corners)
        self.location = self.bounding_box.location
        self.o3d_bbx = o3d_bbx
        self.velocity = carla.Vector3D(0.0, 0.0, 0.0)

    def get_location(self):
        return self.location

    def get_velocity(self):
        return self.velocity

    def set_velocity(self, velocity):
        """
        Set the velocity of the vehicle.
        Args:
            velocity(carla.Vector3D): velocity in 3d vector format.

        Returns:

        """
        self.velocity = velocity


class StaticObstacle(object):
    """
    Currently, we regard all static obstacles such as stop signs and traffic light as the same class.
    """

    def __init__(self, corner, o3d_bbx):
        """
        Construct class.
        Args:
            corner (nd.nparray): Eight corners of the bounding box. shape:(8, 3)
            o3d_bbx (open3d.AlignedBoundingBox): The bounding box object in Open3d. This is mainly used for
            visualization.
        """
        self.bounding_box = BoundingBox(corner)
        self.o3d_bbx = o3d_bbx
