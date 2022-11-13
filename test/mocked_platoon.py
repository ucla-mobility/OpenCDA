# -*- coding: utf-8 -*-
"""
Mock Carla and platoon data structure for unit tests.
"""

# Author: XH <@ucla.edu>
# License: MIT

import numpy as np


class Location(object):
    """ A mock class for Location. """

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Transform(object):
    """A mock class for transform"""

    def __init__(self, x, y, z, pitch=0, yaw=0, roll=0):
        self.location = Location(x, y, z)
        self.rotation = Rotation(pitch, yaw, roll)


class Rotation(object):
    """ A mock class for Rotation. """

    def __init__(self, pitch, yaw, roll):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll


class Vector3D(object):
    """ A mock class for Vector3D. """

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class BoundingBox(object):
    """
    A mock class for bounding box.
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

        self.location = Location(x=center_x, y=center_y, z=center_z)
        self.transform = Transform(x=center_x, y=center_y, z=center_z)
        self.extent = Vector3D(x=extent_x, y=extent_y, z=extent_z)

class Vehicle(object):
    """A mock class for vehicle"""

    def __init__(self, transform, speed):
        corner = np.random.random((8, 3))
        self.transform = transform  # datatype: Transform(x=12, y=12, z=12) default rtation is (0,0,0)
        self.speed = speed          # datatype: Vector3D(x=12, y=12, z=12), m/s
        self.bounding_box = BoundingBox(corner)

    def get_transform(self):
        return self.transform

    def get_location(self):
        return self.transform.location

    def get_velocity(self):
        return self.speed

    def set_location(self, new_transform):
        '''
        Manually set location of a vehicle; 
        used to reset platoon member's position to test APF functionality.
        '''
        self.transform = new_transform


class VehicleManager(object):
    """A mock class for vehicle manager"""

    def __init__(self, vehicle):
        self.vehicle = vehicle

    def get_vehicle(self):
        return self.vehicle