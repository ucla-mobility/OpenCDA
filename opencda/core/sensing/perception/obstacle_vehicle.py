# -*- coding: utf-8 -*-
"""
Obstacle vehicle class to save object detection.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla
import numpy as np
import open3d as o3d

import opencda.core.sensing.perception.sensor_transformation as st


def is_vehicle_cococlass(label):
    """
    Check whether the label belongs to the vehicle class according to coco dataset.
    Args:
        -label(int): The lable of the detecting object.

    Returns:
        -is_vehicle(bool): Whether this label belongs to the vehicle class
    """
    vehicle_class_array = np.array([2, 3, 4, 6, 8], dtype=np.int)
    return True if 0 in (label - vehicle_class_array) else False


class BoundingBox(object):
    """
    Bounding box class for obstacle vehicle.

    Params: 
    -corners : nd.nparray
        Eight corners of the bounding box. (shape:(8, 3))
    Attributes:
    -location : carla.location
        The location of the object.
    -extent : carla.vector3D
        The extent of  the object.
    """

    def __init__(self, corners):

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
    A class for obstacle vehicle. The attributes are designed to match with carla.Vehicle class.

    Parameters: 
    -corners : nd.nparray
        Eight corners of the bounding box. shape:(8, 3).
    -o3d_bbx : pen3d.AlignedBoundingBox
        The bounding box object in Open3d. This is mainly used forvisualization.
    -vehicle : carla.Vehicle
        The carla.Vehicle object.
    -lidar : carla.sensor.lidar
        The lidar sensor.

    Attributes:
    -bounding_box : BoundingBox
        Bounding box of the osbject vehicle. 
    -location : carla.location
        Location of the object.
    -velocty : carla.Vector3D vehicle.
        Velocity of the object vehicle.
    """

    def __init__(self, corners, o3d_bbx, vehicle=None, lidar=None):
        
        if not vehicle:
            self.bounding_box = BoundingBox(corners)
            self.location = self.bounding_box.location
            self.o3d_bbx = o3d_bbx
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
        else:
            self.set_vehicle(vehicle, lidar)

    def get_transform(self):
        """
        Return the transform of the object vehicle. 
        """
        return self.transform

    def get_location(self):
        """
        Return the location of the object vehicle. 
        """
        return self.location

    def get_velocity(self):
        """
        Return the velocity of the object vehicle. 
        """
        return self.velocity

    def set_velocity(self, velocity):
        """
        Set the velocity of the vehicle.
        Args:
            -velocity(carla.Vector3D): The target velocity in 3d vector format.

        """
        self.velocity = velocity

    def set_vehicle(self, vehicle, lidar):
        """
        Assign the attributes from carla.Vehicle to ObstacleVehicle.

        Args:
            -vehicle(carla.Vehicle): The carla.Vehicle object.
            -lidar(carla.sensor.lidar): The lidar sensor, used to project world coordinates to sensor coordinates.
        """
        self.location = vehicle.get_location()
        self.transform = vehicle.get_transform()
        self.bounding_box = vehicle.bounding_box
        self.set_velocity(vehicle.get_velocity())

        # find the min and max boundary
        min_boundary = np.array([self.location.x - self.bounding_box.extent.x,
                                 self.location.y - self.bounding_box.extent.y,
                                 self.location.z + self.bounding_box.location.z - self.bounding_box.extent.z,
                                 1])
        max_boundary = np.array([self.location.x + self.bounding_box.extent.x,
                                 self.location.y + self.bounding_box.extent.y,
                                 self.location.z + self.bounding_box.location.z + self.bounding_box.extent.z,
                                 1])
        min_boundary = min_boundary.reshape((4, 1))
        max_boundary = max_boundary.reshape((4, 1))
        stack_boundary = np.hstack((min_boundary, max_boundary))

        # the boundary coord at the lidar sensor space
        stack_boundary_sensor_cords = st.world_to_sensor(stack_boundary, lidar.get_transform())
        # convert unreal space to o3d space
        stack_boundary_sensor_cords[:1, :] = -stack_boundary_sensor_cords[:1, :]
        # (4,2) -> (3, 2)
        stack_boundary_sensor_cords = stack_boundary_sensor_cords[:-1, :]

        min_boundary_sensor = np.min(stack_boundary_sensor_cords, axis=1)
        max_boundary_sensor = np.max(stack_boundary_sensor_cords, axis=1)

        aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_boundary_sensor,
                                                   max_bound=max_boundary_sensor)
        aabb.color = (1, 0, 0)
        self.o3d_bbx = aabb


class StaticObstacle(object):
    """
    The general class for obstacles. Currently, we regard all static obstacles such as stop signs and traffic light as the same class.

    Parameters 
    -corner : nd.nparray
        Eight corners of the bounding box (shape:(8, 3)).
    -o3d_bbx : open3d.AlignedBoundingBox
        The bounding box object in Open3d. This is mainly used for visualization.
    
    Attributes
    -bounding_box : BoundingBox
        Bounding box of the osbject vehicle. 

    """

    def __init__(self, corner, o3d_bbx):
        
        self.bounding_box = BoundingBox(corner)
        self.o3d_bbx = o3d_bbx
