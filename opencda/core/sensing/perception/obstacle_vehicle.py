# -*- coding: utf-8 -*-
"""
Obstacle vehicle class to save object detection.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
import sys

import carla
import numpy as np
import open3d as o3d

import opencda.core.sensing.perception.sensor_transformation as st
from opencda.core.common.misc import get_speed_sumo


def is_vehicle_cococlass(label):
    """
    Check whether the label belongs to the vehicle class
    according to coco dataset.
    Args:
        -label(int): The lable of the detecting object.

    Returns:
        -is_vehicle(bool): Whether this label belongs to the vehicle class
    """
    vehicle_class_array = np.array([1, 2, 3, 5, 7], dtype=np.int)
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
    A class for obstacle vehicle. The attributes are designed to match
    with carla.Vehicle class.

    Parameters
    ----------
    corners : nd.nparray
        Eight corners of the bounding box. shape:(8, 3).

    o3d_bbx : pen3d.AlignedBoundingBox
        The bounding box object in Open3d. This is mainly used for
        visualization

    vehicle : carla.Vehicle
        The carla.Vehicle object.

    lidar : carla.sensor.lidar
        The lidar sensor.

    sumo2carla_ids : dict
        Sumo to carla mapping dictionary, this is used only when co-simulation
        is activated. We need this since the speed info of  vehicles that
        are controlled by sumo can not be read from carla server. We will
        need this dict to read vehicle speed from sumo api--traci.

    Attributes
    ----------
    bounding_box : BoundingBox
        Bounding box of the osbject vehicle.

    location : carla.location
        Location of the object.

    velocity : carla.Vector3D
        Velocity of the object vehicle.

    carla_id : int
        The obstacle vehicle's id. It should be the same with the
        corresponding carla.Vehicle's id. If no carla vehicle is
        matched with the obstacle vehicle, it should be -1.
    """

    def __init__(self, corners, o3d_bbx,
                 vehicle=None, lidar=None, sumo2carla_ids=None):

        if not vehicle:
            self.bounding_box = BoundingBox(corners)
            self.location = self.bounding_box.location
            # todo: next version will add rotation estimation
            self.transform = None
            self.o3d_bbx = o3d_bbx
            self.carla_id = -1
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
        else:
            if sumo2carla_ids is None:
                sumo2carla_ids = dict()
            self.set_vehicle(vehicle, lidar, sumo2carla_ids)

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

    def set_carla_id(self, id):
        """
        Set carla id according to the carla.vehicle.

        Parameters
        ----------
        id : int
            The id from the carla.vehicle.
        """
        self.carla_id = id

    def set_velocity(self, velocity):
        """
        Set the velocity of the vehicle.

        Parameters
        ----------
        velocity : carla.Vector3D
            The target velocity in 3d vector format.

        """
        self.velocity = velocity

    def set_vehicle(self, vehicle, lidar, sumo2carla_ids):
        """
        Assign the attributes from carla.Vehicle to ObstacleVehicle.

        Parameters
        ----------
        vehicle : carla.Vehicle
            The carla.Vehicle object.

        lidar : carla.sensor.lidar
            The lidar sensor, it is used to project world coordinates to
             sensor coordinates.

        sumo2carla_ids : dict
            Sumo to carla mapping dictionary, this is used only when
            co-simulation is activated. We need this since the speed info of
            vehicles that are controlled by sumo can not be read from carla
            server. We will need this dict to read vehicle speed
            from sumo api--traci.
        """
        self.location = vehicle.get_location()
        self.transform = vehicle.get_transform()
        self.bounding_box = vehicle.bounding_box
        self.carla_id = vehicle.id
        self.type_id = vehicle.type_id
        self.color = vehicle.attributes["color"] \
            if hasattr(vehicle, "attributes") \
               and "color" in vehicle.attributes else None

        self.set_velocity(vehicle.get_velocity())
        # the vehicle controlled by sumo has speed 0 in carla,
        # thus we need to retrieve the correct number from sumo
        if len(sumo2carla_ids) > 0:
            sumo_speed = get_speed_sumo(sumo2carla_ids, self.carla_id)
            if sumo_speed > 0:
                # todo: consider the yaw angle in the future
                speed_vector = carla.Vector3D(sumo_speed, 0, 0)
                self.set_velocity(speed_vector)

        # find the min and max boundary
        min_boundary = np.array([self.location.x - self.bounding_box.extent.x,
                                 self.location.y - self.bounding_box.extent.y,
                                 self.location.z + self.bounding_box.location.z
                                 - self.bounding_box.extent.z,
                                 1])
        max_boundary = np.array([self.location.x + self.bounding_box.extent.x,
                                 self.location.y + self.bounding_box.extent.y,
                                 self.location.z + self.bounding_box.location.z
                                 + self.bounding_box.extent.z,
                                 1])
        min_boundary = min_boundary.reshape((4, 1))
        max_boundary = max_boundary.reshape((4, 1))
        stack_boundary = np.hstack((min_boundary, max_boundary))

        if lidar is None:
            return
        # the boundary coord at the lidar sensor space
        stack_boundary_sensor_cords = st.world_to_sensor(stack_boundary,
                                                         lidar.get_transform())
        # convert unreal space to o3d space
        stack_boundary_sensor_cords[:1, :] = - \
            stack_boundary_sensor_cords[:1, :]
        # (4,2) -> (3, 2)
        stack_boundary_sensor_cords = stack_boundary_sensor_cords[:-1, :]

        min_boundary_sensor = np.min(stack_boundary_sensor_cords, axis=1)
        max_boundary_sensor = np.max(stack_boundary_sensor_cords, axis=1)

        aabb = \
            o3d.geometry.AxisAlignedBoundingBox(min_bound=min_boundary_sensor,
                                                max_bound=max_boundary_sensor)
        aabb.color = (1, 0, 0)
        self.o3d_bbx = aabb
