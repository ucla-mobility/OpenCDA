# -*- coding: utf-8 -*-
"""
Static Obstacle base class
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
import sys
import math

import numpy as np
import carla


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


class StaticObstacle(object):
    """
    The general class for obstacles. Currently, we regard all static obstacles
     such as stop signs and traffic light as the same class.

    Parameters
    ----------
    corner : nd.nparray
        Eight corners of the bounding box (shape:(8, 3)).
    o3d_bbx : open3d.AlignedBoundingBox
        The bounding box object in Open3d.This is
        mainly used for visualization.

    Attributes
    ----------
    bounding_box : BoundingBox
        Bounding box of the osbject vehicle.
    """

    def __init__(self, corner, o3d_bbx):

        self.bounding_box = BoundingBox(corner)
        self.o3d_bbx = o3d_bbx


class TrafficLight(object):
    """
    The class for traffic light. Currently, we retrieve the traffic light info
    from the server directly and assign to this class.

    Parameters
    ---------
    tl : carla.Actor
        The CARLA traffic actor

    trigger_location : carla.Vector3D
        The trigger location of te traffic light.

    pos : carla.Location
        The location of this traffic light.

    light_state : carla.TrafficLightState
        Current state of the traffic light.

    """

    def __init__(self, tl, trigger_location, light_state):
        self._location = trigger_location
        self.state = light_state
        self.actor = tl

    def get_location(self):
        return self._location

    def get_state(self):
        return self.state

    @staticmethod
    def get_trafficlight_trigger_location(traffic_light: carla.Actor) \
            -> carla.Vector3D:  # pylint: disable=invalid-name
        """
        Calculates the yaw of the waypoint that represents the trigger
        volume of the traffic light
        """

        def rotate_point(point, angle):
            """
            rotate a given point by a given angle
            """
            x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
            y_ = math.sin(math.radians(angle)) * point.x - math.cos(math.radians(angle)) * point.y

            return carla.Vector3D(x_, y_, point.z)

        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(traffic_light.trigger_volume.location)
        area_ext = traffic_light.trigger_volume.extent

        point = rotate_point(carla.Vector3D(0, 0, area_ext.z), base_rot)
        point_location = area_loc + carla.Location(x=point.x, y=point.y)

        return carla.Location(point_location.x,
                              point_location.y,
                              point_location.z)
