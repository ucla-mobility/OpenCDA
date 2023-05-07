# -*- coding: utf-8 -*-
"""
Perception module base.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
import time

import numpy as np
import open3d as o3d
from opencda.core.sensing.perception.obstacle_vehicle import \
    ObstacleVehicle
from opencda.core.sensing.perception.perception_manager import \
    PerceptionManager
from opencda.core.sensing.perception.o3d_lidar_libs import \
    o3d_pointcloud_encode
from opencda.customize.core.sensing.perception.util import \
    project_bbx2world
from opencda.customize.core.sensing.perception.lidar_detection import \
    detect_by_clustering


class CustomizedPerceptionManager(PerceptionManager):
    """
    Customized perception module. The students need to implement the LiDAR
    clustering algorithm to detect objects.

    Parameters
    ----------
    vehicle : carla.Vehicle
        carla Vehicle, we need this to spawn sensors.

    config_yaml : dict
        Configuration dictionary for perception.

    cav_world : opencda object
        CAV World object that saves all cav information, shared ML model,
         and sumo2carla id mapping dictionary.

    data_dump : bool
        Whether dumping data, if true, semantic lidar will be spawned.

    carla_world : carla.world
        CARLA world, used for rsu.

    Attributes
    ----------
    lidar : opencda object
        Lidar sensor manager.

    rgb_camera : opencda object
        RGB camera manager.

    o3d_vis : o3d object
        Open3d point cloud visualizer.
    """

    def __init__(self, vehicle, config_yaml, cav_world,
                 data_dump=False, carla_world=None, infra_id=None):
        super(CustomizedPerceptionManager, self).__init__(vehicle, config_yaml,
                                                          cav_world, data_dump,
                                                          carla_world, infra_id)
        # set open3d verbose level
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
        # used to visualize object bounding box, maximum 50
        self.vis_aabbs = []
        for _ in range(50):
            self.vis_aabbs.append(o3d.geometry.AxisAlignedBoundingBox())

    def dist(self, a):
        """
        A fast method to retrieve the obstacle distance the ego
        vehicle from the server directly.

        Parameters
        ----------
        a : carla.actor
            The obstacle vehicle.

        Returns
        -------
        distance : float
            The distance between ego and the target actor.
        """
        return a.get_location().distance(self.ego_pos.location)

    def visualize_3d_detection(self, objects):
        """
        Visualize the 3d bounding box of the detected objects.

        Parameters
        ----------
        objects : dict
            The dictionary that contains all category of detected objects.
            The key is the object category name and value is its 3d coordinates
            and confidence.

        Returns
        -------
        None
        """
        # get the current aabb list
        aabbs = []
        for key, object_list in objects.items():
            # we only draw vehicles for now
            if key != 'vehicles':
                continue
            for i, object_ in enumerate(object_list):
                aabb = object_.o3d_bbx
                aabb.color = (0, 1, 0)
                aabbs.append(aabb)

        if self.count == 0:
            self.o3d_vis.add_geometry(self.lidar.o3d_pointcloud)
            for i in range(len(self.vis_aabbs)):
                if i < len(aabbs):
                    self.vis_aabbs[i].max_bound = aabbs[i].max_bound
                    self.vis_aabbs[i].min_bound = aabbs[i].min_bound
                    self.vis_aabbs[i].color = (0, 1, 0)
                self.o3d_vis.add_geometry(self.vis_aabbs[i])

        for i in range(len(self.vis_aabbs)):
            if i < len(aabbs):
                self.vis_aabbs[i].max_bound = aabbs[i].max_bound
                self.vis_aabbs[i].min_bound = aabbs[i].min_bound
                self.vis_aabbs[i].color = (0, 1, 0)
            else:
                self.vis_aabbs[i].max_bound = np.zeros(3)
                self.vis_aabbs[i].min_bound = np.zeros(3)
            self.o3d_vis.update_geometry(self.vis_aabbs[i])

        self.o3d_vis.update_geometry(self.lidar.o3d_pointcloud)
        self.o3d_vis.poll_events()
        self.o3d_vis.update_renderer()
        time.sleep(0.001)

    def activate_mode(self, objects):
        """
        Use simple LiDAR scanning to detect objects.

        Parameters
        ----------
        objects : dict
            The dictionary that contains all category of detected objects.
            The key is the object category name and value is its 3d coordinates
            and confidence.

        Returns
        -------
         objects: dict
            Updated object dictionary.
        """
        while self.lidar.data is None:
            continue
        # retrieve current lidar data
        lidar_data = self.lidar.data
        # use clustering algorithm to detect objects
        aabb_list, bounding_box_corner = \
            detect_by_clustering(lidar_data)
        # format the objects to a dictionary that the planning can use
        objects = {'vehicles': []}
        for i in range(len(bounding_box_corner)):
            # we need to project the coordinate to the carla world
            bbx = bounding_box_corner[i]
            aabb = aabb_list[i]
            bbx_projected = project_bbx2world(bbx,
                                              self.lidar.sensor.get_transform())
            obstacle_vehicle = ObstacleVehicle(bbx_projected,
                                               aabb)
            objects['vehicles'].append(obstacle_vehicle)

        self.speed_retrieve(objects)

        if self.lidar_visualize:
            while self.lidar.data is None:
                continue
            o3d_pointcloud_encode(self.lidar.data, self.lidar.o3d_pointcloud)
            self.visualize_3d_detection(objects)
        # add traffic light
        objects = self.retrieve_traffic_lights(objects)
        self.objects = objects

        return objects