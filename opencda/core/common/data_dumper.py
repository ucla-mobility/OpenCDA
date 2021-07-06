# -*- coding: utf-8 -*-
"""
Dumping sensor data.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import os

import cv2
import open3d as o3d
import numpy as np


class DataDumper:
    """
    Data dumper class to save data in local disk.

    Parameters
    ----------
    perception_manager : opencda object
        The perception manager contains rgb camera data and lidar data.

    localization_manager : opencda object
        The localization manager contains gnss and imu data.

    vehicle_id : int
        The carla.Vehicle id.

    save_time : str
        The timestamp at the beginning of the simulation.

    Attributes
    ----------
    rgb_camera : list
        A list of opencda.CameraSensor that containing all rgb sensor data
        of the managed vehicle.

    lidar ; opencda object
        The lidar manager from perception manager.

    save_parent_folder : str
        The parent folder to save all data related to a specific vehicle.

    count : int
        Used to count how many steps have been executed. We dump data
        every 10 steps.

    """

    def __init__(self,
                 perception_manager,
                 localization_manager,
                 vehicle_id,
                 save_time):

        self.rgb_camera = perception_manager.rgb_camera
        self.lidar = perception_manager.lidar

        self.save_time = save_time
        self.vehicle_id = vehicle_id

        current_path = os.path.dirname(os.path.realpath(__file__))
        self.save_parent_folder = \
            os.path.join(current_path,
                         '../../../data_dumping',
                         save_time,
                         str(self.vehicle_id))

        if not os.path.exists(self.save_parent_folder):
            os.makedirs(self.save_parent_folder)

        self.count = 0

    def run_step(self):
        """
        Dump data at running time.
        """
        self.count += 1
        # we ignore the first 30 steps
        if self.count < 30:
            return

        # save data for every 5 steps
        if self.count % 5 != 0:
            return

        for (i, camera) in enumerate(self.rgb_camera):

            frame = camera.frame
            image = camera.image

            if i == 0:
                camera_position = 'front'
            elif i == 1:
                camera_position = 'right'
            else:
                camera_position = 'left'

            image_name = '%06d' % frame + '_' + camera_position + '.png'

            cv2.imwrite(os.path.join(self.save_parent_folder, image_name),
                        image)

        point_cloud = self.lidar.data
        frame = self.lidar.frame

        point_xyz = point_cloud[:, :-1]
        point_intensity = point_cloud[:, -1]
        point_intensity = np.c_[
            point_intensity,
            np.zeros_like(point_intensity),
            np.zeros_like(point_intensity)
        ]

        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(point_xyz)
        o3d_pcd.colors = o3d.utility.Vector3dVector(point_intensity)

        # write to pcd file
        pcd_name = '%06d' % frame + '.pcd'
        o3d.io.write_point_cloud(os.path.join(self.save_parent_folder,
                                              pcd_name),
                                 pointcloud=o3d_pcd,
                                 write_ascii=True)
