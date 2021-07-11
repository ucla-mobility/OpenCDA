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

from opencda.core.common.misc import get_speed
from opencda.scenario_testing.utils.yaml_utils import save_yaml


class DataDumper(object):
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

    def run_step(self, perception_manager, localization_manager):
        """
        Dump data at running time.

        Parameters
        ----------
        perception_manager : opencda object
            OpenCDA perception manager.

        localization_manager : opencda object
            OpenCDA localization manager.
        """
        self.count += 1
        # we ignore the first 60 steps
        if self.count < 60:
            return

        # save data for every 5 steps
        if self.count % 5 != 0:
            return

        self.save_rgb_image()
        self.save_lidar_points()
        self.save_yaml_file(perception_manager, localization_manager)

    def save_rgb_image(self):
        """
        Save camera rgb images to disk.
        """
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

    def save_lidar_points(self):
        """
        Save 3D lidar points to disk.
        """
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

    def save_yaml_file(self, perception_manager, localization_manager):
        """
        Save objects positions/spped, true ego position,
        predicted ego position, sensor transformations.

        Parameters
        ----------
        perception_manager : opencda object
            OpenCDA perception manager.

        localization_manager : opencda object
            OpenCDA localization manager.
        """
        frame = self.lidar.frame

        dump_yml = {}
        vehicle_dict = {}

        # dump obstacle vehicles first
        objects = perception_manager.objects
        vehicle_list = objects['vehicles']

        for veh in vehicle_list:
            veh_carla_id = veh.carla_id
            veh_pos = veh.get_transform()
            veh_bbx = veh.bounding_box
            veh_speed = get_speed(veh)

            assert veh_carla_id != -1, "Please turn off perception active" \
                                       "mode if you are dumping data"

            vehicle_dict.update({veh_carla_id: {
                "center": [veh_pos.location.x,
                           veh_pos.location.y,
                           veh_pos.location.z],
                "angle": [veh_pos.rotation.roll,
                          veh_pos.rotation.yaw,
                          veh_pos.rotation.pitch],
                "extent": [veh_bbx.extent.x,
                           veh_bbx.extent.y,
                           veh_bbx.extent.z],
                "speed": veh_speed
            }})

        dump_yml.update({'vehicles': vehicle_dict})

        # dump ego pose and speed
        predicted_ego_pos = localization_manager.get_ego_pos()
        true_ego_pos = localization_manager.vehicle.get_transform()
        dump_yml.update({'predicted_ego_pos': [
            predicted_ego_pos.location.x,
            predicted_ego_pos.location.y,
            predicted_ego_pos.location.z,
            predicted_ego_pos.rotation.roll,
            predicted_ego_pos.rotation.yaw,
            predicted_ego_pos.rotation.pitch]})
        dump_yml.update({'true_ego_pos': [
            true_ego_pos.location.x,
            true_ego_pos.location.y,
            true_ego_pos.location.z,
            true_ego_pos.rotation.roll,
            true_ego_pos.rotation.yaw,
            true_ego_pos.rotation.pitch]})
        dump_yml.update({'ego_speed': localization_manager.get_ego_spd()})

        # dump lidar sensor transformation
        lidar_transformation = self.lidar.sensor.get_transform()
        dump_yml.update({'lidar_pose': [
            lidar_transformation.location.x,
            lidar_transformation.location.y,
            lidar_transformation.location.z,
            lidar_transformation.rotation.roll,
            lidar_transformation.rotation.yaw,
            lidar_transformation.rotation.pitch]})

        # dump camera sensor transformation
        for (i, camera) in enumerate(self.rgb_camera):
            camera_transformation = camera.sensor.get_transform()
            dump_yml.update({'camera_%d' % i: [
                camera_transformation.location.x,
                camera_transformation.location.y,
                camera_transformation.location.z,
                camera_transformation.rotation.roll,
                camera_transformation.rotation.yaw,
                camera_transformation.rotation.pitch
            ]})

        yml_name = '%06d' % frame + '.yaml'
        save_path = os.path.join(self.save_parent_folder,
                                 yml_name)

        save_yaml(dump_yml, save_path)
