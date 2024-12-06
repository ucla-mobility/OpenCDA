# -*- coding: utf-8 -*-
"""
Dumping sensor data.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os

import cv2
import open3d as o3d
import numpy as np

from opencda.core.common.misc import get_speed
from opencda.core.sensing.perception import sensor_transformation as st
from opencda.scenario_testing.utils.yaml_utils import save_yaml
import carla


class DataDumper(object):

    def __init__(self,
                 perception_manager,
                 save_path):

        self.rgb_camera = perception_manager.rgb_camera
        self.lidar = perception_manager.lidar

        self.save_parent_folder = save_path

        if not os.path.exists(self.save_parent_folder):
            os.makedirs(self.save_parent_folder)

        self.count = 0

    def run_step(self,
                 perception_manager,
                 localization_manager,
                 behavior_agent):
        """
        Dump data at running time.

        Parameters
        ----------
        perception_manager : opencda object
            OpenCDA perception manager.

        localization_manager : opencda object
            OpenCDA localization manager.

        behavior_agent : opencda object
            Open
        """
        # self.count += 1 ###

        # we ignore the first 60 steps
        if self.count < 60:
            return
        # 10hz
        # if self.count % 2 != 0:
        #     return

        # print('saving', self.count)
        self.save_rgb_image(self.count)
        self.save_yaml_file(perception_manager,
                            localization_manager,
                            behavior_agent,
                            self.count)
        self.save_lidar_points(self.count)

    def save_rgb_image(self, count):
        """
        Save camera rgb images to disk.
        """
        if not self.rgb_camera:
            return
            
        for (i, camera) in enumerate(self.rgb_camera):

            image = camera.image

            image_name = '%06d' % count + '_' + 'camera%d' % i + '.png'

            cv2.imwrite(os.path.join(self.save_parent_folder, image_name),
                        image)
            
            # print('saved', self.perception_manager.id, os.path.join(self.save_parent_folder, image_name))###debug
            # break # to save front only

    # def reverse_transform(self, points, p, y, r):

    #     tran = carla.Transform(carla.Rotation(pitch=p, yaw=y, roll=r))
    #     rot = tran.get_matrix()[:3, :3]
    #     return (rot @ points.T).T

    # def reverse_transform(self, points, trans):
        
    #     x, y, z = trans.location.x, trans.location.y, trans.location.z
    #     p, y, r = trans.rotation.pitch, trans.rotation.yaw, trans.rotation.roll
    #     trans = carla.Transform(carla.Location(x,y,z), carla.Rotation(0,y,r))
    #     rot = np.array(trans.get_matrix())

    #     points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
    #     points = (rot @ points.T).T

    #     return points[:, :-1]

    def reverse_transform(self, points, trans):
        
        x, y, z = trans.location.x, trans.location.y, trans.location.z
        p, y, r = trans.rotation.pitch, trans.rotation.yaw, trans.rotation.roll
        # trans = carla.Transform(carla.Location(x,y,z), carla.Rotation(p,y,r))
        trans = carla.Transform(carla.Location(0,0,0), carla.Rotation(p,0,0))
        rot = np.linalg.inv(trans.get_matrix())
        # rot = trans.get_matrix()

        points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
        points = (rot @ points.T).T

        return points[:, :-1]

    def save_lidar_points(self, count):
        """
        Save 3D lidar points to disk.
        """
        point_cloud = self.lidar.data
        frame = count # self.lidar.frame

        point_xyz = point_cloud[:, :-1]
        # point_xyz[:, 1] = -point_xyz[:, 1] # horizontal flip
        point_intensity = point_cloud[:, -1]
        point_intensity = np.c_[
            point_intensity,
            np.zeros_like(point_intensity),
            np.zeros_like(point_intensity)
        ]

        # x_rot = y_rot = z_rot = 0
        roll_rot = pitch_rot = yaw_rot = 0
        if self.lidar.spawn_point.rotation.pitch != 0:
            # y_rot = np.radians(-self.lidar.spawn_point.rotation.pitch)
            pitch_rot = -self.lidar.spawn_point.rotation.pitch
        # if self.lidar.spawn_point.rotation.yaw != 0:
            # z_rot = np.radians(-self.lidar.spawn_point.rotation.yaw)
            # yaw_rot = -self.lidar.spawn_point.rotation.yaw

        if roll_rot != 0 or pitch_rot != 0 or yaw_rot != 0:
            # rot = o3d.geometry.get_rotation_matrix_from_axis_angle([x_rot, y_rot, z_rot])
            # point_xyz = (rot @ point_xyz.T).T
            # o3d_pcd.rotate(rot)
            # point_xyz = self.reverse_transform(point_xyz, pitch_rot, yaw_rot, roll_rot)
            pass
            # point_xyz = self.reverse_transform(point_xyz, self.lidar.spawn_point)

        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(point_xyz)
        o3d_pcd.colors = o3d.utility.Vector3dVector(point_intensity)

        # write to pcd file
        pcd_name = '%06d' % frame + '.pcd'
        o3d.io.write_point_cloud(os.path.join(self.save_parent_folder,
                                              pcd_name),
                                 pointcloud=o3d_pcd,
                                 write_ascii=True)
        # print('saved', self.perception_manager.id, os.path.join(self.save_parent_folder,pcd_name))###debug

    def save_yaml_file(self,
                       perception_manager,
                       localization_manager,
                       behavior_agent,
                       count):
        """
        Save objects positions/spped, true ego position,
        predicted ego position, sensor transformations.

        Parameters
        ----------
        perception_manager : opencda object
            OpenCDA perception manager.

        localization_manager : opencda object
            OpenCDA localization manager.

        behavior_agent : opencda object
            OpenCDA behavior agent.
        """
        frame = count

        dump_yml = {}
        vehicle_dict = {}

        # dump obstacle vehicles first
        objects = perception_manager.objects
        vehicle_list = objects['vehicles']
        # print('\nperception_manager vehicles', vehicle_list)

        for veh in vehicle_list:
            if perception_manager.id == veh.carla_id:
                print('perception_manager.id', perception_manager.id)
                print('veh.carla_id', veh.carla_id)
                continue ###
                
            veh_carla_id = veh.carla_id
            # veh_carla_id = str(veh.attributes['role_name'])
            veh_pos = veh.get_transform()
            veh_bbx = veh.bounding_box
            veh_speed = get_speed(veh)

            assert veh_carla_id != -1, "Please turn off perception active" \
                                       "mode if you are dumping data"

            vehicle_dict.update({veh_carla_id: {
                'bp_id': veh.type_id,
                'color': veh.color,
                "location": [veh_pos.location.x,
                             veh_pos.location.y,
                             veh_pos.location.z],
                "center": [veh_bbx.location.x,
                           veh_bbx.location.y,
                           veh_bbx.location.z],
                "angle": [veh_pos.rotation.roll,
                          veh_pos.rotation.yaw,
                          veh_pos.rotation.pitch],
                "extent": [veh_bbx.extent.x,
                           veh_bbx.extent.y,
                           veh_bbx.extent.z],
                "speed": veh_speed
            }})

        dump_yml.update({'vehicles': vehicle_dict})
        # print('perception_manager vehicle_dict', vehicle_dict)

        # dump ego pose and speed, if vehicle does not exist, then it is
        # a rsu(road side unit).
        predicted_ego_pos = localization_manager.get_ego_pos()
        true_ego_pos = localization_manager.vehicle.get_transform() \
            if hasattr(localization_manager, 'vehicle') \
            else localization_manager.true_ego_pos

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
        dump_yml.update({'ego_speed':
                        float(localization_manager.get_ego_spd())})

        # dump lidar sensor coordinates under world coordinate system
        lidar_transformation = self.lidar.sensor.get_transform()
        dump_yml.update({'lidar_pose': [
            lidar_transformation.location.x,
            lidar_transformation.location.y,
            lidar_transformation.location.z,
            lidar_transformation.rotation.roll,
            lidar_transformation.rotation.yaw,
            lidar_transformation.rotation.pitch]})
        # print('dump_yml lidar pose', lidar_transformation);print(dump_yml['lidar_pose'])
        # dump camera sensor coordinates under world coordinate system
        for (i, camera) in enumerate(self.rgb_camera):
            camera_param = {}
            camera_transformation = camera.sensor.get_transform()
            camera_param.update({'cords': [
                camera_transformation.location.x,
                camera_transformation.location.y,
                camera_transformation.location.z,
                camera_transformation.rotation.roll,
                camera_transformation.rotation.yaw,
                camera_transformation.rotation.pitch
            ]})

            # dump intrinsic matrix
            camera_intrinsic = st.get_camera_intrinsic(camera.sensor)
            camera_intrinsic = self.matrix2list(camera_intrinsic)
            camera_param.update({'intrinsic': camera_intrinsic})

            # dump extrinsic matrix lidar2camera
            lidar2world = \
                st.x_to_world_transformation(self.lidar.sensor.get_transform())
            camera2world = \
                st.x_to_world_transformation(camera.sensor.get_transform())

            world2camera = np.linalg.inv(camera2world)
            lidar2camera = np.dot(world2camera, lidar2world)
            lidar2camera = self.matrix2list(lidar2camera)
            camera_param.update({'extrinsic': lidar2camera})
            dump_yml.update({'camera%d' % i: camera_param})

        dump_yml.update({'RSU': True})
        # dump the planned trajectory if it exisit.
        if behavior_agent is not None:
            trajectory_deque = \
                behavior_agent.get_local_planner().get_trajectory()
            trajectory_list = []

            for i in range(len(trajectory_deque)):
                tmp_buffer = trajectory_deque.popleft()
                x = tmp_buffer[0].location.x
                y = tmp_buffer[0].location.y
                spd = tmp_buffer[1]

                trajectory_list.append([x, y, spd])

            dump_yml.update({'plan_trajectory': trajectory_list})
            dump_yml.update({'RSU': False})

        yml_name = '%06d' % frame + '.yaml'
        save_path = os.path.join(self.save_parent_folder,
                                 yml_name)

        save_yaml(dump_yml, save_path)
        # print('saved', self.perception_manager.id, save_path)###debug

    @staticmethod
    def matrix2list(matrix):
        """
        To generate readable yaml file, we need to convert the matrix
        to list format.

        Parameters
        ----------
        matrix : np.ndarray
            The extrinsic/intrinsic matrix.

        Returns
        -------
        matrix_list : list
            The matrix represents in list format.
        """

        assert len(matrix.shape) == 2
        return matrix.tolist()