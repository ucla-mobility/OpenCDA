import os

import cv2
import open3d as o3d
import numpy as np

from opencda.core.common.misc import get_speed
from opencda.core.sensing.perception import sensor_transformation as st
from opencda.scenario_testing.utils.yaml_utils import save_yaml
from opencda.core.common.data_dumper import DataDumper

class DataDumperReplay(DataDumper):

    def __init__(self,
                 perception_manager,
                 save_path):

        self.rgb_camera = perception_manager.rgb_camera
        self.lidar = perception_manager.lidar
        self.semantic_lidar = perception_manager.semantic_lidar ###

        self.save_parent_folder = save_path

        if not os.path.exists(self.save_parent_folder):
            os.makedirs(self.save_parent_folder)

        self.count = 0

    def run_step(self,
                 perception_manager,
                 localization_manager,
                 behavior_agent):

        # we ignore the first 60 steps
        if self.count < 60:
            return
        
        # 10hz
        # if self.count % 2 != 0:
        #     return

        if self.rgb_camera is not None:
            self.save_rgb_image(self.count)
        if self.lidar is not None:
            self.save_yaml_file(perception_manager,
                            localization_manager,
                            behavior_agent,
                            self.count)
        if self.lidar is not None:
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

    def save_lidar_points(self, count):
        """
        Save 3D lidar points to disk.
        """
        point_cloud = self.lidar.data
        frame = count # self.lidar.frame

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

    def save_semantic_lidar_points(self, count):

        LABEL_COLORS = np.array([
            (255, 255, 255), # None
            (70, 70, 70),    # Building
            (100, 40, 40),   # Fences
            (55, 90, 80),    # Other
            (220, 20, 60),   # Pedestrian
            (153, 153, 153), # Pole
            (157, 234, 50),  # RoadLines
            (128, 64, 128),  # Road
            (244, 35, 232),  # Sidewalk
            (107, 142, 35),  # Vegetation
            (0, 0, 142),     # Vehicle
            (102, 102, 156), # Wall
            (220, 220, 0),   # TrafficSign
            (70, 130, 180),  # Sky
            (81, 0, 81),     # Ground
            (150, 100, 100), # Bridge
            (230, 150, 140), # RailTrack
            (180, 165, 180), # GuardRail
            (250, 170, 30),  # TrafficLight
            (110, 190, 160), # Static
            (170, 120, 50),  # Dynamic
            (45, 60, 150),   # Water
            (145, 170, 100), # Terrain
        ]) / 255.0 # normalize each channel [0-1] since is what Open3D uses

        point_cloud = self.semantic_lidar.points
        frame = count # self.lidar.frame

        point_xyz = point_cloud
        label_color = LABEL_COLORS[self.semantic_lidar.obj_tag]

        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(point_xyz)
        o3d_pcd.colors = o3d.utility.Vector3dVector(label_color)

        # write to pcd file
        pcd_name = '%06d' % frame + '.pcd'
        o3d.io.write_point_cloud(os.path.join(self.save_parent_folder,
                                              pcd_name),
                                 pointcloud=o3d_pcd,
                                 write_ascii=True)

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

        for veh in vehicle_list:
            if perception_manager.id == veh.carla_id:
                continue ###
                
            veh_carla_id = veh.carla_id
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

class DataDumperReplayPly(DataDumperReplay):

    def __init__(self,
                 perception_manager,
                 save_path):

        self.semantic_lidar = perception_manager.semantic_lidar ###

        self.save_parent_folder = save_path

        if not os.path.exists(self.save_parent_folder):
            os.makedirs(self.save_parent_folder)

        self.count = 0

    def save_semantic_lidar_points_ply(self, count):
    
        path = os.path.join(self.save_parent_folder, '%06d' % count + '.ply')
        self.semantic_lidar.event.save_to_disk(path)
        print('saved', path)
    
    def run_step(self):

        # we ignore the first 60 steps
        if self.count < 60:
            return
        
        self.save_semantic_lidar_points_ply(self.count)
