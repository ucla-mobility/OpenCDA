import weakref
import numpy as np

from opencda.core.common.misc import get_speed
from opencda.core.sensing.perception.obstacle_vehicle import \
    ObstacleVehicle
import opencda.core.sensing.perception.sensor_transformation as st
from opencood.utils.transformation_utils import x1_to_x2


class CoperceptionLibs:
    def __init__(self, lidar, rgb_camera, localization_manager, behavior_agent, carla_world, cav_world):
        self.lidar = lidar
        self.rgb_camera = rgb_camera
        self.localizer = localization_manager
        self.agent = behavior_agent
        self.carla_world = weakref.ref(carla_world)()
        self.cav_world = weakref.ref(cav_world)()
        self.time_delay = self.calculate_time_delay()

    @staticmethod
    def matrix2list(matrix):
        assert len(matrix.shape) == 2
        return matrix.tolist()

    @staticmethod
    def calculate_time_delay():
        """
        TODO: needs revision to reflect the delay
        """
        return 0

    @staticmethod
    def load_vehicle_bbx(object_vehicle):
        veh_pos = object_vehicle.get_transform()
        veh_bbx = object_vehicle.bounding_box
        veh_speed = get_speed(object_vehicle)
        return {
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
        }

    def load_vehicles(self, ego_id, ego_pos):
        def dist_to_ego(actor):
            return actor.get_location().distance(ego_pos.location)

        world = self.carla_world
        vehicle_list = world.get_actors().filter("*vehicle*")
        vehicle_list = [v for v in vehicle_list if dist_to_ego(v) < 120 and v.id != ego_id]
        vehicle_dict = {}
        if self.lidar:
            for v in vehicle_list:
                object_vehicle = ObstacleVehicle(None, None, v, self.lidar.sensor, self.cav_world.sumo2carla_ids)
                vehicle_dict.update({object_vehicle.carla_id: self.load_vehicle_bbx(object_vehicle)})
        else:
            for v in vehicle_list:
                object_vehicle = ObstacleVehicle(None, None, v, None, self.cav_world.sumo2carla_ids)
                vehicle_dict.update({object_vehicle.carla_id: self.load_vehicle_bbx(object_vehicle)})
        data = {
            'vehicles': vehicle_dict
        }
        return data

    @staticmethod
    def load_transformation_matrix(is_ego, data):
        """
        TODO: needs revision to reflect the cur/delay params
        """
        cur_params = data
        delay_params = data
        cur_ego_params = data
        delay_ego_params = data

        delay_cav_lidar_pose = delay_params['lidar_pose']
        delay_ego_lidar_pose = delay_ego_params["lidar_pose"]
        cur_ego_lidar_pose = cur_ego_params['lidar_pose']
        cur_cav_lidar_pose = cur_params['lidar_pose']

        """"
        TODO: needs to take in the loc_error_flag
        if cav_content['ego'] and self.loc_err_flag:
            delay_cav_lidar_pose = self.add_loc_noise(delay_cav_lidar_pose,
                                                      self.xyz_noise_std,
                                                      self.ryp_noise_std)
            cur_cav_lidar_pose = self.add_loc_noise(cur_cav_lidar_pose,
                                                    self.xyz_noise_std,
                                                    self.ryp_noise_std)
        """
        if is_ego:
            transformation_matrix = x1_to_x2(delay_cav_lidar_pose, cur_ego_lidar_pose)
            spatial_correction_matrix = np.eye(4)
        else:
            transformation_matrix = x1_to_x2(delay_cav_lidar_pose, delay_ego_lidar_pose)
            spatial_correction_matrix = x1_to_x2(delay_ego_lidar_pose, cur_ego_lidar_pose)
        gt_transformation_matrix = x1_to_x2(cur_cav_lidar_pose, cur_ego_lidar_pose)
        return {
            'transformation_matrix': transformation_matrix,
            'gt_transformation_matrx': gt_transformation_matrix,
            'spatial_correction_matrix': spatial_correction_matrix
        }

    def load_cur_lidar_pose(self):
        lidar_transformation = self.lidar.sensor.get_transform()
        return {
            'lidar_pose': [
                lidar_transformation.location.x,
                lidar_transformation.location.y,
                lidar_transformation.location.z,
                lidar_transformation.rotation.roll,
                lidar_transformation.rotation.yaw,
                lidar_transformation.rotation.pitch
            ]
        }

    def load_camera_data(self, lidar, rgb_camera):
        data = {}
        for (i, camera) in enumerate(rgb_camera):
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
                st.x_to_world_transformation(lidar.sensor.get_transform())
            camera2world = \
                st.x_to_world_transformation(camera.sensor.get_transform())

            world2camera = np.linalg.inv(camera2world)
            lidar2camera = np.dot(world2camera, lidar2world)
            lidar2camera = self.matrix2list(lidar2camera)
            camera_param.update({'extrinsic': lidar2camera})
            data.update({'camera%d' % i: camera_param})
        return data

    def load_ego_data(self):
        data = {
            'predicted_ego_pos': [],
            'true_ego_pos': [],
            'ego_speed': 0.0
        }
        if not self.localizer:
            return data

        predicted_ego_pos = self.localizer.get_ego_pos()
        true_ego_pos = self.localizer.vehicle.get_transform() \
            if hasattr(self.localizer, 'vehicle') \
            else self.localizer.true_ego_pos
        data = {
            'predicted_ego_pos': [
                predicted_ego_pos.location.x,
                predicted_ego_pos.location.y,
                predicted_ego_pos.location.z,
                predicted_ego_pos.rotation.roll,
                predicted_ego_pos.rotation.yaw,
                predicted_ego_pos.rotation.pitch
            ],
            'true_ego_pos': [
                true_ego_pos.location.x,
                true_ego_pos.location.y,
                true_ego_pos.location.z,
                true_ego_pos.rotation.roll,
                true_ego_pos.rotation.yaw,
                true_ego_pos.rotation.pitch
            ],
            'ego_speed': float(self.localizer.get_ego_spd())
        }
        return data

    def load_plan_trajectory(self):
        data = {'RSU': True}
        if self.agent is not None:
            trajectory_deque = \
                self.agent.get_local_planner().get_trajectory()
            trajectory_list = []

            for i in range(len(trajectory_deque)):
                buffer = trajectory_deque.popleft()
                x = buffer[0].location.x
                y = buffer[0].location.y
                speed = buffer[1]
                trajectory_list.append([x, y, speed])

            data = {
                'plan_trajectory': trajectory_list,
                'RSU': False
            }
        return data
