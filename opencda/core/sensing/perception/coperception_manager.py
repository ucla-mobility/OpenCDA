from collections import OrderedDict


class CoperceptionManager:
    def __init__(self, vid, v2x_manager, coperception_libs):
        self.vid = vid
        self.v2x_manager = v2x_manager
        self.coperception_libs = coperception_libs
        self.ego_data_dict = None

    def communicate(self):
        data = {}
        if self.v2x_manager is not None:
            for vid, vm in self.v2x_manager.cav_nearby.items():
                data.update({str(vid): vm})
        return data

    def calculate_transformation(self, cav_id, cav_data):
        t_matrix = self.coperception_libs.load_transformation_matrix(self.ego_data_dict, cav_data[cav_id]['params'])
        cav_data[cav_id]['params'].update(t_matrix)
        return cav_data

    def prepare_data(self, cav_id, camera, lidar, pos, localizer, agent, is_ego, ego_params=None):
        data = {cav_id: OrderedDict()}
        data[cav_id]['ego'] = is_ego
        data[cav_id]['time_delay'] = self.coperception_libs.time_delay
        data[cav_id]['params'] = {}
        camera_data = self.coperception_libs.load_camera_data(lidar, camera)
        ego_data = self.coperception_libs.load_ego_data(localizer)
        plan_trajectory_data = self.coperception_libs.load_plan_trajectory(agent)
        lidar_pose_data = self.coperception_libs.load_cur_lidar_pose(lidar)
        vehicles = self.coperception_libs.load_vehicles(cav_id, pos, lidar)
        data[cav_id]['params'].update(plan_trajectory_data)
        data[cav_id]['params'].update(camera_data)
        data[cav_id]['params'].update(ego_data)
        data[cav_id]['params'].update(lidar_pose_data)
        data[cav_id]['params'].update(vehicles)
        data[cav_id].update({'lidar_np': lidar.data})
        # get base_data_dict
        if is_ego:
            self.ego_data_dict = data[cav_id]['params']
        return data
