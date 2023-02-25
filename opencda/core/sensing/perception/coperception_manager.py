from collections import OrderedDict


class CoperceptionManager:
    def __init__(self, vid, v2x_manager, coperception_libs):
        self.vid = vid
        self.v2x_manager = v2x_manager
        self.coperception_libs = coperception_libs

    def communicate(self):
        data = {str(self.vid): {'is_ego': True, 'data': self.v2x_manager}}
        if self.v2x_manager is not None:
            for vid, vm in self.v2x_manager.cav_nearby.items():
                data.update({str(vid): {'is_ego': False, 'data': vm}})
        return data

    def load_ego(self):
        return {'is_ego': True, 'data': self.v2x_manager}

    def prepare_data(self, lidar, ego_pos, cav_id, nearby_cav_dict):
        lidar = nearby_cav_dict['data'].get_ego_lidar()
        ego_pos = nearby_cav_dict['data'].get_ego_pos()
        camera = nearby_cav_dict['data'].get_ego_rgb_image()
        if lidar is None or ego_pos is None:
            return
        # stores the required field
        data = {cav_id: OrderedDict()}
        data[cav_id]['ego'] = nearby_cav_dict['is_ego']
        data[cav_id]['time_delay'] = self.coperception_libs.time_delay
        data[cav_id]['params'] = {}
        camera_data = self.coperception_libs.load_camera_data(lidar, camera)
        ego_data = self.coperception_libs.load_ego_data()
        plan_trajectory_data = self.coperception_libs.load_plan_trajectory()
        lidar_pose_data = self.coperception_libs.load_cur_lidar_pose()
        vehicles = self.coperception_libs.load_vehicles(cav_id, ego_pos)
        data[cav_id]['params'].update(plan_trajectory_data)
        data[cav_id]['params'].update(camera_data)
        data[cav_id]['params'].update(ego_data)
        data[cav_id]['params'].update(lidar_pose_data)
        data[cav_id]['params'].update(vehicles)
        t_matrix = self.coperception_libs.load_transformation_matrix(data[cav_id]['ego'], data[cav_id]['params'])
        data[cav_id]['params'].update(t_matrix)
        data[cav_id].update({'lidar_np': lidar.data})
        return data
