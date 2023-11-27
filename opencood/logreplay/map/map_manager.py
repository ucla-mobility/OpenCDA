# -*- coding: utf-8 -*-

"""HDMap manager
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import os.path
import uuid

import cv2
import carla
import numpy as np
from matplotlib.path import Path
from shapely.geometry import Polygon

from logreplay.assets.presave_lib import EXCLUDE_ROAD_MAP, OR_Z_VALUE_MAP
from logreplay.map.map_utils import \
    world_to_sensor, lateral_shift, list_loc2array, list_wpt2array, \
    convert_tl_status, exclude_off_road_agents, retrieve_city_object_info, \
    obj_in_range
from logreplay.map.map_drawing import \
    cv2_subpixel, draw_agent, draw_road, \
    draw_lane, road_exclude, draw_crosswalks, draw_city_objects
from opencood.hypes_yaml.yaml_utils import save_yaml_wo_overwriting
CV2_SUB_VALUES = {"shift": 9, "lineType": cv2.LINE_AA}


class MapManager(object):
    """
    This class is used to manage HD Map. We emulate the style of Lyft dataset.

    Parameters
    ----------
    world : Carla.World
        CARLA world.

    config : dict
        All the map manager parameters.

    output_root : str
        The data dump root folder.

    scene_name : str
        The name of the scene.

    Attributes
    ----------
    world : carla.world
        Carla simulation world.

    meter_per_pixel : float
        m/pixel

    raster_size : float
        The rasterization map size in pixels.

    radius_meter : float
        The valid radius(m) in the center of the rasterization map.

    topology : list
        Map topology in list.

    lane_info : dict
        A dictionary that contains all lane information.

    crosswalk_info : dict
        A dictionary that contains all crosswalk information.
        todo: will be implemented in the next version.

    traffic_light_info : dict
        A dictionary that contains all traffic light information.

    bound_info : dict
        A dictionary that saves boundary information of lanes and crosswalks.
        It is used to efficiently filter out invalid lanes/crosswarlks.

    lane_sample_resolution : int
        The sampling resolution for drawing lanes.

    draw_lane : bool
        Whether to draw lane on the static bev map

    static_bev : np.array
        The static bev map containing lanes and drivable road information.

    dynamic_bev : np.array
        The dynamic bev map containing vehicle's information.

    vis_bev : np.array
        The comprehensive bev map for visualization.

    """

    def __init__(self, world, config, output_root, scene_name):
        self.world = world
        self.carla_map = world.get_map()
        self.center = None
        self.actor_id = None
        self.out_root = output_root
        self.scene_name = scene_name

        # whether to activate this module
        self.activate = config['activate']
        if not self.activate:
            return

        # save flags
        self.save_yml = config['save_yml']
        self.save_static = config['save_static']
        self.save_dynamic = config['save_dynamic']
        self.save_lane = config['save_lane']
        self.save_bev_vis = config['save_bev_vis']

        # whether to visualize the bev map while running simulation
        self.visualize = config['visualize']
        # whether exclude the road that is unrelated to the ego vehicle
        self.exclude_road = config['static']['exclude_road']
        if self.scene_name in EXCLUDE_ROAD_MAP:
            self.exclude_road = True

        self.radius_meter = config['radius']
        self.z_filter_value = config['static']['z_filter_value']
        self.exclude_intersection_lane = \
            config['static']['exclude_intersection_lane']
        self.other_render_objs = config['static']['other_objs']

        assert config['raster_size'][0] == config['raster_size'][1]
        self.raster_size = np.array([config['raster_size'][0],
                                     config['raster_size'][1]])
        self.pixels_per_meter = self.raster_size[0] / (self.radius_meter * 2)
        self.meter_per_pixel = 1 / self.pixels_per_meter
        self.lane_sample_resolution = config['lane_sample_resolution']

        # list of all start waypoints in HD Map
        topology = [x[0] for x in self.carla_map.get_topology()]
        # sort by altitude
        self.topology = sorted(topology, key=lambda w: w.transform.location.z)

        # basic elements in HDMap: lane, crosswalk and traffic light
        self.lane_info = {}
        self.crosswalk_info = {}
        self.traffic_light_info = {}
        # this is mainly used for efficient filtering
        self.bound_info = {'lanes': {},
                           'crosswalks': {}}
        self.traffic_stop_pos = []

        self.retrieve_light_stop_pos()
        # generate information for traffic light
        self.generate_tl_info(self.world)
        # generate lane, crosswalk and boundary information
        self.generate_lane_cross_info()

        # generate labels
        self.other_objs_info = \
            retrieve_city_object_info(self.world,
                                      self.other_render_objs)

        # static related info
        self.draw_lane = config['static']['draw_lane']
        self.draw_traffic_light = config['static']['draw_traffic_light']
        # dynamic related info
        self.exclude_self = config['dynamic']['exclude_self']
        self.exclude_off_road = config['dynamic']['exclude_off_road']

        # whether to check visibility in camera for ego only
        self.visibility = config['dynamic']['visibility']
        # visible agent id for ego
        self.vis_ids = []

        # whether to check visibility in camera for multi-agent perspective
        self.visibility_corp = config['dynamic']['visibility_corp']
        # visible agent id for all vehicles
        self.vis_corp_ids = []

        # bev maps
        self.dynamic_bev = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.vis_mask = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.vis_corp_mask = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.static_bev = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.lane_bev = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.vis_bev = 255 * np.ones(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)

    def run_step(self, cav_id, cav_content, veh_dict):
        """
        Rasterization + Visualize the bev map if needed.

        Parameters
        ----------
        cav_id : str
            The cav's original id in the dataset.
        cav_content : dict
            The cav's information.
        veh_dict : dict
            All vehicles' information. Used for visibility check for coop.
        """
        if not self.activate:
            return

        self.actor_id = cav_content['actor_id']
        self.center = cav_content['cur_pose']
        self.agent_id = cav_id
        self.current_timstamp = cav_content['cur_count']

        # clean buffer every round starts
        self.vis_ids = []
        self.vis_corp_ids = []

        if self.visibility:
            self.vis_ids = self.check_visibility_single(cav_content,
                                                        self.vis_ids)
        if self.visibility_corp:
            self.vis_corp_ids = self.check_visibility_corp(veh_dict,
                                                           self.vis_corp_ids)

        self.rasterize_static()
        self.rasterize_dynamic()

        if self.exclude_off_road:
            self.dynamic_bev = exclude_off_road_agents(self.static_bev,
                                                       self.dynamic_bev)

        if self.visualize:
            cv2.imshow('the bev map of agent %s' % self.agent_id,
                       self.vis_bev)
            cv2.waitKey(1)
        self.data_dump()

    def check_visibility_corp(self, veh_dict, vis_corp_mask_list):
        for _, veh_contnet in veh_dict.items():
            if 'cav' in veh_contnet:
                vis_corp_mask_list += \
                    self.check_visibility_single(veh_contnet,
                                                 vis_corp_mask_list)
        vis_corp_mask_list = list(set(vis_corp_mask_list))
        return vis_corp_mask_list

    # check visibility agent list for ego vehicle
    @staticmethod
    def check_visibility_single(cav_content, vis_mask_list):
        """
        Retrieve visible objects in the ego camera.

        Parameters
        ----------
        vis_mask_list : list
        cav_content : dict
        """

        sensor_manager = cav_content['sensor_manager']
        meta_data = sensor_manager.sensor_meta
        for key, item in meta_data.items():
            if 'semantic_lidar' in key:
                vis_mask_list += item
        vis_mask_list = list(set(vis_mask_list))
        return vis_mask_list

    @staticmethod
    def get_bounds(left_lane, right_lane):
        """
        Get boundary information of a lane.

        Parameters
        ----------
        left_lane : np.array
            shape: (n, 3)
        right_lane : np.array
            shape: (n,3)
        Returns
        -------
        bound : np.array
        """
        x_min = min(np.min(left_lane[:, 0]),
                    np.min(right_lane[:, 0]))
        y_min = min(np.min(left_lane[:, 1]),
                    np.min(right_lane[:, 1]))
        z_min = min(np.min(left_lane[:, 2]),
                    np.min(right_lane[:, 2]))
        x_max = max(np.max(left_lane[:, 0]),
                    np.max(right_lane[:, 0]))
        y_max = max(np.max(left_lane[:, 1]),
                    np.max(right_lane[:, 1]))
        z_max = max(np.max(left_lane[:, 2]),
                    np.max(right_lane[:, 2]))

        bounds = np.asarray([[[x_min, y_min], [x_max, y_max], [z_min, z_max]]])

        return bounds

    def agents_in_range(self,
                        radius,
                        agents_dict):
        """
        Filter out all agents out of the radius.

        Parameters
        ----------
        radius : float
            Radius in meters

        agents_dict : dict
            Dictionary containing all dynamic agents.

        Returns
        -------
        The dictionary that only contains the agent in range.
        """
        final_agents = {}

        # convert center to list format
        center = [self.center.location.x, self.center.location.y]

        for agent_id, agent in agents_dict.items():
            location = agent['location']
            distance = math.sqrt((location[0] - center[0]) ** 2 + \
                                 (location[1] - center[1]) ** 2)
            if distance < radius:
                final_agents.update({agent_id: agent})

        return final_agents

    def indices_in_bounds(self,
                          bounds: np.ndarray,
                          half_extent: float) -> np.ndarray:
        """
        Get indices of elements for which the bounding box described by bounds
        intersects the one defined around center (square with side 2*half_side)

        Parameters
        ----------
        bounds :np.ndarray
            array of shape Nx2x2 [[x_min,y_min],[x_max, y_max]]

        half_extent : float
            half the side of the bounding box centered around center

        Returns
        -------
        np.ndarray: indices of elements inside radius from center
        """
        x_center, y_center, z_center = \
            self.center.location.x, \
            self.center.location.y, \
            self.center.location.z

        x_min_in = x_center > bounds[:, 0, 0] - half_extent
        y_min_in = y_center > bounds[:, 0, 1] - half_extent
        x_max_in = x_center < bounds[:, 1, 0] + half_extent
        y_max_in = y_center < bounds[:, 1, 1] + half_extent

        z_min_in = abs(z_center - bounds[:, 2, 0]) < self.z_filter_value
        z_max_in = abs(z_center - bounds[:, 2, 1]) < self.z_filter_value

        # this several scenes has some speciality and needs to be hardcoded
        if self.scene_name in OR_Z_VALUE_MAP:
            z_flag = z_min_in | z_max_in
        else:
            z_flag = z_min_in & z_max_in

        return np.nonzero(x_min_in & y_min_in & x_max_in & y_max_in
                          & z_flag)[0]

    def retrieve_light_stop_pos(self):
        # retrieve all stop and traffic light position
        all_actors = self.world.get_actors()
        for actor in all_actors:
            if 'traffic_light' in actor.type_id or 'stop' in actor.type_id:
                location = actor.get_transform().location
                self.traffic_stop_pos.append((location.x, location.y))

    def associate_lane_tl(self, mid_lane):
        """
        Given the waypoints for a certain lane, find the traffic light that
        influence it.

        Parameters
        ----------
        mid_lane : np.ndarray
            The middle line of the lane.
        Returns
        -------
        associate_tl_id : str
            The associated traffic light id.
        """
        associate_tl_id = ''

        for tl_id, tl_content in self.traffic_light_info.items():
            trigger_poly = tl_content['corners']
            # use Path to do fast computation
            trigger_path = Path(trigger_poly.boundary)
            # check if any point in the middle line inside the trigger area
            check_array = trigger_path.contains_points(mid_lane[:, :2])

            if check_array.any():
                associate_tl_id = tl_id
        return associate_tl_id

    def data_dump(self):
        """
        Dump the data to the corresponding folder.
        """
        save_name = os.path.join(self.out_root, self.agent_id)

        if not os.path.exists(save_name):
            os.makedirs(save_name)

        map_info = {'bev_map': {
            'raster_size_pixel': self.raster_size,
            'radius_meter': self.radius_meter,
            'pixel_per_meter': self.pixels_per_meter,
            'topology': self.valid_lane_info
        }}
        if self.save_yml:
            # save metadata
            save_yaml_name = os.path.join(save_name,
                                          self.current_timstamp +
                                          '_additional.yaml')
            save_yaml_wo_overwriting(map_info, save_yaml_name)

        if self.save_static:
            # save rgb image
            save_static_name = os.path.join(save_name,
                                            self.current_timstamp +
                                            '_bev_static.png')
            cv2.imwrite(save_static_name, self.static_bev)

        if self.save_lane:
            # save lane topology seg
            save_lane_name = os.path.join(save_name,
                                          self.current_timstamp +
                                          '_bev_lane.png')
            cv2.imwrite(save_lane_name, self.lane_bev)

        if self.save_dynamic:
            # save dynamic bev
            save_dynamic_name = os.path.join(save_name,
                                             self.current_timstamp +
                                             '_bev_dynamic.png')
            cv2.imwrite(save_dynamic_name, self.dynamic_bev)

        if self.visibility:
            # save  visibility map for ego agent
            save_visibility_name = os.path.join(save_name,
                                                self.current_timstamp +
                                                '_bev_visibility.png')
            cv2.imwrite(save_visibility_name, self.vis_mask)

        if self.visibility_corp:
            # save  visibility map for ego agent
            save_visibility_name = os.path.join(save_name,
                                                self.current_timstamp +
                                                '_bev_visibility_corp.png')
            cv2.imwrite(save_visibility_name, self.vis_corp_mask)

        if self.save_bev_vis:
            # save visualize bev
            save_vis_name = os.path.join(save_name,
                                         self.current_timstamp +
                                         '_bev_vis.png')
            cv2.imwrite(save_vis_name, self.vis_bev)

    def generate_lane_cross_info(self):
        """
        From the topology generate all lane and crosswalk
        information in a dictionary under world's coordinate frame.
        """
        # list of str
        lanes_id = []
        crosswalks_ids = []

        # boundary of each lane for later filtering
        lanes_bounds = np.empty((0, 3, 2), dtype=np.float)
        crosswalks_bounds = np.empty((0, 3, 2), dtype=np.float)

        # for crosswalk information
        crosswalks_list = self.split_cross_walks()
        for (i, crosswalk) in enumerate(crosswalks_list):
            crosswalk_id = uuid.uuid4().hex[:6].upper()
            crosswalks_ids.append(crosswalk_id)

            cross_marking = np.array(crosswalk)
            bound = self.get_bounds(cross_marking, cross_marking)
            crosswalks_bounds = np.append(crosswalks_bounds, bound, axis=0)

            self.crosswalk_info.update({crosswalk_id: {'xyz': cross_marking}})

        self.bound_info['crosswalks']['ids'] = crosswalks_ids
        self.bound_info['crosswalks']['bounds'] = crosswalks_bounds

        # loop all waypoints to get lane information
        for (i, waypoint) in enumerate(self.topology):
            # unique id for each lane
            lane_id = uuid.uuid4().hex[:6].upper()
            lanes_id.append(lane_id)

            intersection_flag = True if waypoint.is_intersection and \
                                        self.exclude_intersection_lane else False
            # we need to exclude the intersections without traffic light/stops
            if intersection_flag:
                intersection_flag = False
                for t_pos in self.traffic_stop_pos:
                    distance = np.sqrt((t_pos[0]-waypoint.transform.location.x)**2 +
                                       (t_pos[1]-waypoint.transform.location.y)**2)
                    if distance <= 30:
                        intersection_flag = True
                        break

            waypoints = [waypoint]
            nxt = waypoint.next(self.lane_sample_resolution)[0]
            # looping until next lane
            while nxt.road_id == waypoint.road_id \
                    and nxt.lane_id == waypoint.lane_id:
                waypoints.append(nxt)
                nxt = nxt.next(self.lane_sample_resolution)[0]

            # waypoint is the centerline, we need to calculate left lane mark
            left_marking = [lateral_shift(w.transform, -w.lane_width * 0.5) for
                            w in waypoints]
            right_marking = [lateral_shift(w.transform, w.lane_width * 0.5) for
                             w in waypoints]
            # convert the list of carla.Location to np.array
            left_marking = list_loc2array(left_marking)
            right_marking = list_loc2array(right_marking)
            mid_lane = list_wpt2array(waypoints)

            # get boundary information
            bound = self.get_bounds(left_marking, right_marking)
            lanes_bounds = np.append(lanes_bounds, bound, axis=0)

            # associate with traffic light
            tl_id = self.associate_lane_tl(mid_lane)

            self.lane_info.update({lane_id: {'xyz_left': left_marking,
                                             'xyz_right': right_marking,
                                             'xyz_mid': mid_lane,
                                             'tl_id': tl_id,
                                             'intersection_flag':
                                                 intersection_flag}})
        # boundary information
        self.bound_info['lanes']['ids'] = lanes_id
        self.bound_info['lanes']['bounds'] = lanes_bounds

    def split_cross_walks(self):
        """
        Find each crosswalk with their key points.
        """
        all_cross_walks = self.carla_map.get_crosswalks()
        cross_walks_list = []

        tmp_list = []
        for key_points in all_cross_walks:
            if (key_points.x, key_points.y, key_points.z) in tmp_list:
                cross_walks_list.append(tmp_list)
                tmp_list = []
            else:
                tmp_list.append((key_points.x, key_points.y, key_points.z))

        return cross_walks_list

    def generate_tl_info(self, world):
        """
        Generate traffic light information under world's coordinate frame.

        Parameters
        ----------
        world : carla.world
            Carla simulator env.

        Returns
        -------
        A dictionary containing traffic lights information.
        """
        tl_list = world.get_actors().filter('traffic.traffic_light*')

        for tl_actor in tl_list:
            tl_id = uuid.uuid4().hex[:4].upper()

            base_transform = tl_actor.get_transform()
            base_rot = base_transform.rotation.yaw
            # this is where the vehicle stops
            area_loc = base_transform.transform(
                tl_actor.trigger_volume.location)
            area_transform = carla.Transform(area_loc,
                                             carla.Rotation(yaw=base_rot))

            # bbx extent
            ext = tl_actor.trigger_volume.extent
            # the y is to narrow
            ext.y += 0.5
            ext_corners = np.array([
                [-ext.x, -ext.y],
                [ext.x, -ext.y],
                [ext.x, ext.y],
                [-ext.x, ext.y]])
            for i in range(ext_corners.shape[0]):
                corrected_loc = area_transform.transform(
                    carla.Location(ext_corners[i][0], ext_corners[i][1]))
                ext_corners[i, 0] = corrected_loc.x
                ext_corners[i, 1] = corrected_loc.y

            # use shapely lib to convert to polygon
            corner_poly = Polygon(ext_corners)
            self.traffic_light_info.update(
                {tl_id: {'actor': tl_actor,
                         'corners': corner_poly,
                         'base_rot': base_rot,
                         'base_transform': base_transform
                         }})

    def generate_lane_area(self, xyz_left, xyz_right):
        """
        Generate the lane area poly under rasterization map's center
        coordinate frame.

        Parameters
        ----------
        xyz_left : np.ndarray
            Left lanemarking of a lane, shape: (n, 3).
        xyz_right : np.ndarray
            Right lanemarking of a lane, shape: (n, 3).

        Returns
        -------
        lane_area : np.ndarray
            Combine left and right lane together to form a polygon.
        """
        lane_area = np.zeros((2, xyz_left.shape[0], 2))
        # convert coordinates to center's coordinate frame
        xyz_left = xyz_left.T
        xyz_left = np.r_[
            xyz_left, [np.ones(xyz_left.shape[1])]]
        xyz_right = xyz_right.T
        xyz_right = np.r_[
            xyz_right, [np.ones(xyz_right.shape[1])]]

        # ego's coordinate frame
        xyz_left = world_to_sensor(xyz_left, self.center).T
        xyz_right = world_to_sensor(xyz_right, self.center).T

        # to image coordinate frame
        lane_area[0] = xyz_left[:, :2]
        lane_area[1] = xyz_right[::-1, :2]
        # switch x and y
        lane_area = lane_area[..., ::-1]
        # y revert
        lane_area[:, :, 1] = -lane_area[:, :, 1]

        lane_area[:, :, 0] = lane_area[:, :, 0] * self.pixels_per_meter + \
                             self.raster_size[0] // 2
        lane_area[:, :, 1] = lane_area[:, :, 1] * self.pixels_per_meter + \
                             self.raster_size[1] // 2

        # to make more precise polygon
        lane_area = cv2_subpixel(lane_area)

        return lane_area

    def generate_cross_area(self, xyz):
        """
        Generate the cross lane under rasterization map's center
        coordinate frame.

        Parameters
        ----------
        xyz : np.ndarray
            Crosswalk lane marking, shape: (n, 3).
        Returns
        -------
        lane_area : np.ndarray
            Combine up and down line together.
        """
        if xyz.shape[0] == 2:
            print('dman')
        lane_area = np.zeros((2, xyz.shape[0] // 2, 2))
        # convert coordinates to center's coordinate frame
        xyz = xyz.T
        xyz = np.r_[
            xyz, [np.ones(xyz.shape[1])]]

        # ego's coordinate frame
        xyz = world_to_sensor(xyz, self.center).T

        # to image coordinate frame
        lane_area[0] = xyz[:xyz.shape[0] // 2, :2]
        lane_area[1] = xyz[xyz.shape[0] // 2:, :2]
        # switch x and y
        lane_area = lane_area[..., ::-1]
        # y revert
        lane_area[:, :, 1] = -lane_area[:, :, 1]

        lane_area[:, :, 0] = lane_area[:, :, 0] * self.pixels_per_meter + \
                             self.raster_size[0] // 2
        lane_area[:, :, 1] = lane_area[:, :, 1] * self.pixels_per_meter + \
                             self.raster_size[1] // 2
        # to make more precise polygon
        lane_area = cv2_subpixel(lane_area)

        return lane_area

    def generate_agent_area(self, corners):
        """
        Convert the agent's bbx corners from world coordinates to
        rasterization coordinates.

        Parameters
        ----------
        corners : list
            The four corners of the agent's bbx under world coordinate.

        Returns
        -------
        agent four corners in image.
        """
        # (4, 3) numpy array
        corners = np.array(corners)
        # for homogeneous transformation
        corners = corners.T
        corners = np.r_[
            corners, [np.ones(corners.shape[1])]]
        # convert to ego's coordinate frame
        corners = world_to_sensor(corners, self.center).T
        corners = corners[:, :2]

        # switch x and y
        corners = corners[..., ::-1]
        # y revert
        corners[:, 1] = -corners[:, 1]

        corners[:, 0] = corners[:, 0] * self.pixels_per_meter + \
                        self.raster_size[0] // 2
        corners[:, 1] = corners[:, 1] * self.pixels_per_meter + \
                        self.raster_size[1] // 2

        # to make more precise polygon
        corner_area = cv2_subpixel(corners[:, :2])

        return corner_area

    def load_agents_world(self):
        """
        Load all the dynamic agents info from server directly
        into a  dictionary.

        Returns
        -------
        The dictionary contains all agents info in the carla world.
        """

        agent_list = self.world.get_actors().filter('vehicle.*')
        dynamic_agent_info = {}

        for agent in agent_list:
            agent_id = agent.id

            agent_transform = agent.get_transform()
            agent_loc = [agent_transform.location.x,
                         agent_transform.location.y,
                         agent_transform.location.z, ]

            agent_yaw = agent_transform.rotation.yaw

            # calculate 4 corners
            bb = agent.bounding_box.extent
            corners = [
                carla.Location(x=-bb.x, y=-bb.y),
                carla.Location(x=-bb.x, y=bb.y),
                carla.Location(x=bb.x, y=bb.y),
                carla.Location(x=bb.x, y=-bb.y)
            ]
            # corners are originally in ego coordinate frame, convert to
            # world coordinate
            agent_transform.transform(corners)
            corners_reformat = [[x.x, x.y, x.z] for x in corners]

            dynamic_agent_info[agent_id] = {'location': agent_loc,
                                            'yaw': agent_yaw,
                                            'corners': corners_reformat}
        return dynamic_agent_info

    def rasterize_dynamic(self):
        """
        Rasterize the dynamic agents.

        Returns
        -------
        Rasterization image.
        """
        self.dynamic_bev = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.vis_mask = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.vis_corp_mask = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        # filter using half a radius from the center
        raster_radius = \
            float(np.linalg.norm(self.raster_size *
                                 np.array([self.meter_per_pixel,
                                           self.meter_per_pixel]))) / 2
        # retrieve all agents
        dynamic_agents = self.load_agents_world()
        # filter out agents out of range
        final_agents = self.agents_in_range(raster_radius,
                                            dynamic_agents)

        corner_list = []
        vis_corner_list = []
        vis_corp_corner_list = []
        for agent_id, agent in final_agents.items():
            # in case we don't want to draw the cav itself
            if agent_id == self.actor_id and self.exclude_self:
                continue
            agent_corner = self.generate_agent_area(agent['corners'])
            corner_list.append(agent_corner)

            # get visibility for single vehicle
            if agent_id in self.vis_ids:
                vis_corner_list.append(agent_corner)
            # get visibility for multi-agents
            if agent_id in self.vis_corp_ids:
                vis_corp_corner_list.append(agent_corner)

        self.dynamic_bev = draw_agent(corner_list, self.dynamic_bev)
        self.vis_bev = draw_agent(corner_list, self.vis_bev)
        self.vis_mask = draw_agent(vis_corner_list, self.vis_mask)
        self.vis_corp_mask = draw_agent(vis_corp_corner_list,
                                        self.vis_corp_mask)

    def rasterize_static(self):
        """
        Generate the static bev map.
        """
        self.static_bev = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.lane_bev = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.vis_bev = 255 * np.ones(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)

        # filter using half a radius from the center
        raster_radius = \
            float(np.linalg.norm(self.raster_size *
                                 np.array([self.meter_per_pixel,
                                           self.meter_per_pixel]))) / 2
        lane_indices = self.indices_in_bounds(self.bound_info['lanes'][
                                                  'bounds'],
                                              raster_radius)
        cross_indices = self.indices_in_bounds(self.bound_info['crosswalks'][
                                                  'bounds'],
                                              raster_radius)

        # --------- Retrieve road topology related first ---------- #
        lanes_area_list = []
        cross_area_list = []
        lane_type_list = []
        intersection_list = []

        for idx, cross_idx in enumerate(cross_indices):
            cross_idx = self.bound_info['crosswalks']['ids'][cross_idx]
            cross_info = self.crosswalk_info[cross_idx]

            cross_area = self.generate_cross_area(cross_info['xyz'])
            cross_area_list.append(cross_area)

        for idx, lane_idx in enumerate(lane_indices):
            lane_idx = self.bound_info['lanes']['ids'][lane_idx]
            lane_info = self.lane_info[lane_idx]

            # save the topology info for this cav
            self.valid_lane_info = lane_info

            xyz_left, xyz_right = \
                lane_info['xyz_left'], lane_info['xyz_right']

            # generate lane area
            lane_area = self.generate_lane_area(xyz_left, xyz_right)
            lanes_area_list.append(lane_area)

            # intersection flag
            intersection_list.append(lane_info['intersection_flag'])

            # check the associated traffic light
            associated_tl_id = lane_info['tl_id']
            if associated_tl_id and self.draw_traffic_light:
                tl_actor = self.traffic_light_info[associated_tl_id]['actor']
                status = convert_tl_status(tl_actor.get_state())
                lane_type_list.append(status)
            else:
                lane_type_list.append('normal')

        self.static_bev = draw_road(lanes_area_list,
                                    self.static_bev)
        if self.exclude_road:
            self.static_bev = road_exclude(self.static_bev)

        if self.draw_lane:
            self.lane_bev = draw_lane(lanes_area_list, lane_type_list,
                                      self.lane_bev, intersection_list,
                                      vis=False)
            self.lane_bev = draw_crosswalks(cross_area_list, self.lane_bev)
            self.lane_bev[self.static_bev == 0] = 0

        # --------- Retrieve city objects such as buildings ---------- #
        # first filter out objects out of range
        final_city_objs = obj_in_range(self.center,
                                       raster_radius,
                                       self.other_objs_info)
        for obj_cat, obj_content in final_city_objs.items():
            for obj_id, obj in obj_content.items():
                corner_area = self.generate_agent_area(obj['corners'])
                obj['corner_area'] = corner_area

        # we try to draw everything for visualization, but only dumping the
        # elements we need.
        self.vis_bev = draw_city_objects(final_city_objs, self.vis_bev)
        self.vis_bev = draw_road(lanes_area_list,
                                 self.vis_bev,
                                 visualize=True)
        self.vis_bev = draw_lane(lanes_area_list, lane_type_list,
                                 self.vis_bev, intersection_list)
        self.vis_bev = draw_crosswalks(cross_area_list, self.vis_bev)
        self.vis_bev = cv2.cvtColor(self.vis_bev, cv2.COLOR_RGB2BGR)

    def destroy(self):
        cv2.destroyAllWindows()
