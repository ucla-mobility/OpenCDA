# -*- coding: utf-8 -*-

"""HDMap manager
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import uuid

import cv2
import carla
import numpy as np
from matplotlib.path import Path
from shapely.geometry import Polygon

from opencda.core.sensing.perception.sensor_transformation import \
    world_to_sensor
from opencda.core.map.map_utils import \
    lateral_shift, list_loc2array, list_wpt2array, convert_tl_status
from opencda.core.map.map_drawing import \
    cv2_subpixel, draw_agent, draw_road, draw_lane


class MapManager(object):
    """
    This class is used to manage HD Map. We emulate the style of Lyft dataset.
    todo: Currently mainly used for map rasterization.
    todo: everything is groundtruth loaded from server directly.

    Parameters
    ----------
    vehicle : Carla.vehicle
        The ego vehicle.

    carla_map : Carla.Map
        The carla simulator map.

    config : dict
        All the map manager parameters.

    Attributes
    ----------
    world : carla.world
        Carla simulation world.

    center : carla.Transform
        The rasterization map's center pose.

    meter_per_pixel : float
        m/pixel

    raster_size : float
        The rasterization map size in pixels.

    raster_radius : float
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

    static_bev : np.array
        The static bev map containing lanes and drivable road information.

    dynamic_bev : np.array
        The dynamic bev map containing vehicle's information.

    vis_bev : np.array
        The comprehensive bev map for visualization.

    """

    def __init__(self, vehicle, carla_map, config):
        self.world = vehicle.get_world()
        self.agent_id = vehicle.id
        self.carla_map = carla_map
        self.center = None

        self.actvate = config['activate']
        self.visualize = config['visualize']
        self.pixels_per_meter = config['pixels_per_meter']
        self.meter_per_pixel = 1 / self.pixels_per_meter
        self.raster_size = np.array([config['raster_size'][0],
                                     config['raster_size'][1]])
        self.lane_sample_resolution = config['lane_sample_resolution']

        self.raster_radius = \
            float(np.linalg.norm(self.raster_size *
                                 np.array(
                                     [self.meter_per_pixel,
                                      self.meter_per_pixel]))) / 2

        # list of all start waypoints in HD Map
        topology = [x[0] for x in carla_map.get_topology()]
        # sort by altitude
        self.topology = sorted(topology, key=lambda w: w.transform.location.z)

        # basic elements in HDMap: lane, crosswalk and traffic light
        self.lane_info = {}
        self.crosswalk_info = {}
        self.traffic_light_info = {}
        # this is mainly used for efficient filtering
        self.bound_info = {'lanes': {},
                           'crosswalks': {}}

        # generate information for traffic light
        self.generate_tl_info(self.world)
        # generate lane, crosswalk and boundary information
        self.generate_lane_cross_info()

        # bev maps
        self.dynamic_bev = 255 * np.zeros(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.static_bev = 255 * np.ones(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        self.vis_bev = 255 * np.ones(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)

    def update_information(self, ego_pose):
        """
        Update the ego pose as the map center.

        Parameters
        ----------
        ego_pose : carla.Transform
        """
        self.center = ego_pose

    def run_step(self):
        """
        Rasterization + Visualize the bev map if needed.
        """
        if not self.actvate:
            return
        self.rasterize_static()
        self.rasterize_dynamic()
        if self.visualize:
            cv2.imshow('the bev map of agent %s' % self.agent_id,
                       self.vis_bev)
            cv2.waitKey(1)

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
        x_max = max(np.max(left_lane[:, 0]),
                    np.max(right_lane[:, 0]))
        y_max = max(np.max(left_lane[:, 1]),
                    np.max(right_lane[:, 1]))
        bounds = np.asarray([[[x_min, y_min], [x_max, y_max]]])

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
        x_center, y_center = self.center.location.x, self.center.location.y

        x_min_in = x_center > bounds[:, 0, 0] - half_extent
        y_min_in = y_center > bounds[:, 0, 1] - half_extent
        x_max_in = x_center < bounds[:, 1, 0] + half_extent
        y_max_in = y_center < bounds[:, 1, 1] + half_extent
        return np.nonzero(x_min_in & y_min_in & x_max_in & y_max_in)[0]

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

    def generate_lane_cross_info(self):
        """
        From the topology generate all lane and crosswalk
        information in a dictionary under world's coordinate frame.
        """
        # list of str
        lanes_id = []
        crosswalks_ids = []

        # boundary of each lane for later filtering
        lanes_bounds = np.empty((0, 2, 2), dtype=np.float)
        crosswalks_bounds = np.empty((0, 2, 2), dtype=np.float)

        # loop all waypoints to get lane information
        for (i, waypoint) in enumerate(self.topology):
            # unique id for each lane
            lane_id = uuid.uuid4().hex[:6].upper()
            lanes_id.append(lane_id)

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
                                             'tl_id': tl_id}})
            # boundary information
            self.bound_info['lanes']['ids'] = lanes_id
            self.bound_info['lanes']['bounds'] = lanes_bounds
            self.bound_info['crosswalks']['ids'] = crosswalks_ids
            self.bound_info['crosswalks']['bounds'] = crosswalks_bounds

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
        for agent_id, agent in final_agents.items():
            agent_corner = self.generate_agent_area(agent['corners'])
            corner_list.append(agent_corner)

        self.dynamic_bev = draw_agent(corner_list, self.dynamic_bev)
        self.vis_bev = draw_agent(corner_list, self.vis_bev)

    def rasterize_static(self):
        """
        Generate the static bev map.
        """
        self.static_bev = 255 * np.ones(
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
        lanes_area_list = []
        lane_type_list = []

        for idx, lane_idx in enumerate(lane_indices):
            lane_idx = self.bound_info['lanes']['ids'][lane_idx]
            lane_info = self.lane_info[lane_idx]
            xyz_left, xyz_right = \
                lane_info['xyz_left'], lane_info['xyz_right']

            # generate lane area
            lane_area = self.generate_lane_area(xyz_left, xyz_right)
            lanes_area_list.append(lane_area)

            # check the associated traffic light
            associated_tl_id = lane_info['tl_id']
            if associated_tl_id:
                tl_actor = self.traffic_light_info[associated_tl_id]['actor']
                status = convert_tl_status(tl_actor.get_state())
                lane_type_list.append(status)
            else:
                lane_type_list.append('normal')

        self.static_bev = draw_road(lanes_area_list,
                                    self.static_bev)
        self.static_bev = draw_lane(lanes_area_list, lane_type_list,
                                    self.static_bev)

        self.vis_bev = draw_road(lanes_area_list,
                                 self.vis_bev)
        self.vis_bev = draw_lane(lanes_area_list, lane_type_list,
                                 self.vis_bev)
        self.vis_bev = cv2.cvtColor(self.vis_bev, cv2.COLOR_RGB2BGR)

    def destroy(self):
        cv2.destroyAllWindows()