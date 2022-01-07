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

from opencda.core.map.map_utils import \
    lateral_shift, list_loc2array, list_wpt2array, convert_tl_status
from opencda.core.map.map_drawing import cv2_subpixel, draw_road, draw_lane


class MapManager(object):
    """
    The class to manage HD Map. We emulate the style of Lyft dataset.

    Parameters
    ----------
    carla_world : Carla.world
        The carla simulator environment.

    carla_map : Carla.Map
        The carla simulator map.

    pixels_per_meter : int
        pixel/m

    Attributes
    ----------
    topology : list
        Map topology in list.
    """

    def __init__(self, carla_world, carla_map, pixels_per_meter):
        self.world = carla_world
        self.meter_per_pixel = 1 / pixels_per_meter
        self.pixels_per_meter = pixels_per_meter
        # todo: hard coded for now
        self.raster_size = np.array([224, 224])
        self.intepolation_point = 300

        # list of all start waypoints in HD Map
        topology = [x[0] for x in carla_map.get_topology()]
        # sort by altitude
        self.topology = sorted(topology, key=lambda w: w.transform.location.z)

        # basic elements in HDMap: lane, crosswalk and traffic light
        # todo: stop sign is not considered in the current version
        self.lane_info = {}
        self.crosswalk_info = {}
        self.traffic_light_info = {}
        # this is mainly used for efficient filtering
        self.bound_info = {'lanes': {},
                           'crosswalks': {}}

        # generate information for traffic light
        self.generate_tl_info(carla_world)
        # generate lane, crosswalk and boundary information
        self.generate_lane_cross_info()

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

    @staticmethod
    def indices_in_bounds(center: np.ndarray,
                          bounds: np.ndarray,
                          half_extent: float) -> np.ndarray:
        """
        Get indices of elements for which the bounding box described by bounds
        intersects the one defined around center (square with side 2*half_side)

        Parameters
        ----------
        center : np.ndarray
            xy of the center in carla world coordinate

        bounds :np.ndarray
            array of shape Nx2x2 [[x_min,y_min],[x_max, y_max]]

        half_extent : float
            half the side of the bounding box centered around center

        Returns
        -------
        np.ndarray: indices of elements inside radius from center
        """
        x_center, y_center = center

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
        # todo: debug purpose only
        if associate_tl_id:
            for i in range(mid_lane.shape[0]):
                np_loc = mid_lane[i]
                carla_loc = carla.Location(x=np_loc[0],
                                           y=np_loc[1],
                                           z=1)
                self.world.debug.draw_point(carla_loc,
                                            size=0.05,
                                            life_time=120,
                                            color=carla.Color(0, 255, 0))
        return associate_tl_id

    def generate_lane_cross_info(self):
        """
        From the topology generate all lane and crosswalk
        information in a dictionary.
        """
        # list of str
        lanes_id = []
        crosswalks_ids = []

        # boundary of each lane for later filtering
        lanes_bounds = np.empty((0, 2, 2), dtype=np.float)
        crosswalks_bounds = np.empty((0, 2, 2), dtype=np.float)

        # loop all waypoints to get lane information
        # todo: seperate lane and crosswalk later
        for (i, waypoint) in enumerate(self.topology):
            # unique id for each lane
            lane_id = uuid.uuid4().hex[:6].upper()
            # todo: seperate lane and crosswalk later
            lanes_id.append(lane_id)

            waypoints = [waypoint]
            # todo sample resolution hard-coded
            nxt = waypoint.next(0.1)[0]
            while nxt.road_id == waypoint.road_id \
                    and nxt.lane_id == waypoint.lane_id:
                waypoints.append(nxt)
                nxt = nxt.next(0.1)[0]

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

            # todo: crosswalk add later
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
        Generate traffic light information.

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
                # todo: debug purpose now
                world.debug.draw_point(corrected_loc + carla.Location(z=1),
                                       size=0.2, life_time=120)

            # use shapely lib to convert to polygon
            corner_poly = Polygon(ext_corners)
            self.traffic_light_info.update({tl_id: {'actor': tl_actor,
                                                    'corners': corner_poly,
                                                    'base_rot': base_rot,
                                                    'base_transform': base_transform
                                                    }})

    def generate_lane_area(self, center, xyz_left, xyz_right):
        """
        Generate the lane area poly.

        Parameters
        ----------
        center : np.ndarray
            Center of the raster.
        xyz_left : np.ndarray
            Left lanemarking of a lane.
        xyz_right : np.ndarray
            Right lanemarking of a lane.

        Returns
        -------
        lane_area : np.ndarray
            Combine left and right lane together to form a polygon.
        """
        lane_area = np.zeros((2, xyz_left.shape[0], 2))
        lane_area[0] = xyz_left[:, :2]
        lane_area[1] = xyz_right[::-1, :2]
        # todo tmp translation, need to be more organized
        lane_area[:, :, 0] = \
            (lane_area[:, :, 0] - center[0]) * self.pixels_per_meter + \
            self.raster_size[1] // 2
        lane_area[:, :, 1] = \
            (lane_area[:, :, 1] - center[1]) * self.pixels_per_meter + \
            self.raster_size[0] // 2
        lane_area = cv2_subpixel(lane_area)

        return lane_area

    def rasterize(self, center):
        """
        Todo: only for unit testing for now
        """
        img = 255 * np.ones(
            shape=(self.raster_size[1], self.raster_size[0], 3),
            dtype=np.uint8)
        # filter using half a radius from the center
        raster_radius = float(np.linalg.norm(self.raster_size *
                                             np.array([self.meter_per_pixel,
                                                       self.meter_per_pixel]))) / 2
        lane_indices = self.indices_in_bounds(center,
                                              self.bound_info['lanes'][
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
            lane_area = self.generate_lane_area(center, xyz_left, xyz_right)
            lanes_area_list.append(lane_area)

            # check the associated traffic light
            associated_tl_id = lane_info['tl_id']
            if associated_tl_id:
                tl_actor = self.traffic_light_info[associated_tl_id]['actor']
                status = convert_tl_status(tl_actor.get_state())
                lane_type_list.append(status)
            else:
                lane_type_list.append('normal')

        img = draw_road(lanes_area_list, img)
        img = draw_lane(lanes_area_list, lane_type_list, img)
        # todo: debug purpose
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow('debug', img)
        cv2.waitKey(0)
