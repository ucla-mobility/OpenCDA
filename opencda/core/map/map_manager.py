# -*- coding: utf-8 -*-

"""HDMap manager
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import numpy as np
import uuid

from opencda.core.map.map_utils import \
    lateral_shift, list_loc2array, list_wpt2array


class MapManager(object):
    """
    The class to manage HD Map. We emulate the style of Lyft dataset.

    Parameters
    ----------
    carla_map : Carla.Map
        The carla simulator map.

    pixels_per_meter : int
        pixel/m

    Attributes
    ----------
    topology : list
        Map topology in list.
    """

    def __init__(self, carla_map, pixels_per_meter):
        self.meter_per_pixel = 1 / pixels_per_meter
        self.meter_per_pixel = pixels_per_meter
        # todo: hard coded for now
        self.raster_size = 224

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

        lanes_id, lanes_boundary, crosswalk_id, crosswalk_boundary = \
            self.generate_lane_cross_dict()
        self.bound_info['lanes']['ids'] = lanes_id
        self.bound_info['lanes']['bounds'] = lanes_boundary
        self.bound_info['crosswalks']['ids'] = crosswalk_id
        self.bound_info['crosswalks']['bounds'] = crosswalk_boundary

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

    def indices_in_bounds(self,
                          center: np.ndarray,
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


    def generate_lane_cross_dict(self):
        """
        From the topology generate all lane and crosswalk
        information in a dictionary.

        Returns
        -------
        lanes_id : list
            The list of lane object uuids.
        lane_bounds : np.array
            Lane boundary information.
        crosswalks_ids : list
            The list of crosswolk object uuids.
        crosswalks_bounds : np.array
            crosswalk boundary information.
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

            # todo: crosswalk add later
            self.lane_info.update({lane_id: {'xyz_left': left_marking,
                                             'xyz_right': right_marking,
                                             'xyz_mid': mid_lane}})
        return lanes_id, lanes_bounds, crosswalks_ids, crosswalks_bounds


