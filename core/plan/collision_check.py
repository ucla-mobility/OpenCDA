# -*- coding: utf-8 -*-
""" This module is used to check collision possibility """

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from math import sin, cos
from scipy import spatial

import carla
import numpy as np

from core.common.misc import cal_distance_angle, draw_trajetory_points
from core.plan.spline import Spline2D


class CollisionChecker:
    def __init__(self, time_ahead=1.2, circle_radius=1.3, circle_offsets=None):
        """
        Construction method
        :param time_ahead: how many seconds we look ahead in advance for collision check
        :param circle_offsets: the offset between collision checking circle and the trajectory point
        :param circle_radius: The radius of the collision checking circle
        """
        self.time_ahead = time_ahead
        self._circle_offsets = [-1.0, 1.0, -3.0, 3.0] if circle_offsets is None else circle_offsets
        self._circle_radius = circle_radius

    def is_in_range(self, ego_vehicle, target_vehicle, candidate_vehicle, carla_map):
        """
        Check whether there is a obstacle vehicle between target_vehicle and ego_vehicle during back_joining
        :param carla_map: carla map
        :param ego_vehicle: The vehicle trying to join platooning
        :param target_vehicle: The vehicle that is suppose to be catched
        :param candidate_vehicle: The possible obstacle vehicle blocking the ego vehicle and target vehicle
        :return:
        """
        ego_loc = ego_vehicle.get_location()
        target_loc = target_vehicle.get_location()
        candidate_loc = candidate_vehicle.get_location()

        # set the checking rectangle
        min_x, max_x = min(ego_loc.x, target_loc.x), max(ego_loc.x, target_loc.x)
        min_y, max_y = min(ego_loc.y, target_loc.y), max(ego_loc.y, target_loc.y)

        # give a small buffer of 2 meters
        if candidate_loc.x <= min_x - 2 or candidate_loc.x >= max_x + 2 or \
                candidate_loc.y <= min_y - 2 or candidate_loc.y >= max_y + 2:
            return False

        ego_wpt = carla_map.get_waypoint(ego_loc)
        candidate_wpt = carla_map.get_waypoint(candidate_loc)
        target_wpt = carla_map.get_waypoint(target_loc)

        # if the candidate vehicle is right behind the target vehicle, then it is blocking
        if target_wpt.lane_id == candidate_wpt.lane_id:
            return True

        # in case they are in the same lane but section id is different which changes their id
        if target_wpt.section_id == candidate_wpt.section_id:
            return False

        # check the angle
        distance, angle = cal_distance_angle(target_wpt.transform.location, candidate_wpt.transform.location,
                                             candidate_wpt.transform.rotation.yaw)

        return True if angle <= 3 else False

    def overtake_collision_path(self, ego_loc, target_wpt, world):
        """
        Generate a rough path to be used for collicion check for overtaking
        :param world: carla world
        :param ego_loc:
        :param target_wpt:
        :return:
        """
        # we first need to consider the vehicle on the other lane in front
        target_wpt_next = target_wpt.next(10)[0]

        # Next we consider the vehicle behind us
        diff_x = target_wpt_next.transform.location.x - ego_loc.x
        diff_y = target_wpt_next.transform.location.y - ego_loc.y
        diff_s = np.hypot(diff_x, diff_y)

        target_wpt_previous = target_wpt.previous(diff_s)[0]

        x, y = [target_wpt_next.transform.location.x, target_wpt_previous.transform.location.x], \
               [target_wpt_next.transform.location.y, target_wpt_previous.transform.location.y]
        ds = 0.1

        sp = Spline2D(x, y)
        s = np.arange(sp.s[0], sp.s[-1], ds)

        debug_tmp = []

        # calculate interpolation points
        rx, ry, ryaw = [], [], []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(sp.calc_yaw(i_s))
            debug_tmp.append(carla.Transform(carla.Location(ix, iy, 0)))

        # TODO: Remove this after debugging
        draw_trajetory_points(world,
                              debug_tmp,
                              color=carla.Color(255, 255, 0),
                              size=0.05,
                              lt=0.1)

        return rx, ry, ryaw

    def collision_circle_check(self, path_x, path_y, path_yaw, obstacle_vehicle, speed, overtake_check=False):
        """
        Use circled collision check to see whether potential hazard on the forwarding path
        :param overtake_check: always give full path for overtaking check
        :param speed: ego vehicle speed in m/s
        :param path_yaw: a list of yaw angles
        :param path_x: a list of x coordinates
        :param path_y: a loist of y coordinates
        :param obstacle_vehicle: potention hazard vehicle on the way
        :return:
        """
        collision_free = True
        # detect 2 second ahead
        distance_check = min(int(self.time_ahead * speed / 0.1), len(path_x)) if not overtake_check else len(path_x)
        obstacle_vehicle_loc = obstacle_vehicle.get_location()
        
        # every step is 0.1m, so we check every 10 points
        for i in range(0, distance_check, 10):
            ptx, pty, yaw = path_x[i], path_y[i], path_yaw[i]

            # circle_x = point_x + circle_offset*cos(yaw), circle_y = point_y + circle_offset*sin(yaw)
            circle_locations = np.zeros((len(self._circle_offsets), 2))
            circle_offsets = np.array(self._circle_offsets)
            circle_locations[:, 0] = ptx + circle_offsets * cos(yaw)
            circle_locations[:, 1] = pty + circle_offsets * sin(yaw)

            # we need compute the four corner of the bbx
            obstacle_vehicle_bbx_array = np.array([[obstacle_vehicle_loc.x - obstacle_vehicle.bounding_box.extent.x,
                                                    obstacle_vehicle_loc.y - obstacle_vehicle.bounding_box.extent.y],
                                                   [obstacle_vehicle_loc.x - obstacle_vehicle.bounding_box.extent.x,
                                                    obstacle_vehicle_loc.y + obstacle_vehicle.bounding_box.extent.y],
                                                   [obstacle_vehicle_loc.x + obstacle_vehicle.bounding_box.extent.x,
                                                    obstacle_vehicle_loc.y - obstacle_vehicle.bounding_box.extent.y],
                                                   [obstacle_vehicle_loc.x + obstacle_vehicle.bounding_box.extent.x,
                                                    obstacle_vehicle_loc.y + obstacle_vehicle.bounding_box.extent.y]])

            # compute whether the distance between the four corners of the vehicle to the trajectory point
            collision_dists = spatial.distance.cdist(obstacle_vehicle_bbx_array, circle_locations)
            collision_dists = np.subtract(collision_dists, self._circle_radius)
            collision_free = collision_free and not np.any(collision_dists < 0)

            if not collision_free:
                break

        return collision_free
