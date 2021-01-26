# -*- coding: utf-8 -*-

"""Behavior manager for platooning specifically
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla
import numpy as np

from core.agents.navigation.behavior_agent import BehaviorAgent
from core.agents.tools.misc import compute_distance, get_speed, positive


class PlatooningBehaviorAgent(BehaviorAgent):
    """
    The behavior agent for platooning
    """

    def __init__(self, vehicle, ignore_traffic_light=False, behavior='normal',
                 sampling_resolution=4.5, buffer_size=5, dynamic_pid=False):
        """
        Construct class
        :param vehicle: actor
        :param ignore_traffic_light: whether to ignore certain traffic light
        :param behavior: driving style
        :param sampling_resolution: the minimum distance between each waypoint
        :param buffer_size: buffer size for local route
        :param dynamic_pid; whether to use dynamic pid params generation. Set to true will require users
        provide customized function under customize/controller
        """

        super(PlatooningBehaviorAgent, self).__init__(vehicle, ignore_traffic_light, behavior, sampling_resolution,
                                                      buffer_size, dynamic_pid)

    def run_step_following(self):
        """
        Behavior planning for following car in the platooning
        :return:
        """

        frontal_vehicle = self.frontal_vehicle.vehicle
        # TODO: Aviod access to protected member
        frontal_agent_target_road_option = \
            self.frontal_vehicle.agent._local_planner.target_road_option
        frontal_vehicle_loc = frontal_vehicle.get_location()
        ego_vehicle_loc = self.vehicle.get_location()

        distance = compute_distance(ego_vehicle_loc, frontal_vehicle_loc)
        # Distance is computed from the center of the two cars,
        # use bounding boxes to calculate the actual distance
        distance = distance - max(
            frontal_vehicle.bounding_box.extent.y, frontal_vehicle.bounding_box.extent.x) - max(
            self.vehicle.bounding_box.extent.y, self.vehicle.bounding_box.extent.x)

        # safe control for car following
        if distance <= self.behavior.braking_distance:
            print("emergency stop!")
            return self.emergency_stop()

        control = self.platooning_following_manager(frontal_vehicle, distance,
                                                    frontal_vehicle.get_transform(), frontal_agent_target_road_option)

        return control

    def platooning_following_manager(self, vehicle, distance,
                                     vehicle_loc, vehicle_target_road_option, debug=False):
        """
        Car following behavior in platooning
        :param vehicle_target_road_option:
        :param vehicle_loc:
        :param vehicle: car to follow
        :param distance: distance from vehicle
        :param debug: boolean for debugging
        :return: control: carla.VehicleControl
        """
        vehicle_speed = get_speed(vehicle)
        delta_v = max(1, (self.speed - vehicle_speed) / 3.6)
        ttc = distance / delta_v if delta_v != 0 else distance / np.nextafter(0., 1.)

        # too close to the frontal vehicle, slow down
        if self.behavior.inter_gap > ttc > 0.0:
            # print("too close!")
            control = self._local_planner.run_step(
                target_speed=positive(vehicle_speed - self.behavior.speed_decrease),
                target_waypoint=vehicle_loc,
                target_road_option=vehicle_target_road_option,
                debug=debug)
        # in the safe following area
        elif 3 * self.behavior.inter_gap > ttc > self.behavior.inter_gap:
            control = self._local_planner.run_step(target_speed=max(self.min_speed, vehicle_speed),
                                                   target_waypoint=vehicle_loc,
                                                   target_road_option=vehicle_target_road_option,
                                                   debug=debug)
        #  print("keep distance!!!!!!!!!, speed: %d" % (max(self.min_speed, vehicle_speed)))
        # too far, tailgating
        else:

            control = self._local_planner.run_step(target_speed=self.behavior.tailgate_speed,
                                                   target_waypoint=vehicle_loc,
                                                   target_road_option=vehicle_target_road_option,
                                                   debug=debug)
        # print("tailgating!!!!!!!!!!!, ttc: %f, speed: %d" % (ttc, self.behavior.tailgate_speed))
        return control
