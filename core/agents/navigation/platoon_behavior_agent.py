# -*- coding: utf-8 -*-

"""Behavior manager for platooning specifically
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import numpy as np

from core.agents.navigation.behavior_agent import BehaviorAgent
from core.agents.tools.misc import compute_distance, get_speed, positive, cal_distance_angle
from core.platooning.fsm import FSM


class PlatooningBehaviorAgent(BehaviorAgent):
    """
    The behavior agent for platooning
    """

    def __init__(self, vehicle, ignore_traffic_light=False, behavior='normal',
                 sampling_resolution=4.5, buffer_size=5, dynamic_pid=False,
                 move_to_point_distance=5, debug_trajectory=False, debug=False):
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
                                                      buffer_size, dynamic_pid, debug_trajectory, debug)
        self._move_to_point_distance = move_to_point_distance

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
                target_road_option=vehicle_target_road_option)
        # in the safe following area
        elif 3 * self.behavior.inter_gap > ttc > self.behavior.inter_gap:
            control = self._local_planner.run_step(target_speed=max(self.min_speed, vehicle_speed),
                                                   target_waypoint=vehicle_loc,
                                                   target_road_option=vehicle_target_road_option)
            # print("keep distance!!!!!!!!!, speed: %d" % (max(self.min_speed, vehicle_speed)))
        # too far, tailgating
        else:

            control = self._local_planner.run_step(target_speed=self.behavior.tailgate_speed,
                                                   target_waypoint=vehicle_loc,
                                                   target_road_option=vehicle_target_road_option)
            # print("tailgating!!!!!!!!!!!, ttc: %f, speed: %d" % (ttc, self.behavior.tailgate_speed))
        return control

    def platooning_merge_management(self, frontal_vehicle_vm):
        """
        Merge the vehicle into the platooning
        :param frontal_vehicle_vm:
        :return:
        """
        print("start merging !")
        frontal_vehicle_transform = frontal_vehicle_vm.vehicle.get_location()

        # retrieve the platooning's destination
        _, _, platooning_manager = frontal_vehicle_vm.get_platooning_status()
        destination = platooning_manager.destination

        # regenerate route the route to make merge(lane change)
        self.set_destination(frontal_vehicle_transform, destination, clean=True)

        control = self._local_planner.run_step(target_speed=0.8 * get_speed(frontal_vehicle_vm.vehicle))

        return control

    def run_step_maintaining(self, frontal_vehicle=None):
        """
        Behavior planning for speed maintaining
        :return:
        """
        if not self.frontal_vehicle:
            self.frontal_vehicle = frontal_vehicle

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

    def run_step_move2point(self, frontal_vehicle_vm, rear_vehicle_vm=None):
        """
        TODO: right now we only consider the vehicle is sitting next to the platooning lane
        The vehicle is trying to get to the move in point
        :param frontal_vehicle_vm:
        :param rear_vehicle_vm:
        :return: control command and whether it meets to the point
        """
        frontal_vehicle = frontal_vehicle_vm.vehicle

        ego_vehicle_loc = self.vehicle.get_location()
        ego_vehicle_yaw = self.vehicle.get_transform().rotation.yaw

        distance, angle = cal_distance_angle(frontal_vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)

        # if the ego vehicle is still too far away
        if distance > get_speed(frontal_vehicle, True) * 1.5:
            print('trying to get the vehicle')
            return self.run_step(1.5 * get_speed(frontal_vehicle)), False

        # if the ego vehicle is too close or exceed the frontal vehicle
        if distance < get_speed(frontal_vehicle, True) * 1.0 or angle >= 70:
            print('too close, step back!')
            return self.run_step(0.9 * get_speed(frontal_vehicle)), False

        # communicate to the rear vehicle for open gap
        if not rear_vehicle_vm:
            return self.platooning_merge_management(frontal_vehicle_vm), True

        rear_vehicle_vm.set_platooning_status(FSM.OPEN_GAP)

        distance, angle = cal_distance_angle(rear_vehicle_vm.vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)
        if distance < get_speed(rear_vehicle_vm.vehicle, True) or angle <= 90:
            return self.run_step(1.0 * get_speed(frontal_vehicle)), False

        return self.platooning_merge_management(frontal_vehicle_vm), True

    def run_step_joining(self, frontal_vehicle_vm):
        """
        Check if the vehicle has been joined succusfully
        :param frontal_vehicle_vm:
        :return:
        """

        frontal_vehicle = frontal_vehicle_vm.vehicle
        frontal_lane = self._map.get_waypoint(frontal_vehicle.get_location()).lane_id

        ego_vehicle_loc = self.vehicle.get_location()
        ego_vehicle_lane = self._map.get_waypoint(ego_vehicle_loc).lane_id
        ego_vehicle_yaw = self.vehicle.get_transform().rotation.yaw

        distance, angle = cal_distance_angle(frontal_vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)

        if frontal_lane == ego_vehicle_lane and angle <= 10:
            return self.run_step_maintaining(frontal_vehicle_vm), True

        return self._local_planner.run_step(target_speed=get_speed(frontal_vehicle)), False
