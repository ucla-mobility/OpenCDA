# -*- coding: utf-8 -*-

"""Behavior manager for platooning specifically
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from collections import deque

import carla
import numpy as np

from core.agents.navigation.behavior_agent import BehaviorAgent, RoadOption
from core.agents.tools.misc import compute_distance, get_speed, positive, cal_distance_angle
from core.platooning.fsm import FSM


class PlatooningBehaviorAgent(BehaviorAgent):
    """
    The behavior agent for platooning
    """

    def __init__(self, vehicle, ignore_traffic_light=True, behavior='normal',
                 sampling_resolution=4.5, buffer_size=5, dynamic_pid=False,
                 move_to_point_distance=5, debug_trajectory=True, debug=True):
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
        # used to see the average time gap between ego and frontal vehicle
        self.time_gap_list = []
        self.velocity_list = []

    def platooning_folloing_manager(self):
        """
        Car following behavior in platooning with gap regulation
        :return:
        """
        # must match leading vehicle's trajectory unit time
        t_origin = 0
        if len(self._local_planner.get_trajetory()) > 7:
            return self._local_planner.run_step()
        else:
            frontal_vehicle_vm = self.frontal_vehicle
            frontal_trajectory = frontal_vehicle_vm.agent.get_local_planner().get_trajetory()
            print("vehicle id is %d,"
                  " length of self trajectory is %d,"
                  " the length of frontal is %d" % (self.vehicle.id,
                                                    len(self._local_planner.get_trajetory()),
                                                    len(frontal_trajectory)))

            ego_trajetory = deque(maxlen=30)
            ego_loc_x, ego_loc_y, ego_loc_z = self.vehicle.get_location().x, \
                                              self.vehicle.get_location().y, self.vehicle.get_location().z
            tracked_length = len(frontal_trajectory) - 1 if not self.frontal_vehicle.agent.frontal_vehicle \
                else len(frontal_trajectory)

            for i in range(tracked_length):
                delta_t = 0.1
                # print('previous x :%f, delta t: %f' % (frontal_trajectory[i][0].location.x, delta_t))
                if i == 0:
                    pos_x = (frontal_trajectory[i][0].location.x + self.behavior.inter_gap / delta_t * ego_loc_x) / \
                            (1 + self.behavior.inter_gap / delta_t)
                    pos_y = (frontal_trajectory[i][0].location.y + self.behavior.inter_gap / delta_t * ego_loc_y) / \
                            (1 + self.behavior.inter_gap / delta_t)
                else:
                    pos_x = (frontal_trajectory[i][0].location.x +
                             self.behavior.inter_gap / delta_t * ego_trajetory[i - 1][0].location.x) / \
                            (1 + self.behavior.inter_gap / delta_t)
                    pos_y = (frontal_trajectory[i][0].location.y +
                             self.behavior.inter_gap / delta_t * ego_trajetory[i - 1][0].location.y) / \
                            (1 + self.behavior.inter_gap / delta_t)

                distance = np.sqrt((pos_x - ego_loc_x) ** 2 + (pos_y - ego_loc_y) ** 2)
                velocity = distance / delta_t * 3.6
                ego_trajetory.append([carla.Transform(carla.Location(pos_x, pos_y, ego_loc_z)),
                                      frontal_trajectory[i][1],
                                      velocity,
                                      t_origin + delta_t])

                t_origin = frontal_trajectory[i][3]
                ego_loc_x = pos_x
                ego_loc_y = pos_y

            if not ego_trajetory:
                wpt = self._map.get_waypoint(self.vehicle.get_location())
                next_wpt = wpt.next(get_speed(self.vehicle, True) * 0.2)[0]
                ego_trajetory.append((next_wpt.transform,
                                      RoadOption.LANEFOLLOW,
                                      get_speed(self.vehicle),
                                      t_origin + 0.2))

            # consider all frontal vehicles TODO: repitive codes, use a wrapper function
            counter = 2
            frontal_vehicle_vm = frontal_vehicle_vm.agent.frontal_vehicle

            # while frontal_vehicle_vm:
            #     frontal_trajectory = frontal_vehicle_vm.agent.get_local_planner()._trajectory_complete_buffer.copy()
            #
            #     ego_loc_x, ego_loc_y, ego_loc_z = self.vehicle.get_location().x, \
            #                                       self.vehicle.get_location().y, self.vehicle.get_location().z
            #     for i in range(len(ego_trajetory)):
            #         delta_t = ego_trajetory[i][3]
            #         if i == 0:
            #             pos_x = (frontal_trajectory[i][0].location.x + self.behavior.inter_gap * counter /
            #                      delta_t * ego_loc_x) / \
            #                     (1 + self.behavior.inter_gap * counter / delta_t)
            #             pos_y = (frontal_trajectory[i][0].location.y + self.behavior.inter_gap * counter /
            #                      delta_t * ego_loc_y) / \
            #                     (1 + self.behavior.inter_gap * counter / delta_t)
            #         else:
            #             pos_x = (frontal_trajectory[i][0].location.x + self.behavior.inter_gap * counter /
            #                      delta_t * ego_trajetory[i - 1][0].location.x) / \
            #                     (1 + self.behavior.inter_gap * counter / delta_t)
            #             pos_y = (frontal_trajectory[i][0].location.y + self.behavior.inter_gap * counter /
            #                      delta_t * ego_trajetory[i - 1][0].location.y) / \
            #                     (1 + self.behavior.inter_gap * counter / delta_t)
            #
            #         distance = np.sqrt((pos_x - ego_loc_x) ** 2 + (pos_y - ego_loc_y) ** 2)
            #         velocity = distance / delta_t * 3.6
            #
            #         # TODO: Use weighted coefficient
            #         # print('previous x :%f' % ego_trajetory[i][0].location.x)
            #         ego_trajetory[i][0].location.x = (ego_trajetory[i][0].location.x * 8 + pos_x * 2) / 10
            #         # print('after x :%f' % ego_trajetory[i][0].location.x)
            #         ego_trajetory[i][0].location.y = (ego_trajetory[i][0].location.y * 8 + pos_y * 2) / 10
            #         ego_trajetory[i][3] = (ego_trajetory[i][3] * 8 + velocity * 2) / 10
            #
            #         # iterotar
            #         ego_loc_x = pos_x
            #         ego_loc_y = pos_y
            #
            #     break
            #     counter += 1
            #     frontal_vehicle_vm = frontal_vehicle_vm.agent.frontal_vehicle

            # if counter >= 3 :
            #     print('check here ')

            return self._local_planner.run_step(trajectory=ego_trajetory)

    def platooning_following_manager_naive(self, frontal_vehicle, distance,
                                           vehicle_loc, vehicle_target_road_option, debug=False):
        """
        Naive Car following behavior in platooning
        :param frontal_vehicle:  car to follow
        :param vehicle_target_road_option:
        :param vehicle_loc:
        :param distance: distance from vehicle
        :param debug: boolean for debugging
        :return: control: carla.VehicleControl
        """
        vehicle_speed = get_speed(frontal_vehicle)
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
        frontal_vehicle_loc = frontal_vehicle_vm.vehicle.get_location()

        # we choose next waypoint of the frontal vehicle as starting point to have smooth speed
        frontal_vehicle_waypoint = frontal_vehicle_vm.agent._map.get_waypoint(frontal_vehicle_loc)
        frontal_vehicle_next_waypoint = frontal_vehicle_waypoint.next(6)[0].transform.location

        # retrieve the platooning's destination
        _, _, platooning_manager = frontal_vehicle_vm.get_platooning_status()
        destination = platooning_manager.destination

        # regenerate route the route to make merge(lane change)
        self.set_destination(frontal_vehicle_next_waypoint, destination, clean=True)

        control = self._local_planner.run_step(target_speed=1.1 * get_speed(frontal_vehicle_vm.vehicle))

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
            self.frontal_vehicle.agent.get_local_planner().target_road_option
        frontal_vehicle_loc = frontal_vehicle.get_location()
        ego_vehicle_loc = self.vehicle.get_location()

        distance = compute_distance(ego_vehicle_loc, frontal_vehicle_loc)

        delta_v = get_speed(self.vehicle, True)
        ttc = distance / delta_v
        self.time_gap_list.append(ttc)
        self.velocity_list.append(delta_v * 3.6)
        # print("THE TTC IS %f" % ttc)

        # Distance is computed from the center of the two cars,
        # use bounding boxes to calculate the actual distance
        distance = distance - max(
            frontal_vehicle.bounding_box.extent.y, frontal_vehicle.bounding_box.extent.x) - max(
            self.vehicle.bounding_box.extent.y, self.vehicle.bounding_box.extent.x)

        # safe control for car following
        if distance <= self.behavior.braking_distance:
            print("emergency stop!")
            return self.emergency_stop()

        # control = self.platooning_following_manager_naive(frontal_vehicle, distance,
        #                                                   frontal_vehicle.get_transform(),
        #                                                   frontal_agent_target_road_option)
        control = self.platooning_folloing_manager()

        return control

    def run_step_cut_in_move2point(self, frontal_vehicle_vm, rear_vehicle_vm=None):
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

        # the vehicle needs to warm up first
        if get_speed(self.vehicle) <= self.behavior.warm_up_speed:
            print("warming up speed")
            return self.run_step(self.behavior.tailgate_speed), False

        # if the ego vehicle is still too far away
        if distance > get_speed(frontal_vehicle, True) * 1.7 and angle <= 70:
            print('trying to get the vehicle')
            rear_vehicle_vm.set_platooning_status(FSM.MAINTINING)
            return self.run_step(1.5 * get_speed(frontal_vehicle)), False

        # if the ego vehicle is too close or exceed the frontal vehicle
        if distance < get_speed(frontal_vehicle, True) * 1.0 or angle >= 70:
            print('too close, step back!')
            return self.run_step(0.95 * get_speed(frontal_vehicle)), False

        # communicate to the rear vehicle for open gap
        if not rear_vehicle_vm:
            return self.platooning_merge_management(frontal_vehicle_vm), True

        distance, angle = cal_distance_angle(rear_vehicle_vm.vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)
        # check whether the rear vehicle gives enough gap
        if distance < 1.2 * get_speed(rear_vehicle_vm.vehicle, True) or angle <= 90:
            # force the rear vehicle open gap for self
            rear_vehicle_vm.set_platooning_status(FSM.OPEN_GAP)
            return self.run_step(1.05 * get_speed(frontal_vehicle)), False

        return self.platooning_merge_management(frontal_vehicle_vm), True

    def run_step_cut_in_joining(self, frontal_vehicle_vm, rear_vehicle_vm=None):
        """
        Check if the vehicle has been joined succusfully
        :param rear_vehicle_vm:
        :param frontal_vehicle_vm:
        :return:
        """
        print("merging speed %d" % get_speed(self.vehicle))
        frontal_vehicle = frontal_vehicle_vm.vehicle
        frontal_lane = self._map.get_waypoint(frontal_vehicle.get_location()).lane_id

        ego_vehicle_loc = self.vehicle.get_location()
        ego_vehicle_lane = self._map.get_waypoint(ego_vehicle_loc).lane_id
        ego_vehicle_yaw = self.vehicle.get_transform().rotation.yaw

        distance, angle = cal_distance_angle(frontal_vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)

        if frontal_lane == ego_vehicle_lane and angle <= 5:
            print('merge finished')
            if rear_vehicle_vm:
                rear_vehicle_vm.set_platooning_status(FSM.MAINTINING)
            return self.run_step_maintaining(frontal_vehicle_vm), True

        return self._local_planner.run_step(target_speed=get_speed(frontal_vehicle)), False

    def run_step_open_gap(self):
        """
        Open gap for cut-in vehicle
        :return:
        """
        frontal_vehicle = self.frontal_vehicle.vehicle

        vehicle_speed = get_speed(frontal_vehicle)
        vehicle_loc = frontal_vehicle.get_transform()
        frontal_agent_target_road_option = \
            self.frontal_vehicle.agent.get_local_planner().target_road_option

        control = self._local_planner.run_step(target_speed=0.80 * vehicle_speed,
                                               target_waypoint=vehicle_loc,
                                               target_road_option=frontal_agent_target_road_option)

        return control
