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

    def __init__(self, vehicle, ignore_traffic_light=True, behavior='normal', overtake_allowed=False,
                 sampling_resolution=4.5, buffer_size=5, dynamic_pid=False,
                 update_freq=15, debug_trajectory=True, debug=True):
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

        super(PlatooningBehaviorAgent, self).__init__(vehicle, ignore_traffic_light, behavior, overtake_allowed,
                                                      sampling_resolution, buffer_size, dynamic_pid,
                                                      debug_trajectory, debug, update_freq)
        # used for control open gap gradually
        self.current_gap = self.behavior.inter_gap

        # used to see the average time gap between ego and frontal vehicle
        self.time_gap_list = []
        self.velocity_list = []

        self.destination_changed = False

    def platooning_following_manager(self, inter_gap):
        """
        Car following behavior in platooning with gap regulation
        :param inter_gap: the gap designed for platooning
        :return:
        """
        # must match leading vehicle's trajectory unit time
        t_origin = 0
        if len(self._local_planner.get_trajetory()) > 7:
            return self._local_planner.run_step([], [], [], following=True)
        else:
            frontal_vehicle_vm = self.frontal_vehicle
            frontal_trajectory = frontal_vehicle_vm.agent.get_local_planner().get_trajetory()
            # print("vehicle id is %d,"
            #       " length of self trajectory is %d,"
            #       " the length of frontal is %d" % (self.vehicle.id,
            #                                         len(self._local_planner.get_trajetory()),
            #                                         len(frontal_trajectory)))

            ego_trajetory = deque(maxlen=30)
            ego_loc_x, ego_loc_y, ego_loc_z = self.vehicle.get_location().x, \
                                              self.vehicle.get_location().y, self.vehicle.get_location().z
            tracked_length = len(frontal_trajectory) - 1 if not self.frontal_vehicle.agent.frontal_vehicle \
                else len(frontal_trajectory)

            for i in range(tracked_length):
                delta_t = 0.3
                # print('previous x :%f, delta t: %f' % (frontal_trajectory[i][0].location.x, delta_t))
                if i == 0:
                    pos_x = (frontal_trajectory[i][0].location.x + inter_gap / delta_t * ego_loc_x) / \
                            (1 + inter_gap / delta_t)
                    pos_y = (frontal_trajectory[i][0].location.y + inter_gap / delta_t * ego_loc_y) / \
                            (1 + inter_gap / delta_t)
                else:
                    pos_x = (frontal_trajectory[i][0].location.x +
                             inter_gap / delta_t * ego_trajetory[i - 1][0].location.x) / \
                            (1 + inter_gap / delta_t)
                    pos_y = (frontal_trajectory[i][0].location.y +
                             inter_gap / delta_t * ego_trajetory[i - 1][0].location.y) / \
                            (1 + inter_gap / delta_t)

                distance = np.sqrt((pos_x - ego_loc_x) ** 2 + (pos_y - ego_loc_y) ** 2)
                velocity = distance / delta_t * 3.6
                ego_trajetory.append([carla.Transform(
                    carla.Location(pos_x, pos_y,
                                   self._map.get_waypoint(self.vehicle.get_location()).transform.location.z)),
                    frontal_trajectory[i][1],
                    velocity,
                    t_origin + delta_t])

                t_origin = frontal_trajectory[i][3]
                ego_loc_x = pos_x
                ego_loc_y = pos_y

            if not ego_trajetory:
                wpt = self._map.get_waypoint(self.vehicle.get_location())
                next_wpt = wpt.next(max(2, get_speed(self.vehicle, True) * 1))[0]
                ego_trajetory.append((next_wpt.transform,
                                      RoadOption.LANEFOLLOW,
                                      get_speed(self.vehicle),
                                      t_origin + 0.2))

            return self._local_planner.run_step([], [], [], trajectory=ego_trajetory)

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
        frontal_vehicle_next_waypoint = frontal_vehicle_waypoint.next(
            get_speed(frontal_vehicle_vm.vehicle, True))[0].transform.location

        # retrieve the platooning's destination
        _, _, platooning_manager = frontal_vehicle_vm.get_platooning_status()
        destination = platooning_manager.destination

        # regenerate route the route to make merge(lane change)
        self.set_destination(frontal_vehicle_next_waypoint, destination, clean=True)

        control = self.run_step(target_speed=1.5 * get_speed(frontal_vehicle_vm.vehicle))

        return control

    def run_step_maintaining(self, frontal_vehicle=None):
        """
        Behavior planning for speed maintaining
        :return:
        """
        self.current_gap = self.behavior.inter_gap

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

        # Distance is computed from the center of the two cars,
        # use bounding boxes to calculate the actual distance
        distance = distance - max(
            frontal_vehicle.bounding_box.extent.y, frontal_vehicle.bounding_box.extent.x) - max(
            self.vehicle.bounding_box.extent.y, self.vehicle.bounding_box.extent.x)

        # safe control for car following
        if distance <= self.behavior.braking_distance:
            print("emergency stop!")
            return self.emergency_stop()

        control = self.platooning_following_manager(self.behavior.inter_gap)

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
        if distance > get_speed(self.vehicle, True) * (self.behavior.inter_gap + 0.5) and angle <= 70:
            print('trying to get the vehicle')
            return self.run_step(2.0 * get_speed(frontal_vehicle)), False

        # if the ego vehicle is too close or exceed the frontal vehicle
        if distance < get_speed(self.vehicle, True) * self.behavior.inter_gap or angle >= 80:
            print('too close, step back!')
            return self.run_step(0.95 * get_speed(frontal_vehicle)), False

        # communicate to the rear vehicle for open gap
        if not rear_vehicle_vm:
            return self.platooning_merge_management(frontal_vehicle_vm), True

        distance, angle = cal_distance_angle(rear_vehicle_vm.vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)
        # check whether the rear vehicle gives enough gap
        if distance < 1.0 * self.behavior.inter_gap * get_speed(rear_vehicle_vm.vehicle, True) \
                or angle <= 100 or rear_vehicle_vm.agent.current_gap < self.behavior.open_gap:
            # force the rear vehicle open gap for self
            print("too close to rear vehicle!")
            rear_vehicle_vm.set_platooning_status(FSM.OPEN_GAP)
            return self.run_step(1.5 * get_speed(frontal_vehicle)), False

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

        return self.run_step(target_speed=get_speed(frontal_vehicle)), False

    def run_step_open_gap(self):
        """
        Open gap for cut-in vehicle
        :return:
        """
        # gradually open the gap
        if self.current_gap < self.behavior.open_gap:
            self.current_gap += 0.005
        print('cuurent gap is %f' % self.current_gap)
        control = self.platooning_following_manager(self.current_gap)

        return control

    def run_step_back_joining(self, frontal_vehicle_vm):
        """
        Back-joining Algorithm
        :param frontal_vehicle_vm: the vehicle that ego is trying to catch up
        :return: control command and whether back joining finished
        """
        # 1. make sure the speed is warmed up first
        if get_speed(self.vehicle) < self.behavior.warm_up_speed:
            print('warm up speed')
            return self.run_step(self.behavior.tailgate_speed), False

        # get necessary information of the ego vehicle and target vehicle in the platooning
        frontal_vehicle = frontal_vehicle_vm.vehicle
        frontal_lane = self._map.get_waypoint(frontal_vehicle.get_location()).lane_id
        # retrieve the platooning's destination
        _, _, platooning_manager = frontal_vehicle_vm.get_platooning_status()
        frontal_destination = platooning_manager.destination

        ego_vehicle_loc = self.vehicle.get_location()
        ego_wpt = self._map.get_waypoint(ego_vehicle_loc)
        ego_vehicle_lane = ego_wpt.lane_id
        ego_vehicle_yaw = self.vehicle.get_transform().rotation.yaw

        if not self.destination_changed:
            self.destination_changed = True
            self.set_destination(ego_wpt.next(4.5)[0].transform.location, frontal_destination)

        distance, angle = cal_distance_angle(frontal_vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)

        # 2. check if there is any other vehicle blocking between ego and platooning
        vehicle_list = self._world.get_actors().filter("*vehicle*")

        def dist(v):
            return v.get_location().distance(ego_vehicle_loc)

        # only consider vehicles in 45 meters, not in the platooning as the candidate of collision
        vehicle_list = [v for v in vehicle_list if dist(v) < 45 and
                        v.id != self.vehicle.id and
                        v.id not in self._platooning_world.vehicle_id_set]

        vehicle_blocking_status = False
        for vehicle in vehicle_list:
            vehicle_blocking_status = vehicle_blocking_status or self._collision_check.is_in_range(self.vehicle,
                                                                                                   frontal_vehicle,
                                                                                                   vehicle,
                                                                                                   self._map)

        # 3. if no other vehicle is blocking, the ego vehicle is in the same lane with the platooning
        # and it is close enough, then we regard the back joining finished
        if frontal_lane == ego_vehicle_lane \
                and not vehicle_blocking_status \
                and distance < 1.5 * get_speed(self.vehicle, True):
            print('joining finished !')
            return self.run_step_maintaining(frontal_vehicle_vm), True

        # 4. If vehicle is not blocked, make ego back to the frontal vehicle's lane
        if not vehicle_blocking_status:
            if frontal_lane != ego_vehicle_lane:
                left_wpt = ego_wpt.next(max(get_speed(self.vehicle, True), 5))[0].get_left_lane()
                right_wpt = ego_wpt.next(max(get_speed(self.vehicle, True), 5))[0].get_right_lane()
                # check which lane is closer to the platooning
                if abs(left_wpt.lane_id - frontal_lane) < abs(right_wpt.lane_id - frontal_lane):
                    self.set_destination(left_wpt.transform.location, frontal_destination, clean=True)
                else:
                    self.set_destination(right_wpt.transform.location, frontal_destination, clean=True)

        return self.run_step(self.behavior.tailgate_speed), False
