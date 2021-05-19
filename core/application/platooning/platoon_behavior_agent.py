# -*- coding: utf-8 -*-

"""Behavior manager for platooning specifically
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from collections import deque

import carla
import numpy as np

from core.plan.behavior_agent import BehaviorAgent, RoadOption
from core.common.misc import compute_distance, get_speed, cal_distance_angle
from core.application.platooning.fsm import FSM


class PlatooningBehaviorAgent(BehaviorAgent):
    """
    The behavior agent for platooning
    """

    def __init__(self, vehicle, vehicle_manager, v2x_manager, behavior_yaml, platoon_yaml, platoon_world):
        """
        Construct class
        :param vehicle: carla actor todo:remove this later
        :param vehicle_manager: vehicle manager of this agent. todo: try to avoid such circle reference in future
        :param v2x_manager: communication manager
        :param behavior_yaml: configure yaml file for normal behavior agent
        :param platoon_yaml:  configure yaml file for platoon behavior agent
        :param platoon_world: platoon world object that contains all existing cavs
        :return
        """

        super(PlatooningBehaviorAgent, self).__init__(vehicle, behavior_yaml)

        self.vehicle_manager = vehicle_manager
        # communication manager
        self.v2x_manager = v2x_manager
        self.platoon_world = platoon_world

        # used for gap keeping
        self.inter_gap = platoon_yaml['inter_gap']
        # used when open a gap
        self.open_gap = platoon_yaml['open_gap']
        # this is used to control gap opening during cooperative joining
        self.current_gap = self.inter_gap

        # used for merging vehicle
        self.destination_changed = False

        # merging vehicle needs to reach this speed before cooperative merge
        self.warm_up_speed = platoon_yaml['warm_up_speed']

        # used to calculate performance
        self.time_gap_list = []
        self.distance_gap_list = []
        self.velocity_list = []
        self.acceleration_list = []

    def run_step(self, target_speed=None, collision_detector_enabled=True):
        """
        Run a single step for naviation
        :param target_speed:
        :param collision_detector_enabled:
        :return:
        """
        status = self.v2x_manager.get_platoon_status()
        # case1: the vehicle is not cda enabled
        if status == FSM.DISABLE:
            return super().run_step(target_speed, collision_detector_enabled)

        # case2: single vehicle keep searching platoon to join
        if status == FSM.SEARCHING:
            find_platoon, min_index = self.v2x_manager.match_platoon(self.platoon_world)

            # no platoon found, stay in searching status
            if not find_platoon:
                return super().run_step(target_speed, collision_detector_enabled)

            # platoon found and agreement achieved
            front_vehicle, rear_vehicle = self.v2x_manager.get_platoon_front_rear()
            # if no front vehicle, meaning it will be a frontal joining
            if not front_vehicle and rear_vehicle:
                print('merging vehicle chooses frontal joining')
                self.v2x_manager.set_platoon_status(FSM.FRONT_JOINING)
            # if front vehicle and rear vehicle both exist
            if front_vehicle and rear_vehicle:
                print("merging vehicle chooses cut-in joining")
                self.v2x_manager.set_platoon_status(FSM.MOVE_TO_POINT)
            # if only front vehicle exits
            if front_vehicle and not rear_vehicle:
                print("merging vehicle chooses back joining")
                self.v2x_manager.set_platoon_status(FSM.BACK_JOINING)

            return super().run_step(target_speed, collision_detector_enabled)

        # case3.1: the merging vehicle chooses cut-in joining and is moving to the meeting point
        if status == FSM.MOVE_TO_POINT:
            target_speed, target_waypoint, new_status = self.run_step_cut_in_move2point()
            self.v2x_manager.set_platoon_status(new_status)
            return target_speed, target_waypoint
        # case3.2: the merging vehicle chooses cut-in joining and is ready for merging
        if status == FSM.JOINING:
            target_speed, target_waypoint, new_status = self.run_step_cut_in_joining()
            # if joining is finished
            if new_status == FSM.JOINING_FINISHED:
                self.joining_finish_manager()
            return target_speed, target_waypoint

        # case 4: the merging vehicle selects back joining
        if status == FSM.BACK_JOINING:
            target_speed, target_waypoint, new_status = self.run_step_back_joining()
            # if joining is finshed
            if new_status == FSM.JOINING_FINISHED:
                self.joining_finish_manager()
            return target_speed, target_waypoint

        # case 5: the merging vehicle selects frontal joining
        if status == FSM.FRONT_JOINING:
            target_speed, target_waypoint, new_status = self.run_step_front_joining()
            self.v2x_manager.set_platoon_status(new_status)

            # if joining abandoned
            if new_status == FSM.ABONDON:
                self.v2x_manager.set_platoon_status(FSM.SEARCHING)
                _, rear_vehicle_manager = self.v2x_manager.get_platoon_front_rear()
                self.v2x_manager.add_platoon_blacklist(rear_vehicle_manager.v2x_manager.get_platoon_manager()[0].pmid)
            if new_status == FSM.JOINING_FINISHED:
                self.joining_finish_manager('rear')

            return target_speed, target_waypoint

        # case 6: leading vehicle behavior
        if status == FSM.LEADING_MODE:
            return super().run_step(target_speed, collision_detector_enabled)

        # case7: maintaining status
        if status == FSM.MAINTINING:
            return self.run_step_maintaining()

        # case8: Open Gap status
        if status == FSM.OPEN_GAP:
            return self.run_step_open_gap()

    def joining_finish_manager(self, insert_vehicle='front'):
        """
        Called when a joining is finish to update the platoon manager list.
        :param insert_vehicle: indicate use the front or rear vehicle index to update the platoon manager list.
        :return:
        """
        frontal_vehicle_manager, rear_vehicle_manager = self.v2x_manager.get_platoon_front_rear()
        if insert_vehicle == 'front':
            platoon_manger, index = frontal_vehicle_manager.v2x_manager.get_platoon_manager()
            platoon_manger.set_member(self.vehicle_manager, index + 1)
        else:
            platoon_manger, index = rear_vehicle_manager.v2x_manager.get_platoon_manager()
            platoon_manger.set_member(self.vehicle_manager, index, lead=True)

        platoon_manger.update_member_order()
        self._local_planner.debug_trajectory = False

    def calculate_gap(self, distance):
        """
        Calculate the current vehicle and frontal vehicle's time/distance gap
        :param distance:  distance between the ego vehicle and frontal vehicle
        :return:
        """
        # we need to count the vehicle length in to calculate the gap
        boundingbox = self._vehicle.bounding_box
        veh_length = 2 * abs(boundingbox.location.y - boundingbox.extent.y)

        delta_v = self._ego_speed / 3.6
        time_gap = distance / delta_v
        self.time_gap_list.append(time_gap)
        self.distance_gap_list.append(distance - veh_length)

    def platooning_following_manager(self, inter_gap):
        """
        Car following behavior in platooning with gap regulation
        :param inter_gap: the gap designed for platooning
        :return:
        """

        frontal_vehicle_manager, _ = self.v2x_manager.get_platoon_front_rear()
        frontal_front_vehicle_manger, _ = frontal_vehicle_manager.v2x_manager.get_platoon_front_rear()

        # must match leading vehicle's trajectory unit time
        t_origin = 0

        # check whether the platooning is in car following mode
        if frontal_vehicle_manager.agent.car_following_flag:
            self.car_following_flag = True

        if len(self._local_planner.get_trajetory()) > 7:
            return self._local_planner.run_step([], [], [], following=True)
        else:
            frontal_trajectory = frontal_vehicle_manager.agent.get_local_planner().get_trajetory()

            ego_trajetory = deque(maxlen=30)
            ego_loc_x, ego_loc_y, ego_loc_z = self._ego_pos.location.x, self._ego_pos.location.y, \
                                              self._ego_pos.location.z

            tracked_length = len(frontal_trajectory) - 1 if not frontal_front_vehicle_manger \
                else len(frontal_trajectory)
            # todo: current not working well on curve
            for i in range(tracked_length):
                delta_t = 0.25  # todo: this is harded coded
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
                                   self._map.get_waypoint(self._ego_pos.location).transform.location.z)),
                    frontal_trajectory[i][1],
                    velocity,
                    t_origin + delta_t])

                t_origin = frontal_trajectory[i][3]
                ego_loc_x = pos_x
                ego_loc_y = pos_y

            if not ego_trajetory:
                wpt = self._map.get_waypoint(self._ego_pos.location)
                next_wpt = wpt.next(max(2, self._ego_speed / 3.6 * 1))[0]
                ego_trajetory.append((next_wpt.transform,
                                      RoadOption.LANEFOLLOW,
                                      self._ego_speed,
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
        platooning_manager, _ = frontal_vehicle_vm.v2x_manager.get_platoon_manager()
        destination = platooning_manager.destination

        # regenerate route the route to make merge(lane change)
        self.set_destination(frontal_vehicle_next_waypoint, destination, clean=True)

        target_speed, target_waypoint = super().run_step(target_speed=1.5 * get_speed(frontal_vehicle_vm.vehicle),
                                                         collision_detector_enabled=False)

        return target_speed, target_waypoint

    def run_step_maintaining(self):
        """
        Behavior planning for speed maintaining
        :return:
        """
        frontal_vehicle_manager, _ = self.v2x_manager.get_platoon_front_rear()
        self.current_gap = self.inter_gap

        frontal_vehicle = frontal_vehicle_manager.vehicle
        frontal_vehicle_loc = frontal_vehicle.get_location()
        ego_vehicle_loc = self._ego_pos.location

        # headway distance
        distance = compute_distance(ego_vehicle_loc, frontal_vehicle_loc)
        self.calculate_gap(distance)

        # Distance is computed from the center of the two cars,
        # use bounding boxes to calculate the actual distance
        distance = distance - max(
            frontal_vehicle.bounding_box.extent.y, frontal_vehicle.bounding_box.extent.x) - max(
            self.vehicle.bounding_box.extent.y, self.vehicle.bounding_box.extent.x)

        # safe control for car following todo: make the coefficient controllable
        if distance <= self._ego_speed / 3.6 * 0.01:
            print("emergency stop!")
            return 0, None

        target_speed, target_waypoint = self.platooning_following_manager(self.inter_gap)

        return target_speed, target_waypoint

    def run_step_cut_in_move2point(self):
        """
        TODO:  More than 2 lanes not working
        The vehicle is trying to get to the move in point
        :return: target_speed, target_waypoint, next FSM state
        """

        frontal_vehicle_manager, rear_vehicle_vm = self.v2x_manager.get_platoon_front_rear()
        frontal_vehicle = frontal_vehicle_manager.vehicle

        # todo: use localization module later
        ego_vehicle_loc =  self._ego_pos.location
        ego_vehicle_yaw =  self._ego_pos.rotation.yaw

        distance, angle = cal_distance_angle(frontal_vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)

        # calculate the time gap with the frontal vehicle
        self.calculate_gap(distance)

        # if there is a obstacle blocking ahead, we just change to back joining mode todo: lane change not considered
        if self.hazard_flag:
            if rear_vehicle_vm:
                rear_vehicle_vm.v2x_manager.set_platoon_status(FSM.MAINTINING)
            # retrieve the last member in the platoon
            platoon_manager, _ = frontal_vehicle_manager.v2x_manager.get_platoon_manager()
            last_member = platoon_manager.vehicle_manager_list[-1]

            # set the last member as the frontal vehicle
            self.v2x_manager.set_platoon_front(last_member)
            self.v2x_manager.set_platoon_rear(None)

            # slow down to join back
            return (*super().run_step(self.max_speed / 2), FSM.BACK_JOINING)

        # the vehicle needs to warm up first. But if the platooning is in car following state, then we should ignore
        if self._ego_speed <= self.warm_up_speed and not frontal_vehicle_manager.agent.car_following_flag:
            print("warming up speed")
            return (*super().run_step(self.tailgate_speed), FSM.MOVE_TO_POINT)

        # if the ego vehicle is still too far away from the front vehicle
        if distance > self._ego_speed / 3.6 * (self.inter_gap + 0.5) and angle <= 80:
            print('trying to get the vehicle')
            return (*super().run_step(2.0 * get_speed(frontal_vehicle)), FSM.MOVE_TO_POINT)

        # if the ego vehicle is too close or exceed the frontal vehicle
        if distance < self._ego_speed / 3.6 * self.inter_gap / 1.5 or angle >= 80:
            print('too close, step back!')
            return (*super().run_step(0.9 * get_speed(frontal_vehicle)), FSM.MOVE_TO_POINT)

        # communicate to the rear vehicle for open gap if rear vehicle exists
        if not rear_vehicle_vm:
            return (*self.platooning_merge_management(frontal_vehicle_manager), FSM.JOINING)

        distance, angle = cal_distance_angle(rear_vehicle_vm.vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)

        # check whether the rear vehicle gives enough gap
        if distance < 1.0 * self.inter_gap / 2.0 * self._ego_speed / 3.6 \
                or angle <= 100 or rear_vehicle_vm.agent.current_gap < self.open_gap:
            # force the rear vehicle open gap for self
            print("too close to rear vehicle!")
            rear_vehicle_vm.v2x_manager.set_platoon_status(FSM.OPEN_GAP)
            return (*super().run_step(1.5 * get_speed(frontal_vehicle)), FSM.MOVE_TO_POINT)

        return (*self.platooning_merge_management(frontal_vehicle_manager), FSM.JOINING)

    def run_step_cut_in_joining(self):
        """
        Check if the vehicle has been joined successfully TODO: Return status instead of True of False
        :return:
        """
        print("merging speed %d" % self._ego_speed)

        frontal_vehicle_manager, rear_vehicle_vm = self.v2x_manager.get_platoon_front_rear()

        frontal_vehicle = frontal_vehicle_manager.vehicle
        frontal_lane = self._map.get_waypoint(frontal_vehicle.get_location()).lane_id

        ego_vehicle_loc = self._ego_pos.location
        ego_vehicle_lane = self._map.get_waypoint(ego_vehicle_loc).lane_id
        ego_vehicle_yaw = self._ego_pos.rotation.yaw

        distance, angle = cal_distance_angle(frontal_vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)
        # calculate the time gap with the frontal vehicle
        self.calculate_gap(distance)

        if frontal_lane == ego_vehicle_lane and angle <= 5:
            print('merge finished')
            if rear_vehicle_vm:
                rear_vehicle_vm.v2x_manager.set_platoon_status(FSM.MAINTINING)
            return (*self.run_step_maintaining(), FSM.JOINING_FINISHED)

        return (*super().run_step(target_speed=get_speed(frontal_vehicle),
                                  collision_detector_enabled=False), FSM.JOINING)

    def run_step_open_gap(self):
        """
        Open gap for cut-in vehicle
        :return:
        """
        frontal_vehicle_manager, rear_vehicle_manager = self.v2x_manager.get_platoon_front_rear()

        # calculate the time gap under this state
        ego_vehicle_loc = self._ego_pos.location
        ego_vehicle_yaw = self._ego_pos.rotation.yaw

        distance, _ = cal_distance_angle(frontal_vehicle_manager.localizer.get_ego_pos().location,
                                         ego_vehicle_loc, ego_vehicle_yaw)
        self.calculate_gap(distance)

        # gradually open the gap TODO: Make this dynamic to map a linear relationship with speed
        if self.current_gap < self.open_gap:
            self.current_gap += 0.01
        print('cuurent gap is %f' % self.current_gap)
        target_speed, target_loc = self.platooning_following_manager(self.current_gap)

        return target_speed, target_loc

    def run_step_back_joining(self):
        """
        Back-joining Algorithm
        :return: control command and whether back joining finished
        """
        frontal_vehicle_manager, _ = self.v2x_manager.get_platoon_front_rear()
        # reset lane change flag every step
        self.lane_change_allowed = True

        # get necessary information of the ego vehicle and target vehicle in the platooning
        frontal_vehicle = frontal_vehicle_manager.vehicle
        frontal_lane = self._map.get_waypoint(frontal_vehicle.get_location()).lane_id

        # retrieve the platooning's destination
        platooning_manager, _ = frontal_vehicle_manager.v2x_manager.get_platoon_manager()
        frontal_destination = platooning_manager.destination

        # retrieve ego vehicle info todo: remove this later
        ego_vehicle_loc = self._ego_pos.location
        ego_wpt = self._map.get_waypoint(ego_vehicle_loc)
        ego_vehicle_lane = ego_wpt.lane_id
        ego_vehicle_yaw = self._ego_pos.rotation.yaw

        distance, angle = cal_distance_angle(frontal_vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)

        # calculate the time gap with the frontal vehicle
        self.calculate_gap(distance)

        # 0. make sure the vehicle is behind the ego vehicle
        if angle >= 60 or distance < self._ego_speed / 3.6 * 0.5:
            self.overtake_allowed = False
            print("angle is too large, wait")
            self.lane_change_allowed = False
            return (*super().run_step(get_speed(frontal_vehicle) * 0.95), FSM.BACK_JOINING)

        else:
            self.overtake_allowed = True

        # 1. make sure the speed is warmed up first. Also we don't want to reset destination during lane change
        if self._ego_speed < self.warm_up_speed or self.get_local_planner().lane_change:
            print('warm up speed')
            return (*super().run_step(self.tailgate_speed), FSM.BACK_JOINING)

        if not self.destination_changed:
            print('destination reset!!!!')
            self.destination_changed = True
            self.set_destination(ego_wpt.next(4.5)[0].transform.location, frontal_destination)

        # 2. check if there is any other vehicle blocking between ego and platooning
        vehicle_list = self._world.get_actors().filter("*vehicle*")

        def dist(v):
            return v.get_location().distance(ego_vehicle_loc)

        # only consider vehicles in 45 meters, not in the platooning as the candidate of collision
        # todo: this is ugly
        vehicle_list = [v for v in vehicle_list if dist(v) < 60 and
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
                and distance < 1.0 * self._ego_speed / 3.6 :
            print('joining finished !')
            return (*self.run_step_maintaining(), FSM.JOINING_FINISHED)

        # 4. If vehicle is not blocked, make ego back to the frontal vehicle's lane
        if not vehicle_blocking_status:
            print('no vehicle is blocking!!!')
            if frontal_lane != ego_vehicle_lane:
                left_wpt = ego_wpt.next(max(1.2 * self._ego_speed / 3.6 , 5))[0].get_left_lane()
                right_wpt = ego_wpt.next(max(1.2 * self._ego_speed / 3.6 , 5))[0].get_right_lane()

                if not left_wpt and not right_wpt:
                    pass
                # if no right lane
                elif not right_wpt:
                    print('take left lane')
                    self.set_destination(left_wpt.transform.location, frontal_destination, clean=True)
                # if no left lane available
                elif not left_wpt:
                    print('take right lane')
                    self.set_destination(right_wpt.transform.location, frontal_destination, clean=True)
                # check which lane is closer to the platooning
                elif abs(left_wpt.lane_id - frontal_lane) < abs(right_wpt.lane_id - frontal_lane):
                    print('take left lane')
                    self.set_destination(left_wpt.transform.location, frontal_destination, clean=True)
                else:
                    print('take right lane')
                    self.set_destination(right_wpt.transform.location, frontal_destination, clean=True)

        return (*super().run_step(self.tailgate_speed), FSM.BACK_JOINING)

    def run_step_front_joining(self):
        """
        Front-joining algorithm
        :return:
        """
        _, rear_vehicle_manager = self.v2x_manager.get_platoon_front_rear()
        # get necessary information of the ego vehicle and target vehicle in the platooning
        rear_vehicle = rear_vehicle_manager.vehicle
        rear_lane = self._map.get_waypoint(rear_vehicle.get_location()).lane_id

        # retrieve the platooning's destination
        platooning_manager, _ = rear_vehicle_manager.v2x_manager.get_platoon_manager()
        rear_destination = platooning_manager.destination

        # retrieve ego vehicle info
        ego_vehicle_loc = self._ego_pos.location
        ego_wpt = self._map.get_waypoint(ego_vehicle_loc)
        ego_vehicle_lane = ego_wpt.lane_id
        ego_vehicle_yaw = self._ego_pos.rotation.yaw

        distance, angle = cal_distance_angle(rear_vehicle.get_location(),
                                             ego_vehicle_loc, ego_vehicle_yaw)

        # if there is a vehicle blocking between, then abandon this joining
        vehicle_list = self._world.get_actors().filter("*vehicle*")

        def dist(v):
            return v.get_location().distance(ego_vehicle_loc)

        # only consider vehicles in 60 meters, not in the platooning as the candidate of collision
        vehicle_list = [v for v in vehicle_list if dist(v) < 60 and
                        v.id != self.vehicle.id and
                        v.id not in self._platooning_world.vehicle_id_set]

        vehicle_blocking_status = False
        for vehicle in vehicle_list:
            vehicle_blocking_status = vehicle_blocking_status or self._collision_check.is_in_range(self.vehicle,
                                                                                                   rear_vehicle,
                                                                                                   vehicle,
                                                                                                   self._map)
        # if vehicle blocking between ego and platooning, then abandon this joining
        if vehicle_blocking_status:
            print('abandon front joining')
            return (*super().run_step(self.max_speed - self.speed_lim_dist), FSM.ABONDON)

        # if the ego vehilce is already behind the platooning
        if angle <= 90:
            print('transition to other joining')
            return (*super().run_step(self.tailgate_speed), FSM.SEARCHING)

        # if vehicle is already in the same lane with the platooning
        if ego_vehicle_lane == rear_lane:
            # after in the same lane, no overtake is allowed anymore
            self.overtake_allowed = False

            if not self.destination_changed:
                self.set_destination(ego_wpt.next(4.5)[0].transform.location, rear_destination)
            if distance < self._ego_speed / 3.6 * self.inter_gap * 1.5:
                print('joining finished')
                return (*super().run_step(self.max_speed - self.speed_lim_dist), FSM.JOINING_FINISHED)
            else:
                return (*super().run_step(get_speed(rear_vehicle) * 0.95), FSM.FRONT_JOINING)

        # if the ego is too close to the platooning or speed is too slow
        if distance < self._ego_speed / 3.6 * self.inter_gap \
                or self._ego_speed < self.warm_up_speed \
                or angle <= 90 or get_speed(rear_vehicle) > self._ego_speed \
                or self.get_local_planner().lane_change:
            print('need to speed up before change lane')
            return (*super().run_step(self.tailgate_speed), FSM.FRONT_JOINING)

        # set destination same as platooning
        if not self.destination_changed:
            print('destination reset!!!!')
            self.destination_changed = True
            self.set_destination(ego_wpt.next(4.5)[0].transform.location, rear_destination)

        # check which lane is closer to operate lane change
        left_wpt = ego_wpt.next(max(1.2 * self._ego_speed / 3.6, 5))[0].get_left_lane()
        right_wpt = ego_wpt.next(max(1.2 * self._ego_speed / 3.6, 5))[0].get_right_lane()
        # check which lane is closer to the platooning
        if not left_wpt and not right_wpt:
            pass
        # check which lane is closer to the platooning
        elif not right_wpt or abs(left_wpt.lane_id - rear_lane) < abs(right_wpt.lane_id - rear_lane):
            print('take left lane')
            self.set_destination(left_wpt.transform.location, rear_destination, clean=True)
        else:
            print('take right lane')
            self.set_destination(right_wpt.transform.location, rear_destination, clean=True)

        return (*super().run_step(self.tailgate_speed), FSM.FRONT_JOINING)
