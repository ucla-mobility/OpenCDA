# -*- coding: utf-8 -*-

"""Platooning Manager
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import uuid
import carla

from scenerio_testing.utils.profile_plotting import draw_sub_plot


class PlatooningManager(object):
    """
    Platooning manager for vehicle managers
    """

    def __init__(self, config_yaml, world):
        """
        Construct class
        :param config_yaml:
        :param world: platooning world object
        """
        # TODO: Find a better way to give id
        self.pmid = str(uuid.uuid1())

        self.vehicle_manager_list = []
        self.maximum_capacity = config_yaml['max_capacity']

        self.destination = None
        self.center_loc = None

        # this is used to control platooning speed during joining
        self.leader_target_speed = 0
        self.origin_leader_target_speed = 0
        self.recover_speed_counter = 0

        world.update_platooning(self)

    def set_lead(self, vehicle_manager):
        """
        Set the leader of the platooning
        :param vehicle_manager:
        :return:
        """
        self.add_member(vehicle_manager, leader=True)

        # this variable is used to control leader speed
        self.origin_leader_target_speed = vehicle_manager.agent.max_speed - vehicle_manager.agent.speed_lim_dist

    def add_member(self, vehicle_manager, leader=False):
        """
        Add memeber to the current platooning
        :param leader: whether this cav is a leader
        :param vehicle_manager:
        :return:
        """
        self.vehicle_manager_list.append(vehicle_manager)
        vehicle_manager.v2x_manager.set_platoon(len(self.vehicle_manager_list)-1,
                                                platooning_object=self,
                                                platooning_id=self.pmid,
                                                leader=leader)

    def set_member(self, vehicle_manager, index, lead=False):
        """
        Set member at specific index
        :param lead:
        :param vehicle_manager:
        :param index:
        :return:
        """
        self.vehicle_manager_list.insert(index, vehicle_manager)
        vehicle_manager.v2x_manager.set_platoon(index,
                                                platooning_object=self,
                                                platooning_id=self.pmid,
                                                leader=lead)

    def cal_center_loc(self):
        """
        Calculate center location of the platoon
        :return:
        """
        v1_ego_transform = self.vehicle_manager_list[0].localizer.get_ego_pos()
        v2_ego_transform = self.vehicle_manager_list[-1].localizer.get_ego_pos()

        self.center_loc = carla.Location(x=(v1_ego_transform.location.x + v2_ego_transform.location.x)/2,
                                         y=(v1_ego_transform.location.y + v2_ego_transform.location.y)/2,
                                         z=(v1_ego_transform.location.z + v2_ego_transform.location.z)/2)

    def update_member_order(self):
        """
        Update the members' front and rear vehicle.
        This should be called whenever new member added to the platoon list
        :return:
        """
        for i, vm in enumerate(self.vehicle_manager_list):
            if i != 0:
                vm.v2x_manager.set_platoon_front(self.vehicle_manager_list[i-1])
            if i != len(self.vehicle_manager_list)-1:
                vm.v2x_manager.set_platoon_rear(self.vehicle_manager_list[i+1])

    def reset_speed(self):
        """
        After joining request accepted for certain steps, the platoon will return to the origin speed.
        :return:
        """
        if self.recover_speed_counter <= 0:
            self.leader_target_speed = self.origin_leader_target_speed
        else:
            self.recover_speed_counter -= 1

    def response_joining_request(self):
        """
        Check whether to accept joining
        TODO: Consider platooning status as well, etc. changinglane, collision status
        :return:
        """
        if len(self.vehicle_manager_list) >= self.maximum_capacity:
            return False
        else:
            # when the platoon accept a joining request,by default it will decrease the speed
            # so the merging vehicle can better catch up with
            self.leader_target_speed = self.origin_leader_target_speed - 5
            self.recover_speed_counter = 200
            return True

    def set_destination(self, destination):
        """
        Set desination of the vehicle managers in the platoon.
        TODO: Currently we assume all vehicles in a platoon will have the same destination
        :return:
        """
        self.destination = destination
        for i in range(len(self.vehicle_manager_list)):
            self.vehicle_manager_list[i].set_destination(
                self.vehicle_manager_list[i].vehicle.get_location(), destination, clean=True)

    def update_information(self, world):
        """
        Update world information for every member in the list
        :param world:
        :return:
        """
        self.reset_speed()
        for i in range(len(self.vehicle_manager_list)):
            self.vehicle_manager_list[i].update_info(world)
        # update the center location of the platoon
        self.cal_center_loc()

    def run_step(self):
        """
        Run a step for each vehicles
        :return:
        """
        control_list = []
        for i in range(len(self.vehicle_manager_list)):
            control = self.vehicle_manager_list[i].run_step(self.leader_target_speed)
            control_list.append(control)

        for (i, control) in enumerate(control_list):
            self.vehicle_manager_list[i].vehicle.apply_control(control)

        return control_list

    def destroy(self):
        """
        Destroy vehicles
        :return:
        """
        for i in range(len(self.vehicle_manager_list)):
            self.vehicle_manager_list[i].vehicle.destroy()
