# -*- coding: utf-8 -*-

"""Platooning Manager
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import numpy as np

from core.vehicle.vehicle_manager import VehicleManager
from pviz.profile_plotting import draw_velocity_profile, draw_intergap_profile


class PlatooningManager(object):
    """
    Platooning manager for vehicle managers
    """

    def __init__(self, world, maximum_capcity=10, pmid=1):
        """
        Construct class
        :param world: platooning world object
        :param maximum_capcity:
        """
        # TODO: Find a better way to give id
        self.pmid = pmid

        # TODO: Use a more stable data structure
        self.vehicle_manager_list = []
        self.maximum_capacity = maximum_capcity

        self.leader_uuid = None
        self.agent = None
        self.destination = None

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
        self.leader_uuid = vehicle_manager.vid
        self.vehicle_manager_list.append(vehicle_manager)

        leader_behavior = vehicle_manager.agent.behavior
        self.origin_leader_target_speed = leader_behavior.max_speed - leader_behavior.speed_lim_dist

        vehicle_manager.set_platooning(self, self.pmid, 0, True)

    def add_member(self, vehicle_manager):
        """
        Add memeber to the current platooning
        TODO: This very naive and temp solution. We need strictly follow FSM to add member into a platooning
        :param vehicle_manager:
        :return:
        """
        self.vehicle_manager_list.append(vehicle_manager)
        vehicle_manager.set_platooning(self, self.pmid, len(self.vehicle_manager_list)-1, False)

    def set_member(self, vehicle_manager, index, lead=False):
        """
        Set member at specific index
        :param lead:
        :param vehicle_manager:
        :param index:
        :return:
        """
        self.vehicle_manager_list.insert(index, vehicle_manager)
        vehicle_manager.set_platooning(self, self.pmid, index, lead)

    def reset_speed(self):
        """
        Reset the leader speed to old state
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
            # TODO: USE GFS Model to do this
            self.leader_target_speed = self.origin_leader_target_speed - 10
            self.recover_speed_counter = 200
            return True

    def set_destination(self, destination):
        """
        Set desination of the vehicle managers
        TODO: Right now all the vehicles have the same destination, change it later
        :return:
        """
        self.destination = destination
        for i in range(len(self.vehicle_manager_list)):
            self.vehicle_manager_list[i].agent.set_destination(
                self.vehicle_manager_list[i].vehicle.get_location(), destination, clean=True)

    def update_information(self, world):
        """
        Update world information for every member in the list
        :param world:
        :return:
        """
        self.reset_speed()
        for i in range(len(self.vehicle_manager_list)):
            if i == 0:
                self.vehicle_manager_list[i].update_info(world)
            else:
                self.vehicle_manager_list[i].update_info(world,
                                                         self.vehicle_manager_list[i - 1])

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

    def destroy(self):
        """
        TODO: Only destroy vehicles for now
        :return:
        """
        # for evaluation purpose
        velocity_list = []
        gap_list = []
        for i in range(len(self.vehicle_manager_list)):
            self.vehicle_manager_list[i].vehicle.destroy()
            if i > 0:
                self.vehicle_manager_list[i].cal_performance()
                gap_list.append(self.vehicle_manager_list[i].agent.time_gap_list[100:-10])
            velocity_list.append(self.vehicle_manager_list[i].agent.velocity_list)

        draw_velocity_profile(velocity_list, np.arange(0, len(velocity_list)))
        draw_intergap_profile(gap_list, np.arange(0, len(gap_list)))
