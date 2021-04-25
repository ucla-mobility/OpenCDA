# -*- coding: utf-8 -*-

"""Platooning Manager
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

from scenerio_testing.evaluation.profile_plotting import draw_sub_plot


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

    def set_controller_longitudinal(self, max_throttle, max_brake):
        """
        Set controller statistics
        :param max_throttle:
        :param max_brake:
        :return:
        """
        for vm in self.vehicle_manager_list:
            vm.agent.set_controller_longitudinal(max_throttle, max_brake)

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

        return control_list

    def destroy(self):
        """
        Destroy vehicles and record all stats
        :return:
        """
        # for evaluation purpose
        velocity_list = []
        acceleration_list = []
        time_gap_list = []
        distance_gap_list = []

        # we don't want to skip the first 100 data points of time gap for merging vehicle
        max_len = max(len(self.vehicle_manager_list[1].agent.time_gap_list),
                      len(self.vehicle_manager_list[-1].agent.time_gap_list))

        for i in range(len(self.vehicle_manager_list)):
            self.vehicle_manager_list[i].vehicle.destroy()
            if len(self.vehicle_manager_list[i].agent.time_gap_list) > 0:
                self.vehicle_manager_list[i].cal_performance()
                start_index = 0 if len(self.vehicle_manager_list[i].agent.time_gap_list) < max_len else 100

                time_gap_list.append(self.vehicle_manager_list[i].agent.time_gap_list[start_index:-10])
                distance_gap_list.append(self.vehicle_manager_list[i].agent.distance_gap_list[start_index:-10])
            else:
                time_gap_list.append((len(self.vehicle_manager_list[i].agent.velocity_list) - 100) * [200])
                distance_gap_list.append((len(self.vehicle_manager_list[i].agent.velocity_list) - 100) * [200])

            velocity_list.append(self.vehicle_manager_list[i].agent.velocity_list[100:-10])
            acceleration_list.append(self.vehicle_manager_list[i].agent.acceleration_list[100:-10])

        draw_sub_plot(velocity_list, acceleration_list, time_gap_list, distance_gap_list)
