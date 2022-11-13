# -*- coding: utf-8 -*-

"""All Predesessor Following 
"""

# Author: XH <...>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import numpy as np

from enum import Enum
from collections import deque
from opencda.core.common.misc import get_speed


class AllPredesessorFollowing(object):

    def __init__(self, vehicle_manager_list=[],
                 current_in_id=0,
                 vehicle_length=4.0,
                 ss_theta=4.0,
                 minAllowHeadway=0.5,
                 maxAllowHeadway=1.8):

        # construction variabes
        self.vehicle_manager_list = vehicle_manager_list
        self.current_in_id = current_in_id
        self.current_dynamic_leader_index = ""
        self.previous_dynamic_leader_index = ""

        # functional variabes
        self.dynamic_leader_index_buffer = deque(maxlen=5)
        # populate the index buffer with initial "previous_dynamic_leader_index" value
        # self.dynamic_leader_index_buffer.append(self.previous_dynamic_leader_index)
        self.timeHeadways = []
        self.partialTimeHeadways = []

        # default functional parameters
        self.vehicle_length = vehicle_length
        self.ss_theta = ss_theta
        # self.minAllowHeadway = 0.5 ( parameter that can pass unit test)
        self.minAllowHeadway = minAllowHeadway
        self.maxAllowHeadway = maxAllowHeadway
        # self.headawayStableLowerBond = 0.7
        self.headawayStableLowerBond = self.minAllowHeadway + 0.16
        # self.headawayStableUpperBond = 1.7
        self.headawayStableUpperBond = self.maxAllowHeadway - 0.16

        self.minAllowDistGap = 2
        self.maxStableGap = 2.3

    def update_platoon_info(self, vehicle_manager_list,
                            current_in_id,
                            vehicle_length,
                            ss_theta,
                            minAllowHeadway,
                            maxAllowHeadway
                            ):
        """
        Update platoon information.
        """
        self.vehicle_manager_list = vehicle_manager_list
        self.current_in_id = current_in_id
        self.vehicle_length = vehicle_length
        self.ss_theta = ss_theta
        self.minAllowHeadway = minAllowHeadway
        self.maxAllowHeadway = maxAllowHeadway

    def run_step(self):
        """
		Update the variables of the APF algorithm.
		"""
        # self.current_dynamic_leader_index = run_APF(self.vehicle_manager_list,
        # 									   		self.current_in_id,
        # 									   		self.previous_dynamic_leader_index)

        current_dynamic_leader_index = self.run_APF()
        self.dynamic_leader_index_buffer.append(current_dynamic_leader_index)
        self.current_dynamic_leader_index = current_dynamic_leader_index
        # if first time running APF, automatic assign previous leader as 0 to reduce one cycle.
        if len(self.dynamic_leader_index_buffer) <= 1:
            self.previous_dynamic_leader_index = current_dynamic_leader_index
        else:
            self.previous_dynamic_leader_index = self.dynamic_leader_index_buffer[-1]

        return current_dynamic_leader_index

    def reset(self):
        """
		Reset APF functional variabes.
		"""
        self.previous_dynamic_leader_index = ""
        self.timeHeadways = []
        self.partialTimeHeadways = []
        self.dynamic_leader_index_buffer.clear()

    def calculate_dist_2D(self, front_v_loc, host_v_loc):
        """
		Calculate distance in 2D space (x,y).
		to do: use downtrack dist and cross track dist. 
		"""
        dist_x = abs(front_v_loc.x - host_v_loc.x)
        dist_y = abs(front_v_loc.y - host_v_loc.y)

        distance = math.sqrt(dist_x ** 2 + dist_y ** 2)

        # return dist_x, dist_y, distance
        return distance

    def calculateTimeHeadway(self, vehicle_manager_list):
        """
		Generate a time headway list for all platoon members
		"""

        timeHeadways = []
        # the headway list is one item less than vehicle_manager_list
        for i in range(len(vehicle_manager_list) - 1):
            # read values
            vehicle = vehicle_manager_list[i + 1].vehicle
            speed = get_speed(vehicle, meters=True)

            # compare with standstill threshold
            if speed >= self.ss_theta:
                front_v_loc = vehicle_manager_list[i].vehicle.get_location()
                rear_v_loc = vehicle_manager_list[i + 1].vehicle.get_location()
                dist_2D = self.calculate_dist_2D(front_v_loc, rear_v_loc)
                # populate time headway
                timeHeadways.append(dist_2D / speed)

            # inf headway if host standstill
            else:
                timeHeadways.append(float('inf'))

        return timeHeadways

    def findMinViolationClosestToTheHost(self, timeHeadways):
        """
		Loop over time headway list to find the minimum violator's index
		"""
        for i in range(len(timeHeadways) - 1, -1, -1):
            if timeHeadways[i] <= self.minAllowHeadway:
                return i
        return -1

    def findMaxViolationClosestToTheHost(self, timeHeadways):
        """
		Loop over time headway list to find the maximum violator's index
		"""
        for i in range(len(timeHeadways)-1, -1, -1):
            curr_headway = timeHeadways[i]
            if timeHeadways[i] > self.maxAllowHeadway:
                return i
        return -1

    def determineDynamicLeaderBasedOnViolation(self, timeHeadways):
        # find closest violations
        closestMinViolation = self.findMinViolationClosestToTheHost(timeHeadways)
        closestMaxViolation = self.findMaxViolationClosestToTheHost(timeHeadways)

        # compare violation indexes, return the larger (i.e., closer to host) index
        if closestMinViolation > closestMaxViolation:
            return closestMinViolation if closestMinViolation < self.current_in_id \
                else self.current_dynamic_leader_index

        elif closestMinViolation < closestMaxViolation:
            return closestMaxViolation + 1 if closestMaxViolation + 1 < self.current_in_id \
                else self.current_dynamic_leader_index

        else:
            return 0

    def APF_with_prior_leader(self,
                              timeHeadways,
                              partialTimeHeadways,
                              previous_dynamic_leader_index):
        """
    	Conditions where previous dynamic leader is not empty.
    	Case four to nine.
    	"""
        # Calculate violation index based on partial headways.
        closestMinViolation = self.findMinViolationClosestToTheHost(partialTimeHeadways)
        closestMaxViolation = self.findMaxViolationClosestToTheHost(partialTimeHeadways)

        # if no violation at all
        if (closestMinViolation == -1 and closestMaxViolation == -1):
            front_gap_stable = timeHeadways[previous_dynamic_leader_index] > self.headawayStableLowerBond
            rear_gap_stable = timeHeadways[previous_dynamic_leader_index - 1] < self.headawayStableUpperBond

            # [CASE FOUR]: Front and rear gap stable, reselect dynamic leader
            if (front_gap_stable and rear_gap_stable):
                return self.determineDynamicLeaderBasedOnViolation(timeHeadways)

            # [CASE FIVE]: Cannot switch leader, unstable gap exist
            else:
                return previous_dynamic_leader_index

        # [CASE SIX]: min violation exist in partial headway, change dynamic leader accordingly.
        elif (closestMinViolation != -1 and closestMaxViolation == -1):
            return previous_dynamic_leader_index - 1 + closestMinViolation

        # [CASE SEVEN]: max violation exist in partial headway, change dynamic leader accordingly.
        elif (closestMinViolation == -1 and closestMaxViolation != -1):
            return previous_dynamic_leader_index + closestMaxViolation

        # both violation exists
        else:
            # [CASE EIGHT]: Min violation has larger index (closer to host), hence addressed sooner
            if closestMinViolation > closestMaxViolation:
                return previous_dynamic_leader_index - 1 + closestMinViolation

            # [CASE NINE]: Max violation has larger index (closer to host), hence addressed sooner
            elif closestMinViolation < closestMaxViolation:
                return previous_dynamic_leader_index + closestMaxViolation

            else:
                # APF calculation error
                return 0

    def run_APF(self):

        # [CASE ZERO]: the host vehicle is the second follower of a platoon
        if self.current_in_id == 1:
            return 0

        # [CASE ONE]: If no previous leader, return platoon leader as default
        if self.previous_dynamic_leader_index == "":
            return 0

        # check the gap with immediate preceding vehicle
        host_v_loc = self.vehicle_manager_list[self.current_in_id].vehicle.get_location()
        front_v_loc = self.vehicle_manager_list[self.current_in_id - 1].vehicle.get_location()
        dist_2D = self.calculate_dist_2D(front_v_loc, host_v_loc)

        # check front gap
        frontGapIsTooSmall = (dist_2D < self.minAllowDistGap)
        frontGapIsNotLargeEnough = (dist_2D < self.maxStableGap and
                                    self.previous_dynamic_leader_index == self.current_in_id - 1)

        # [CASE TWO]: If the immediate gap is small, follow preceding vehicle
        if (frontGapIsTooSmall or frontGapIsNotLargeEnough):
            return self.current_in_id - 1

        # Otherwise, check other vehicles
        else:
            # calculate time headway list for all members
            self.timeHeadways = self.calculateTimeHeadway(self.vehicle_manager_list)

            # [CASE THREE]: If the host was following the leader (i.e., no violation), follow violators
            if self.previous_dynamic_leader_index == 0:
                return self.determineDynamicLeaderBasedOnViolation(self.timeHeadways)

            # Otherwise check previous dynamic leader and move toward downstream direction
            else:
                # Only consider partial time headway list, starting with the gap in front of the previous leader
                start_index = self.previous_dynamic_leader_index - 1
                self.partialTimeHeadways = self.timeHeadways[start_index:]

                # the rest of the logic covers [Case four] to [Case nine]
                dynamic_leader_index = self.APF_with_prior_leader(self.timeHeadways,
                                                                  self.partialTimeHeadways,
                                                                  self.previous_dynamic_leader_index)
                return dynamic_leader_index
