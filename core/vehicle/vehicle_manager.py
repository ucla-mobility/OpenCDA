# -*- coding: utf-8 -*-

"""A class manager to embed different plugins with vehicle
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import uuid
import weakref
import statistics

from core.agents.navigation.platoon_behavior_agent import PlatooningBehaviorAgent
from core.agents.tools.misc import get_speed
from core.platooning.platooning_plugin import PlatooningPlugin
from core.platooning.fsm import FSM


class VehicleManager(object):
    """
    A class manager to embed different modules with vehicle together
    """

    def __init__(self, vehicle, world, behavior='normal', communication_range=35, update_freq=15,
                 buffer_size=8, sample_resolution=4.5, cda_enabled=True, status=FSM.MAINTINING, time_ahead=1.1,
                 ignore_traffic_light=False, overtake_allowed=False, debug_trajectory=False, debug=False):
        """
        Construct class
        :param vehicle: carla Actor
        :param world: platooning world object
        :param behavior: driving style.
        :param communication_range:
        :param buffer_size: queue size for behavior planning
        :param sample_resolution: the minimum distance between any waypoint in the routing
        :param cda_enabled:  whether the vehicle equipped with cda feature
        """
        self.vid = str(uuid.uuid1())
        self.destination = None

        self.vehicle = vehicle
        self.agent = PlatooningBehaviorAgent(vehicle, behavior=behavior, ignore_traffic_light=ignore_traffic_light,
                                             buffer_size=buffer_size, sampling_resolution=sample_resolution,
                                             overtake_allowed=overtake_allowed, time_ahead=time_ahead,
                                             debug_trajectory=debug_trajectory, debug=debug, update_freq=update_freq)

        self._platooning_plugin = PlatooningPlugin(cda_enabled, status=status, search_range=communication_range)

        world.update_vehicle_manager(self)
        self.world = weakref.ref(world)()

    def get_platooning_status(self):
        """
        Check whether the vehicle in the platooning
        :return:
        """
        return self._platooning_plugin.in_platooning, self._platooning_plugin.id_in_team, \
               self._platooning_plugin.platooning_object

    def get_pmid(self):
        return self._platooning_plugin.platooning_id

    def get_fsm_status(self):
        """
        return the fsm status
        :return:
        """
        return self._platooning_plugin.status

    def set_platooning_status(self, status):
        """
        Set the platooning status
        :param status:
        :return:
        """
        self._platooning_plugin.status = status

    def set_platooning(self, platooning_object, platooning_id, in_team_id, lead=False):
        """
        Called when vehicle joined/formed a platooning
        :param in_team_id:
        :param lead:
        :param platooning_object: the platooning object
        :param platooning_id:
        :return:
        """
        self._platooning_plugin.platooning_id = platooning_id
        self._platooning_plugin.in_platooning = True
        self._platooning_plugin.platooning_object = weakref.ref(platooning_object)()
        self._platooning_plugin.id_in_team = in_team_id

        if lead:
            self._platooning_plugin.take_charge()

    def unset_lead(self):
        """
        cancel lead position
        :return:
        """
        self._platooning_plugin.leader = False

    def update_info(self, world, frontal_vehicle=None):
        """
        Update the world and platooning information
        :param world:
        :param frontal_vehicle:
        :return:
        """
        self.agent.update_information(world, frontal_vehicle)

    def cal_performance(self):
        """
        Quantitive way to judge the peroformance of the system
        :return:
        """
        time_gap_list = self.agent.time_gap_list[60:]
        velocity_list = self.agent.velocity_list[60:]

        print(len(time_gap_list))
        print("the mean of the time gap is %f and std is %f" % (statistics.mean(time_gap_list),
                                                                statistics.stdev(time_gap_list)))
        print("the mean of the velocity is %f and std is %f" % (statistics.mean(velocity_list),
                                                                statistics.stdev(velocity_list)))

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation based on platooning status
        :return:
        """
        # get current speed
        self.agent.velocity_list.append(get_speed(self.vehicle, True))
        # acceleration list
        if len(self.agent.velocity_list) > 1:
            self.agent.acceleration_list.append((self.agent.velocity_list[-1] - self.agent.velocity_list[-2])/0.05)
        else:
            self.agent.acceleration_list.append(self.agent.velocity_list[-1] / 0.05)

        # TODO: Right now take lead is not in consideration
        if not self._platooning_plugin.in_platooning:
            # if the ego-vehicle is still searching for platooning
            if self._platooning_plugin.status == FSM.SEARCHING:
                platoon_searched, distance, min_index, platoon_manager = \
                    self._platooning_plugin.platooning_search(self.vid, self.world, self.vehicle)
                # if find platooning and platooning is close enough, we use cut-joining
                if platoon_searched and self._platooning_plugin.front_vehicle \
                        and self._platooning_plugin.rear_vechile:
                    print('cut in joining mode')
                    self.set_platooning_status(FSM.MOVE_TO_POINT)

                elif platoon_searched and self._platooning_plugin.front_vehicle \
                        and not self._platooning_plugin.rear_vechile:
                    print('back joining mode')
                    self.set_platooning_status(FSM.BACK_JOINING)

                elif platoon_searched and min_index == 0 \
                        and not self._platooning_plugin.front_vehicle:
                    print('frontal joining mode')
                    self.set_platooning_status(FSM.FRONT_JOINING)

                return self.agent.run_step()

            # if the vehicle already find the platooning and trying to move to the merge point
            elif self._platooning_plugin.status == FSM.MOVE_TO_POINT:
                control, ready_to_join, transition_to_back_joining = \
                    self.agent.run_step_cut_in_move2point(self._platooning_plugin.front_vehicle,
                                                          self._platooning_plugin.rear_vechile)
                # change from cut_in_joining to back joining
                if transition_to_back_joining:
                    print('transition to back joining')
                    _, index, platooning_manager = self._platooning_plugin.front_vehicle.get_platooning_status()
                    self._platooning_plugin.front_vehicle = platooning_manager.vehicle_manager_list[-1]
                    self._platooning_plugin.rear_vechile = None
                    self.set_platooning_status(FSM.BACK_JOINING)

                if ready_to_join:
                    self.set_platooning_status(FSM.JOINING)
                return control

            # if the vehicle arrives at the meeting point and ready for merging
            elif self._platooning_plugin.status == FSM.JOINING:
                control, joining_finished = self.agent.run_step_cut_in_joining(self._platooning_plugin.front_vehicle,
                                                                               self._platooning_plugin.rear_vechile)

                if joining_finished:
                    _, index, platooning_manager = self._platooning_plugin.front_vehicle.get_platooning_status()
                    platooning_manager.set_member(self, index + 1)
                    self.set_platooning_status(FSM.MAINTINING)
                return control

            # if the vehicle is doing back joining
            elif self._platooning_plugin.status == FSM.BACK_JOINING:
                control, joining_finished = self.agent.run_step_back_joining(self._platooning_plugin.front_vehicle)

                if joining_finished:
                    _, index, platooning_manager = self._platooning_plugin.front_vehicle.get_platooning_status()
                    platooning_manager.set_member(self, index + 1)
                    self.set_platooning_status(FSM.MAINTINING)
                return control

            elif self._platooning_plugin.status == FSM.FRONT_JOINING:
                rear_vehicle = self._platooning_plugin.rear_vechile
                control, status = self.agent.run_step_front_joining(rear_vehicle)
                _, index, platooning_manager = rear_vehicle.get_platooning_status()

                if status == FSM.SEARCHING:
                    self.set_platooning_status(status)
                    self._platooning_plugin.platooning_black_list.append(
                        rear_vehicle.get_pmid())
                if status == FSM.BACK_JOINING:
                    self._platooning_plugin.front_vehicle = platooning_manager.vehicle_manager_list[-1]
                    self._platooning_plugin.rear_vechile = None
                    self.set_platooning_status(FSM.BACK_JOINING)
                if status == FSM.JOINING_FINISHED:
                    # first take away old leader position
                    rear_vehicle.unset_lead()
                    rear_vehicle.set_platooning_status(FSM.MAINTINING)
                    # add lead position to ego
                    platooning_manager.set_member(self, 0, True)
                return control

        else:
            if self._platooning_plugin.leader:
                control = self.agent.run_step(target_speed)
            elif self._platooning_plugin.status == FSM.OPEN_GAP:
                print("opening gap with speed %d!" % get_speed(self.vehicle))
                control = self.agent.run_step_open_gap()
            else:
                control = self.agent.run_step_maintaining()

            return control
