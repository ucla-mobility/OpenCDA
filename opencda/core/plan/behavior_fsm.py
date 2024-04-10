# -*- coding: utf-8 -*-

"""Behavior planning module
"""

# Author: Xu Han <>
# License: TDG-Attribution-NonCommercial-NoDistrib


import math
import random
import sys

import numpy as np
import networkx as nx
import carla

from opencda.core.plan.behavior_fsm_states import BehaviorSuperStates, BehaviorStates


class BehaviorFSM(object):
    """
    A modulized version of carla BehaviorAgent.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    Attributes
    ----------
    _ego_pos : carla.position
        Posiion of the ego vehicle.

    """

    def __init__(self, local_planner):  # consider add vehicle as variable when calculating cost, vehicle):

        # self.vehicle = vehicle
        # ego pos(transform) and speed(km/h) retrieved from localization module
        self._ego_pos = None
        self._ego_speed = 0.0
        self._local_planner = local_planner

        # Create superstates
        self.superstate_graph = nx.DiGraph()
        self.superstates = {
            # "lane_following": self.lane_following_graph,
            "LANE_FOLLOWING": BehaviorSuperStates.LANE_FOLLOWING,
            "INTERSECTION": BehaviorSuperStates.INTERSECTION,
            "OVERTAKING": BehaviorSuperStates.OVERTAKING,
            "COLLISION_AVOIDANCE": BehaviorSuperStates.COLLISION_AVOIDANCE,
        }
        self.init_superstate_transitions()

        # Create states
        self.states = {
            "GO_STRAIGHT": BehaviorStates.GO_STRAIGHT,
            "PREPARE_LANE_CHANGE_LEFT": BehaviorStates.PREPARE_LANE_CHANGE_LEFT,
            "PREPARE_LANE_CHANGE_RIGHT": BehaviorStates.PREPARE_LANE_CHANGE_RIGHT,
            "LANE_CHANGE_LEFT": BehaviorStates.LANE_CHANGE_LEFT,
            "LANE_CHANGE_RIGHT": BehaviorStates.LANE_CHANGE_RIGHT,
            "CAR_FOLLOWING": BehaviorStates.CAR_FOLLOWING,
            "TURN_LEFT": BehaviorStates.TURN_LEFT,
            "TURN_RIGHT": BehaviorStates.TURN_RIGHT,
            "STOP": BehaviorStates.STOP,
        }
        # Define state transitions for each superstates
        # Lane following
        self.lane_following_graph = nx.DiGraph()
        self.init_lane_following_transitions()
        # Intersection
        self.intersection_graph = nx.DiGraph()
        self.init_intersection_transitions()
        # Overtaking
        self.overtaking_graph = nx.DiGraph()
        self.init_overtaking_transitions()

        # Define initial superstate
        self.current_superstate = self.superstates["LANE_FOLLOWING"]
        self.current_state = self.states["GO_STRAIGHT"]

        # Current prompt for vision-language model
        self.prompt = ""

        # Lane change
        self.prepare_lane_change_counter = 0
        self.give_up_lane_change = False
        self.reset_give_up_lane_change_counter = 10

    def init_superstate_transitions(self):
        # Add nodes to the graph
        for superstate in self.superstates.values():
            self.superstate_graph.add_node(superstate)

        # Define transitions for superstates
        # start in lane following
        self.superstate_graph.add_edge(self.superstates["LANE_FOLLOWING"], self.superstates["INTERSECTION"])
        self.superstate_graph.add_edge(self.superstates["LANE_FOLLOWING"], self.superstates["OVERTAKING"])
        self.superstate_graph.add_edge(self.superstates["LANE_FOLLOWING"], self.superstates["COLLISION_AVOIDANCE"])
        self.superstate_graph.add_edge(self.superstates["LANE_FOLLOWING"], self.superstates["LANE_FOLLOWING"])
        # start in intersection
        self.superstate_graph.add_edge(self.superstates["INTERSECTION"], self.superstates["LANE_FOLLOWING"])
        self.superstate_graph.add_edge(self.superstates["INTERSECTION"], self.superstates["COLLISION_AVOIDANCE"])
        self.superstate_graph.add_edge(self.superstates["INTERSECTION"], self.superstates["INTERSECTION"])
        # start in overtaking
        self.superstate_graph.add_edge(self.superstates["OVERTAKING"], self.superstates["LANE_FOLLOWING"])
        self.superstate_graph.add_edge(self.superstates["OVERTAKING"], self.superstates["COLLISION_AVOIDANCE"])
        self.superstate_graph.add_edge(self.superstates["OVERTAKING"], self.superstates["OVERTAKING"])
        # start in collision avoidance
        self.superstate_graph.add_edge(self.superstates["COLLISION_AVOIDANCE"], self.superstates["LANE_FOLLOWING"])
        self.superstate_graph.add_edge(self.superstates["COLLISION_AVOIDANCE"], self.superstates["COLLISION_AVOIDANCE"])

    def init_lane_following_transitions(self):
        # Add nodes to the graph
        for state in self.states.values():
            self.lane_following_graph.add_node(state)

        # Define transitions
        self.lane_following_graph.add_edge(self.states["GO_STRAIGHT"], self.states["GO_STRAIGHT"])
        self.lane_following_graph.add_edge(self.states["GO_STRAIGHT"], self.states["PREPARE_LANE_CHANGE_LEFT"])
        self.lane_following_graph.add_edge(self.states["GO_STRAIGHT"], self.states["PREPARE_LANE_CHANGE_RIGHT"])
        self.lane_following_graph.add_edge(self.states["GO_STRAIGHT"], self.states["CAR_FOLLOWING"])

        self.lane_following_graph.add_edge(self.states["PREPARE_LANE_CHANGE_LEFT"],
                                           self.states["PREPARE_LANE_CHANGE_LEFT"])
        self.lane_following_graph.add_edge(self.states["PREPARE_LANE_CHANGE_LEFT"], self.states["LANE_CHANGE_LEFT"])
        self.lane_following_graph.add_edge(self.states["PREPARE_LANE_CHANGE_LEFT"], self.states["GO_STRAIGHT"])

        self.lane_following_graph.add_edge(self.states["LANE_CHANGE_LEFT"], self.states["LANE_CHANGE_LEFT"])
        self.lane_following_graph.add_edge(self.states["LANE_CHANGE_LEFT"], self.states["GO_STRAIGHT"])

        self.lane_following_graph.add_edge(self.states["PREPARE_LANE_CHANGE_RIGHT"],
                                           self.states["PREPARE_LANE_CHANGE_RIGHT"])
        self.lane_following_graph.add_edge(self.states["PREPARE_LANE_CHANGE_RIGHT"], self.states["LANE_CHANGE_RIGHT"])
        self.lane_following_graph.add_edge(self.states["PREPARE_LANE_CHANGE_RIGHT"], self.states["GO_STRAIGHT"])

        self.lane_following_graph.add_edge(self.states["LANE_CHANGE_RIGHT"], self.states["LANE_CHANGE_RIGHT"])
        self.lane_following_graph.add_edge(self.states["LANE_CHANGE_RIGHT"], self.states["GO_STRAIGHT"])

        self.lane_following_graph.add_edge(self.states["CAR_FOLLOWING"], self.states["CAR_FOLLOWING"])
        self.lane_following_graph.add_edge(self.states["CAR_FOLLOWING"], self.states["GO_STRAIGHT"])

    def init_intersection_transitions(self):
        # Add nodes to the graph
        for state in self.states.values():
            self.intersection_graph.add_node(state)

        # Define transitions
        self.intersection_graph.add_edge(self.states["GO_STRAIGHT"], self.states["GO_STRAIGHT"])
        self.intersection_graph.add_edge(self.states["GO_STRAIGHT"], self.states["TURN_LEFT"])
        self.intersection_graph.add_edge(self.states["GO_STRAIGHT"], self.states["TURN_RIGHT"])
        self.intersection_graph.add_edge(self.states["GO_STRAIGHT"], self.states["CAR_FOLLOWING"])
        self.intersection_graph.add_edge(self.states["GO_STRAIGHT"], self.states["STOP"])

        self.intersection_graph.add_edge(self.states["TURN_LEFT"], self.states["TURN_LEFT"])
        self.intersection_graph.add_edge(self.states["TURN_LEFT"], self.states["GO_STRAIGHT"])
        self.intersection_graph.add_edge(self.states["TURN_LEFT"], self.states["CAR_FOLLOWING"])
        self.intersection_graph.add_edge(self.states["TURN_LEFT"], self.states["STOP"])

        self.intersection_graph.add_edge(self.states["TURN_RIGHT"], self.states["TURN_RIGHT"])
        self.intersection_graph.add_edge(self.states["TURN_RIGHT"], self.states["GO_STRAIGHT"])
        self.intersection_graph.add_edge(self.states["TURN_RIGHT"], self.states["CAR_FOLLOWING"])
        self.intersection_graph.add_edge(self.states["TURN_RIGHT"], self.states["STOP"])

        self.intersection_graph.add_edge(self.states["STOP"], self.states["STOP"])
        self.intersection_graph.add_edge(self.states["STOP"], self.states["GO_STRAIGHT"])
        self.intersection_graph.add_edge(self.states["STOP"], self.states["CAR_FOLLOWING"])
        self.intersection_graph.add_edge(self.states["STOP"], self.states["TURN_LEFT"])
        self.intersection_graph.add_edge(self.states["STOP"], self.states["TURN_RIGHT"])

        self.intersection_graph.add_edge(self.states["CAR_FOLLOWING"], self.states["CAR_FOLLOWING"])
        self.intersection_graph.add_edge(self.states["CAR_FOLLOWING"], self.states["GO_STRAIGHT"])
        self.intersection_graph.add_edge(self.states["CAR_FOLLOWING"], self.states["TURN_LEFT"])
        self.intersection_graph.add_edge(self.states["CAR_FOLLOWING"], self.states["TURN_RIGHT"])
        self.intersection_graph.add_edge(self.states["CAR_FOLLOWING"], self.states["STOP"])

    def init_overtaking_transitions(self):
        # Add nodes to the graph
        for state in self.states.values():
            self.overtaking_graph.add_node(state)

        # Define transitions
        self.overtaking_graph.add_edge(self.states["GO_STRAIGHT"], self.states["GO_STRAIGHT"])
        self.overtaking_graph.add_edge(self.states["GO_STRAIGHT"], self.states["PREPARE_LANE_CHANGE_LEFT"])
        self.overtaking_graph.add_edge(self.states["GO_STRAIGHT"], self.states["PREPARE_LANE_CHANGE_RIGHT"])

        self.overtaking_graph.add_edge(self.states["PREPARE_LANE_CHANGE_LEFT"],
                                           self.states["PREPARE_LANE_CHANGE_LEFT"])
        self.overtaking_graph.add_edge(self.states["PREPARE_LANE_CHANGE_LEFT"], self.states["LANE_CHANGE_LEFT"])
        self.overtaking_graph.add_edge(self.states["PREPARE_LANE_CHANGE_LEFT"], self.states["GO_STRAIGHT"])

        self.overtaking_graph.add_edge(self.states["LANE_CHANGE_LEFT"], self.states["LANE_CHANGE_LEFT"])
        self.overtaking_graph.add_edge(self.states["LANE_CHANGE_LEFT"], self.states["PREPARE_LANE_CHANGE_RIGHT"])
        self.overtaking_graph.add_edge(self.states["LANE_CHANGE_LEFT"], self.states["GO_STRAIGHT"])

        self.overtaking_graph.add_edge(self.states["PREPARE_LANE_CHANGE_RIGHT"],
                                           self.states["PREPARE_LANE_CHANGE_RIGHT"])
        self.overtaking_graph.add_edge(self.states["PREPARE_LANE_CHANGE_RIGHT"], self.states["LANE_CHANGE_RIGHT"])
        self.overtaking_graph.add_edge(self.states["PREPARE_LANE_CHANGE_RIGHT"], self.states["GO_STRAIGHT"])

        self.overtaking_graph.add_edge(self.states["LANE_CHANGE_RIGHT"], self.states["LANE_CHANGE_RIGHT"])
        self.overtaking_graph.add_edge(self.states["LANE_CHANGE_RIGHT"], self.states["PREPARE_LANE_CHANGE_LEFT"])
        self.overtaking_graph.add_edge(self.states["LANE_CHANGE_RIGHT"], self.states["GO_STRAIGHT"])

    def get_current_superstate(self):
        return self.current_superstate

    def get_current_state(self):
        return self.current_state

    def get_possible_next_superstates(self):
        return [state.name for state in self.superstate_graph.neighbors(self.current_superstate)]

    def get_possible_next_states(self, possible_next_superstate_names):
        possible_next_states_names = []
        # if superstate not transitioned
        if self.current_superstate.name in possible_next_superstate_names:
            # search current graph
            # case 1. LANE_FOLLOWING
            if self.current_superstate.name == 'LANE_FOLLOWING':
                # search graph
                possible_next_states_names += [state.name for state in \
                                               self.lane_following_graph.neighbors(self.current_state)]
            # case 2. INTERSECTION
            elif self.current_superstate.name == 'INTERSECTION':
                # search graph
                possible_next_states_names += [state.name for state in \
                                               self.intersection_graph.neighbors(self.current_state)]
            # following case wait to be implemented
        # if superstate transitioned, it is only possible to start in "go straight" state.
        else:
            possible_next_states_names.append(self.states["GO_STRAIGHT"].name)
        return possible_next_states_names

    def get_next_states_based_on_one_superstate(self, best_next_superstate):
        possible_next_states_names = []
        if self.current_superstate.name == best_next_superstate:
            if self.current_superstate.name == 'LANE_FOLLOWING':
                # search graph
                possible_next_states_names += [state.name for state in \
                                               self.lane_following_graph.neighbors(self.current_state)]
            # case 2. INTERSECTION
            elif self.current_superstate.name == 'INTERSECTION':
                # search graph
                possible_next_states_names += [state.name for state in \
                                               self.intersection_graph.neighbors(self.current_state)]
            # case 3. OVERTAKE
            elif self.current_superstate.name == 'OVERTAKING':
                # search graph
                possible_next_states_names += [state.name for state in \
                                               self.overtaking_graph.neighbors(self.current_state)]
        else:
            possible_next_states_names.append(self.states["GO_STRAIGHT"].name)
        return possible_next_states_names

    def rank_next_superstates(self,
                              next_superstates,
                              is_intersection, is_hazard,
                              is_red_light, lane_change_allowed,
                              is_overtake_proper,
                              is_obstacle_confirmed):
        nxt_superstates = {}
        if 'LANE_FOLLOWING' in next_superstates:
            if not is_intersection and not is_hazard:
                nxt_superstates['LANE_FOLLOWING'] = 5
            else:
                nxt_superstates['LANE_FOLLOWING'] = 10
        if 'INTERSECTION' in next_superstates:
            if is_intersection:
                nxt_superstates['INTERSECTION'] = 5
            else:
                nxt_superstates['INTERSECTION'] = 10
        if 'OVERTAKING' in next_superstates:
            # condition where overtaking is in favor
            if is_hazard and is_overtake_proper:
                nxt_superstates['OVERTAKING'] = 5 if is_obstacle_confirmed else 10
            else:
                nxt_superstates['OVERTAKING'] = 200
        if 'COLLISION_AVOIDANCE' in next_superstates:
            if is_hazard:
                nxt_superstates['COLLISION_AVOIDANCE'] = 100
            else:
                nxt_superstates['COLLISION_AVOIDANCE'] = 100
        # rank dict
        sorted_superstates = dict(sorted(nxt_superstates.items(), key=lambda item: item[1]))
        return sorted_superstates

    # state transition #0 GO_STRAIGHT
    def state_transition_go_straight(self, next_states,
                                     is_lane_change_ahead=None,
                                     is_red_light=None,
                                     is_hazard=None,
                                     overtake_left=False,
                                     overtake_right=False):
        # state transition started in go straight state as current state
        path = {}
        # generate path
        rx, ry, rk, ryaw = self._local_planner.generate_path()
        # reset give up lane change marker
        if self.give_up_lane_change:
            # previous lane change was blocked, start a counter to reset it
            self.reset_give_up_lane_change_counter -= 1
        if self.reset_give_up_lane_change_counter == 0:
            # reset marker and counter
            self.give_up_lane_change = False
            self.reset_give_up_lane_change_counter = 10
        # transitions associate with costs
        if self.current_superstate.name == 'LANE_FOLLOWING':
            if 'GO_STRAIGHT' in next_states:
                cost = 10
                path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'PREPARE_LANE_CHANGE_LEFT' in next_states:
                if is_lane_change_ahead:
                    cost = 5
                else:
                    cost = 15
                path['PREPARE_LANE_CHANGE_LEFT'] = [rx, ry, rk, ryaw, cost]
            if 'PREPARE_LANE_CHANGE_RIGHT' in next_states:
                if is_lane_change_ahead:
                    cost = 1
                else:
                    cost = 15
                path['PREPARE_LANE_CHANGE_RIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'CAR_FOLLOWING' in next_states:
                if is_hazard:
                    cost = 5
                else:
                    cost = 30
                path['CAR_FOLLOWING'] = [rx, ry, rk, ryaw, cost]
        elif self.current_superstate.name == 'INTERSECTION':
            # determine next step turn
            is_left_turn_ahead, is_right_turn_ahead = self._local_planner.is_turn_ahead()
            # check all next states
            if 'GO_STRAIGHT' in next_states:
                if is_red_light or is_left_turn_ahead or is_right_turn_ahead:
                    # do not proceed if red light or turns ahead
                    cost = 30
                else:
                    cost = 1

            if 'TURN_RIGHT' in next_states:
                # prepare right turn
                # todo: Add regulation here to determine right turn on red, hardcode for now
                is_right_turn_on_red_legal = False
                if is_right_turn_ahead and is_red_light:
                    # go ahead on red
                    cost = 10 if is_right_turn_on_red_legal else 30
                elif is_right_turn_ahead and not is_red_light:
                    # go ahead on green
                    cost = 10
                else:
                    # do not proceed on all other condition
                    cost = 30
                path['TURN_RIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'TURN_LEFT' in next_states:
                # prepare left turn
                # todo: Add regulation here to determine right turn on red, hardcode for now
                is_left_turn_on_red_legal = False
                if is_left_turn_ahead and is_red_light:
                    # go ahead on red
                    cost = 10 if is_left_turn_on_red_legal else 30
                elif is_left_turn_ahead and not is_red_light:
                    # go ahead on green
                    cost = 10
                else:
                    # do not proceed on all other condition
                    cost = 30
                path['TURN_LEFT'] = [rx, ry, rk, ryaw, cost]
            if 'CAR_FOLLOWING' in next_states:
                if is_hazard:
                    cost = 5
                else:
                    cost = 30
                path['CAR_FOLLOWING'] = [rx, ry, rk, ryaw, cost]
            if 'STOP' in next_states:
                if is_red_light and not \
                        (is_left_turn_ahead or is_right_turn_ahead):
                    # stop at red
                    cost = 3
                else:
                    cost = 30
                path['STOP'] = [rx, ry, rk, ryaw, cost]
        elif self.current_superstate.name == 'OVERTAKING':
            if 'GO_STRAIGHT' in next_states:
                cost = 10
                path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'PREPARE_LANE_CHANGE_LEFT' in next_states:
                if overtake_left:
                    cost = 5
                else:
                    cost = 15
                path['PREPARE_LANE_CHANGE_LEFT'] = [rx, ry, rk, ryaw, cost]
            if 'PREPARE_LANE_CHANGE_RIGHT' in next_states:
                if overtake_right:
                    cost = 5
                else:
                    cost = 15
                path['PREPARE_LANE_CHANGE_RIGHT'] = [rx, ry, rk, ryaw, cost]

        return path

    # state transition #1 PREPARE_LANE_CHANGE_LEFT
    def state_transition_prepare_left_lane_change(self, next_states, is_lane_change_ahead,
                                                  lane_change_allowed, is_target_lane_safe):
        path = {}
        # increase counter
        self.prepare_lane_change_counter += 1
        # check for obstacle on target lane (loop through all near-by actors)
        is_blocked = lane_change_allowed  # todo: determine by original logic in OpenCDA behavior agent
        # generate path
        # lane following superstate
        if self.current_superstate.name == 'LANE_FOLLOWING':
            rx, ry, rk, ryaw = self._local_planner.generate_path()
            if 'GO_STRAIGHT' in next_states:
                if is_blocked:
                    cost = 5
                else:
                    cost = 15
                path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'LANE_CHANGE_LEFT' in next_states:
                if not is_blocked:
                    # start lane change, mark starting point
                    self._local_planner.mark_lane_change_start()
                    cost = 5
                else:
                    cost = 15
                path['LANE_CHANGE_LEFT'] = [rx, ry, rk, ryaw, cost]
            if 'PREPARE_LANE_CHANGE_LEFT' in next_states:
                if is_lane_change_ahead and \
                        self.prepare_lane_change_counter <= 2 and \
                        is_blocked:
                    # if prepare counter small, still in favor of wait
                    cost = 3
                elif is_lane_change_ahead and \
                        self.prepare_lane_change_counter > 2 and \
                        is_blocked:
                    # give up lane change
                    self.give_up_lane_change = True
                    cost = 15
                    # reset counter
                    self.prepare_lane_change_counter = 0
                else:
                    # proceed to lane change if not blocked
                    cost = 15
                    # reset counter
                    self.prepare_lane_change_counter = 0
                path['PREPARE_LANE_CHANGE_LEFT'] = [rx, ry, rk, ryaw, cost]
        # overtaking superstate
        elif self.current_superstate.name == 'OVERTAKING':
            rx, ry, rk, ryaw = self._local_planner.generate_path()
            if 'GO_STRAIGHT' in next_states:
                if not is_target_lane_safe:
                    cost = 5
                else:
                    cost = 15
                path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'LANE_CHANGE_LEFT' in next_states:
                if is_target_lane_safe:
                    # start lane change, mark starting point
                    self._local_planner.mark_lane_change_start()
                    cost = 5
                else:
                    cost = 15
                path['LANE_CHANGE_LEFT'] = [rx, ry, rk, ryaw, cost]
            if 'PREPARE_LANE_CHANGE_LEFT' in next_states:
                if self.prepare_lane_change_counter <= 2 and \
                        not is_target_lane_safe:
                    # if prepare counter small, still in favor of wait
                    cost = 3
                elif is_lane_change_ahead and \
                        self.prepare_lane_change_counter > 2 and \
                        not is_target_lane_safe:
                    # give up lane change
                    self.give_up_lane_change = True
                    cost = 15
                    # reset counter
                    self.prepare_lane_change_counter = 0
                else:
                    # proceed to lane change if not blocked
                    cost = 15
                    # reset counter
                    self.prepare_lane_change_counter = 0
                path['PREPARE_LANE_CHANGE_LEFT'] = [rx, ry, rk, ryaw, cost]
        return path

    # state transition #2 PREPARE_LANE_CHANGE_RIGHT
    def state_transition_prepare_right_lane_change(self, next_states, is_lane_change_ahead,
                                                   lane_change_allowed, is_target_lane_safe):
        path = {}
        # increase counter
        self.prepare_lane_change_counter += 1
        # check for obstacle on target lane (loop through all near-by actors)
        is_blocked = lane_change_allowed # todo: determine by original logic in OpenCDA behavior agent
        # lane following
        if self.current_superstate.name == 'LANE_FOLLOWING':
            # generate path
            rx, ry, rk, ryaw = self._local_planner.generate_path()
            if 'GO_STRAIGHT' in next_states:
                if is_blocked:
                    cost = 5
                else:
                    cost = 15
                path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'LANE_CHANGE_RIGHT' in next_states:
                if not is_blocked:
                    # start lane change, mark starting point
                    self._local_planner.mark_lane_change_start()
                    cost = 5
                else:
                    cost = 15
                path['LANE_CHANGE_RIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'PREPARE_LANE_CHANGE_RIGHT' in next_states:
                if is_lane_change_ahead and \
                        self.prepare_lane_change_counter <= 2 and \
                        is_blocked:
                    # if prepare counter small, still in favor of wait
                    cost = 3
                elif is_lane_change_ahead and \
                        self.prepare_lane_change_counter > 2 and \
                        is_blocked:
                    # give up lane change
                    self.give_up_lane_change = True
                    cost = 15
                    # reset counter
                    self.prepare_lane_change_counter = 0
                else:
                    # proceed to lane change if not blocked
                    cost = 15
                    # reset counter
                    self.prepare_lane_change_counter = 0
                path['PREPARE_LANE_CHANGE_RIGHT'] = [rx, ry, rk, ryaw, cost]
        # overtaking
        elif self.current_superstate.name == 'OVERTAKING':
            rx, ry, rk, ryaw = self._local_planner.generate_path()
            if 'GO_STRAIGHT' in next_states:
                if not is_target_lane_safe:
                    cost = 5
                else:
                    cost = 15
                path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'LANE_CHANGE_RIGHT' in next_states:
                if is_target_lane_safe:
                    # start lane change, mark starting point
                    self._local_planner.mark_lane_change_start()
                    cost = 5
                else:
                    cost = 15
                path['LANE_CHANGE_RIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'PREPARE_LANE_CHANGE_RIGHT' in next_states:
                if self.prepare_lane_change_counter <= 2 and \
                        not is_target_lane_safe:
                    # if prepare counter small, still in favor of wait
                    cost = 3
                elif is_lane_change_ahead and \
                        self.prepare_lane_change_counter > 2 and \
                        not is_target_lane_safe:
                    # give up lane change
                    self.give_up_lane_change = True
                    cost = 15
                    # reset counter
                    self.prepare_lane_change_counter = 0
                else:
                    # proceed to lane change if not blocked
                    cost = 15
                    # reset counter
                    self.prepare_lane_change_counter = 0
                path['PREPARE_LANE_CHANGE_RIGHT'] = [rx, ry, rk, ryaw, cost]
        return path

    # state transition #3. LANE_CHANGE_LEFT
    def state_transition_lane_change_left(self, next_states, is_lane_change_finished):
        path = {}
        rx, ry, rk, ryaw = self._local_planner.generate_path()
        if self.current_superstate.name == 'LANE_FOLLOWING':
            if 'GO_STRAIGHT' in next_states:
                if is_lane_change_finished:
                    cost = 5
                else:
                    cost = 15
                path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'LANE_CHANGE_LEFT' in next_states:
                if not is_lane_change_finished:
                    cost = 5
                    # reset lane change starting point
                    self._local_planner.reset_lane_change_marker()
                    # reset give up lane change marker
                    self.give_up_lane_change = False
                else:
                    cost = 15
                path['LANE_CHANGE_LEFT'] = [rx, ry, rk, ryaw, cost]
        elif self.current_superstate.name == 'OVERTAKING':
            if 'GO_STRAIGHT' in next_states:
                if is_lane_change_finished:
                    cost = 5
                else:
                    cost = 15
                path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'LANE_CHANGE_LEFT' in next_states:
                if not is_lane_change_finished:
                    cost = 5
                    # reset lane change starting point
                    self._local_planner.reset_lane_change_marker()
                    # reset give up lane change marker
                    self.give_up_lane_change = False
                else:
                    cost = 15
                path['LANE_CHANGE_LEFT'] = [rx, ry, rk, ryaw, cost]

        return path

    # state transition #4. LANE_CHANGE_RIGHT
    def state_transition_lane_change_right(self, next_states, is_lane_change_finished):
        path = {}
        rx, ry, rk, ryaw = self._local_planner.generate_path()
        if self.current_superstate.name == 'LANE_FOLLOWING':
            if 'GO_STRAIGHT' in next_states:
                if is_lane_change_finished:
                    cost = 5
                    # reset lane change starting point
                    self._local_planner.reset_lane_change_marker()
                else:
                    cost = 15
                path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'LANE_CHANGE_RIGHT' in next_states:
                if not is_lane_change_finished:
                    cost = 5
                else:
                    cost = 15
                path['LANE_CHANGE_RIGHT'] = [rx, ry, rk, ryaw, cost]
        elif self.current_superstate.name == 'OVERTAKING':
            if 'GO_STRAIGHT' in next_states:
                if is_lane_change_finished:
                    cost = 5
                    # reset lane change starting point
                    self._local_planner.reset_lane_change_marker()
                else:
                    cost = 15
                path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
            if 'LANE_CHANGE_RIGHT' in next_states:
                if not is_lane_change_finished:
                    cost = 5
                else:
                    cost = 15
                path['LANE_CHANGE_RIGHT'] = [rx, ry, rk, ryaw, cost]
        return path

    # state transition #5 car following
    def state_transition_car_followng(self, is_hazard):
        path = {}
        # generate path
        rx, ry, rk, ryaw = self._local_planner.generate_path()
        if self.current_superstate.name == 'LANE_FOLLOWING':
            if is_hazard:
                cost = 5
            else:
                cost = 30
            path['CAR_FOLLOWING'] = [rx, ry, rk, ryaw, cost]

        elif self.current_superstate.name == 'INTERSECTION':
            if is_hazard:
                cost = 5
            else:
                cost = 30
            path['CAR_FOLLOWING'] = [rx, ry, rk, ryaw, cost]
        return path

    # todo: add determination for left and right turn

    # state transition #8. STOP
    def state_transition_stop(self, next_states,
                              is_red_light=None,
                              is_hazard=None):
        # stop do not need trajectory, just send target speed as zero
        path = {}
        # generate path
        rx, ry, rk, ryaw = self._local_planner.generate_path()
        # STOP can only show up at intersection superstate, no need to check superstate
        is_left_turn_ahead, is_right_turn_ahead = self._local_planner.is_turn_ahead()
        if 'STOP' in next_states:
            if is_red_light:
                cost = 3
            else:
                cost = 30
            path['STOP'] = [rx, ry, rk, ryaw, cost]
        if 'TURN_LEFT' in next_states:
            if not is_red_light:
                cost = 5 if is_left_turn_ahead else 30
            else:
                cost = 30
            path['TURN_LEFT'] = [rx, ry, rk, ryaw, cost]
        if 'TURN_RIGHT' in next_states:
            if not is_red_light:
                cost = 5 if is_right_turn_ahead else 30
            else:
                cost = 30
            path['TURN_RIGHT'] = [rx, ry, rk, ryaw, cost]
        if 'CAR_FOLLOWING' in next_states:
            if not is_red_light:
                cost = 5 if is_hazard else 30
            else:
                cost = 30
            path['CAR_FOLLOWING'] = [rx, ry, rk, ryaw, cost]
        if 'GO_STRAIGHT' in next_states:
            if not is_red_light:
                cost = 5 if not (is_left_turn_ahead and is_right_turn_ahead) else 30
            else:
                cost = 30
            path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
        return path

    def generate_trajectory_no_superstate_transition(self, next_states, is_intersection,
                                                     is_hazard, is_red_light, lane_change_allowed,
                                                     overtake_left, overtake_right, is_target_lane_safe):
        path = {}
        # Transitions
        # Lane following
        if self.current_superstate.name == 'LANE_FOLLOWING':
            # check if there's planned lane change ahead
            is_lane_change_ahead = self._local_planner.is_lane_change_ahead()
            # check if lane change finished
            is_lane_change_finished = self._local_planner.is_lane_change_finished()
            # start in go-straight
            if self.current_state.name == 'GO_STRAIGHT':
                path = self.state_transition_go_straight(next_states,
                                                         is_lane_change_ahead=is_lane_change_ahead)
            # start in lane following
            elif self.current_state.name == 'CAR_FOLLOWING':
                path = self.state_transition_car_followng(is_hazard)
            # start in prepare left
            elif self.current_state.name == 'PREPARE_LANE_CHANGE_LEFT':
                path = self.state_transition_prepare_left_lane_change(next_states,
                                                                      is_lane_change_ahead,
                                                                      lane_change_allowed,
                                                                      is_target_lane_safe)
            # start in lane change left
            elif self.current_state.name == 'LANE_CHANGE_LEFT':
                path = self.state_transition_lane_change_left(next_states,
                                                              is_lane_change_finished)
            # start in prepare right
            elif self.current_state.name == 'PREPARE_LANE_CHANGE_RIGHT':
                path = self.state_transition_prepare_right_lane_change(next_states,
                                                                       is_lane_change_ahead,
                                                                       lane_change_allowed,
                                                                       is_target_lane_safe)
            # start in lane change left
            elif self.current_state.name == 'LANE_CHANGE_RIGHT':
                path = self.state_transition_lane_change_right(next_states,
                                                               is_lane_change_finished)
        # Intersection
        elif self.current_superstate.name == 'INTERSECTION':
            if 'GO_STRAIGHT' in next_states:
                path = self.state_transition_go_straight(next_states,
                                                         is_red_light=is_red_light,
                                                         is_hazard=is_hazard)
            if 'CAR_FOLLOWING' in next_states:
                path = self.state_transition_car_followng(is_hazard)
            if 'STOP' in next_states:
                path = self.state_transition_stop(next_states,
                                                  is_red_light=is_red_light,
                                                  is_hazard=is_hazard)
            if 'TURN_LEFT' in next_states:
                rx, ry, rk, ryaw = self._local_planner.generate_path()
                cost = 15
                path['TURN_LEFT'] = [rx, ry, rk, ryaw, cost]
            if 'TURN_RIGHT' in next_states:
                rx, ry, rk, ryaw = self._local_planner.generate_path()
                cost = 15
                path['TURN_RIGHT'] = [rx, ry, rk, ryaw, cost]
        # Overtaking
        elif self.current_superstate.name == 'OVERTAKING':
            # check if there's planned lane change ahead
            is_lane_change_ahead = self._local_planner.is_lane_change_ahead()
            # check if lane change finished
            is_lane_change_finished = self._local_planner.is_lane_change_finished()

            # start in go-straight
            if self.current_state.name == 'GO_STRAIGHT':
                path = self.state_transition_go_straight(next_states,
                                                         is_lane_change_ahead=is_lane_change_ahead,
                                                         overtake_left=overtake_left,
                                                         overtake_right=overtake_right,
                                                         )
            # start in prepare left
            elif self.current_state.name == 'PREPARE_LANE_CHANGE_LEFT':
                path = self.state_transition_prepare_left_lane_change(next_states,
                                                                      is_lane_change_ahead,
                                                                      lane_change_allowed,
                                                                      is_target_lane_safe)
            # start in lane change left
            elif self.current_state.name == 'LANE_CHANGE_LEFT':
                path = self.state_transition_lane_change_left(next_states,
                                                              is_lane_change_finished)
            # start in prepare right
            elif self.current_state.name == 'PREPARE_LANE_CHANGE_RIGHT':
                path = self.state_transition_prepare_right_lane_change(next_states,
                                                                       is_lane_change_ahead,
                                                                       lane_change_allowed,
                                                                       is_target_lane_safe)
            # start in lane change left
            elif self.current_state.name == 'LANE_CHANGE_RIGHT':
                path = self.state_transition_lane_change_right(next_states,
                                                               is_lane_change_finished)

        # rank dict
        sorted_path = dict(sorted(path.items(), key=lambda item: item[1]))
        return sorted_path

    def generate_trajectory_with_superstate_transition(self, next_states, is_intersection,
                                                       is_hazard, is_red_light,lane_change_allowed,
                                                       is_target_lane_safe):
        path = {}
        rx, ry, rk, ryaw = self._local_planner.generate_path()
        cost = 5
        path['GO_STRAIGHT'] = [rx, ry, rk, ryaw, cost]
        return path

    def generate_trajectory(self, next_superstate,
                            next_states, is_intersection,
                            is_hazard, is_red_light, lane_change_allowed,
                            overtake_left, overtake_right, is_target_lane_safe):
        if next_superstate == self.current_superstate.name:
            path = self.generate_trajectory_no_superstate_transition(next_states,
                                                                     is_intersection,
                                                                     is_hazard,
                                                                     is_red_light,
                                                                     lane_change_allowed,
                                                                     overtake_left,
                                                                     overtake_right,
                                                                     is_target_lane_safe)
        else:
            path = self.generate_trajectory_with_superstate_transition(next_states,
                                                                       is_intersection,
                                                                       is_hazard,
                                                                       is_red_light,
                                                                       lane_change_allowed,
                                                                       is_target_lane_safe)

        return path

    def transition(self, superstate_name=None, state_name=None):
        if superstate_name:
            self.current_superstate = self.superstates[superstate_name]
            # print(f"Transitioned to superstate {self.current_superstate}")
        if state_name:
            self.current_state = self.states[state_name]

    def get_current_prompt(self):
        '''
        Generate prompt for Vision-language model.
        '''
        if self.current_state == self.states["GO_STRAIGHT"]:
            prompt = "Based on current traffic condition including traffic light, \
                    generate future driving plan in one short sentence.\
                    If there's no traffic light in pucture, just say it's not detected."

        elif self.current_state == self.states["PREPARE_LANE_CHANGE_LEFT"]:
            prompt = "Based on current lane position and traffic condition, generate future\
                 driving plan for a potential left lane change in one short sentence."

        elif self.current_state == self.states["PREPARE_LANE_CHANGE_RIGHT"]:
            prompt = "Based on current lane position and traffic condition, generate future\
                 driving plan for a potential right lane change in one short sentence."

        elif self.current_state == self.states["LANE_CHANGE_LEFT"]:
            prompt = "Based on current lane position and traffic condition, generate future\
                 driving plan for a lane change to the left adjacent lane in one short sentence."

        elif self.current_state == self.states["LANE_CHANGE_RIGHT"]:
            prompt = "Based on current lane position and traffic condition, generate future\
                 driving plan for a lane change to the right adjacent lane in one short sentence."

        elif self.current_state == self.states["CAR_FOLLOWING"]:
            prompt = "Based on current lane position and traffic condition, generate future\
                 driving plan for a car following behavior in one short sentence."

        elif self.current_state == self.states["TURN_LEFT"]:
            prompt = "Based on current lane position and traffic light, generate future\
                 driving plan for a left turn in one short sentence."

        elif self.current_state == self.states["TURN_RIGHT"]:
            prompt = "Based on current lane position and traffic light, generate future\
                 driving plan for a right turn in one short sentence."

        elif self.current_state == self.states["STOP"]:
            prompt = "Based on current lane position, traffic light and traffic condition, generate future\
                 driving plan to stop the ego vehicle in one short sentence."

        else:
            print('Warning: FSM state error. Current state not exist.')
            prompt = "Based on current traffic condition including traffic light, \
                    generate future driving plan in one short sentence.\
                    If there's no traffic light in pucture, just say it's not detected."

        return prompt

