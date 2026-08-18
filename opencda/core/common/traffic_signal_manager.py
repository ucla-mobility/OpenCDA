# -*- coding: utf-8 -*-

"""
Traffic signal manager for intersection control.
"""
# Author: Xu Han <hanxu417@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla


class TrafficSignalManager(object):
    """
    A class manager to control the traffic signals of an intersection.

    The manager controls four CARLA traffic light actors corresponding
    to the northbound, eastbound, southbound, and westbound approaches.
    It supports both normal fixed-cycle control and adaptive traffic
    signal control.

    Parameters
    ----------
    world : carla.World
        The CARLA simulation world.

    northbound_id : int
        CARLA actor ID of the northbound traffic light.

    eastbound_id : int
        CARLA actor ID of the eastbound traffic light.

    southbound_id : int
        CARLA actor ID of the southbound traffic light.

    westbound_id : int
        CARLA actor ID of the westbound traffic light.

    adaptive_cycle : bool
        Indicator of whether adaptive traffic signal control is enabled.

    Attributes
    ----------
    world : carla.World
        The CARLA simulation world.

    traffic_lights : dict
        Dictionary containing the four CARLA traffic light actors.

    adaptive_cycle : bool
        Whether adaptive traffic signal control is enabled.

    current_phase : int
        Current traffic signal phase.

    phase_elapsed_time : float
        Elapsed simulation time of the current phase.
    """

    def __init__(
            self,
            world,
            northbound_id,
            eastbound_id,
            southbound_id,
            westbound_id,
            adaptive_cycle=False):

        self.world = world

        # traffic light actor IDs
        self.northbound_id = northbound_id
        self.eastbound_id = eastbound_id
        self.southbound_id = southbound_id
        self.westbound_id = westbound_id

        # traffic signal control mode
        self.adaptive_cycle = adaptive_cycle

        # retrieve traffic light actors
        self.traffic_lights = {
            'northbound': None,
            'eastbound': None,
            'southbound': None,
            'westbound': None
        }

        self._initialize_traffic_lights()

        # traffic signal phase information
        self.current_phase = None
        self.phase_elapsed_time = 0.0

        # traffic signal phase sequence
        # This will be defined based on the SPAT diagram.
        self.phase_sequence = []

    def _initialize_traffic_lights(self):
        """
        Retrieve the four traffic light actors from CARLA.
        """

        traffic_light_ids = {
            'northbound': self.northbound_id,
            'eastbound': self.eastbound_id,
            'southbound': self.southbound_id,
            'westbound': self.westbound_id
        }

        for direction, actor_id in traffic_light_ids.items():

            actor = self.world.get_actor(actor_id)

            if actor is None:
                raise RuntimeError(
                    'Traffic light actor %d cannot be found.'
                    % actor_id)

            if not isinstance(actor, carla.TrafficLight):
                raise TypeError(
                    'Actor %d is not a CARLA traffic light.'
                    % actor_id)

            self.traffic_lights[direction] = actor

    def update_info(self, dt):
        """
        Update traffic signal information.

        Parameters
        ----------
        dt : float
            Simulation time step in seconds.
        """

        self.phase_elapsed_time += dt

    def run_step(self):
        """
        Execute one step of traffic signal control.
        """

        if self.adaptive_cycle:
            self.run_step_adaptive()
        else:
            self.run_step_normal()

    def run_step_normal(self):
        """
        Placeholder normal traffic signal cycle.

        Northbound and southbound are green.
        Eastbound and westbound are red.
        """

        self._apply_phase(0)

    def run_step_adaptive(self):
        """
        Execute one step of adaptive traffic signal control.

        The adaptive phase transition logic will be implemented
        according to the SPAT diagram.
        """

        pass

    def set_phase(self, phase):
        """
        Set the current traffic signal phase.

        Parameters
        ----------
        phase : int
            The target traffic signal phase.
        """

        self.current_phase = phase
        self.phase_elapsed_time = 0.0

        self._apply_phase(phase)

    def _apply_phase(self, phase):
        """
        Apply the placeholder traffic signal phase.

        Northbound  -> GREEN
        Southbound  -> GREEN
        Eastbound   -> RED
        Westbound   -> RED
        """
        # temp phase for validation
        if phase == 0:
            self.traffic_lights['northbound'].set_state(
                carla.TrafficLightState.Green)

            self.traffic_lights['southbound'].set_state(
                carla.TrafficLightState.Green)

            self.traffic_lights['eastbound'].set_state(
                carla.TrafficLightState.Red)

            self.traffic_lights['westbound'].set_state(
                carla.TrafficLightState.Red)

    def set_traffic_light_state(self, direction, state):
        """
        Set the state of an individual traffic light.

        Parameters
        ----------
        direction : str
            Traffic approach direction. Valid values are
            'northbound', 'eastbound', 'southbound', and 'westbound'.

        state : carla.TrafficLightState
            Desired CARLA traffic light state.
        """

        if direction not in self.traffic_lights:
            raise ValueError(
                'Invalid traffic light direction: %s'
                % direction)

        self.traffic_lights[direction].set_state(state)

    def set_all_red(self):
        """
        Set all traffic lights to red.
        """

        for traffic_light in self.traffic_lights.values():
            traffic_light.set_state(
                carla.TrafficLightState.Red)

    def set_adaptive_cycle(self, enabled):
        """
        Enable or disable adaptive traffic signal control.

        Parameters
        ----------
        enabled : bool
            True to enable adaptive traffic signal control.
        """

        self.adaptive_cycle = enabled

    def reset(self):
        """
        Reset the traffic signal manager.
        """

        self.current_phase = None
        self.phase_elapsed_time = 0.0

        self.set_all_red()

    def destroy(self):
        """
        Clean up the traffic signal manager.

        The CARLA traffic light actors are map actors and therefore
        are not destroyed by this manager.
        """

        self.reset()