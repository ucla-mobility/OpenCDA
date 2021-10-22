# -*- coding: utf-8 -*-

"""Communication manager for cooperation
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from collections import deque
import weakref

import carla
import numpy as np

from opencda.core.application.platooning.platooning_plugin \
    import PlatooningPlugin
from opencda.core.common.misc import compute_distance


class V2XManager(object):
    """
    V2X Manager for platooning, cooperative perception and so on.

    Parameters
    ----------
    cav_world : opencda object
        CAV world.

    config_yaml : dict
        The configuration dictionary of the v2x module.

    vid : str
        The corresponding vehicle manager's uuid.

    Attributes
    ----------
    _recieved_buffer : dict
        A buffer for receive data.

    cav_nearby : dict
        The dictionary that contains the cavs in the communication range.

    platooning_plugin : opencda object
        The platooning plugin for communication during platooning.

    ego_pos : carla.transform
        Ego position.

    ego_spd : float
        Ego speed(km/h).

    """

    def __init__(self, cav_world, config_yaml, vid):
        # if disabled, no cooperation will be operated
        self.cda_enabled = config_yaml['enabled']
        self.communication_range = config_yaml['communication_range']

        # found CAVs nearby
        self.cav_nearby = {}

        # used for cooperative perception.
        self._recieved_buffer = {}

        # used for platooning communication
        self.platooning_plugin = PlatooningPlugin(
            self.communication_range, self.cda_enabled)

        self.cav_world = weakref.ref(cav_world)()

        # ego position buffer. use deque so we can simulate lagging
        self.ego_pos = deque(maxlen=100)
        self.ego_spd = deque(maxlen=100)
        # used to exclude the cav self during searching
        self.vid = vid

        # check if lag or noise needed to be added during communication
        self.loc_noise = 0.0
        self.yaw_noise = 0.0
        self.speed_noise = 0.0
        self.lag = 0

        # Add noise to V2X communication if needed.
        if 'loc_noise' in config_yaml:
            self.loc_noise = config_yaml['loc_noise']
        if 'yaw_noise' in config_yaml:
            self.yaw_noise = config_yaml['yaw_noise']
        if 'speed_noise' in config_yaml:
            self.speed_noise = config_yaml['speed_noise']
        if 'lag' in config_yaml:
            self.lag = config_yaml['lag']

    def update_info(self, ego_pos, ego_spd):
        """
        Update all communication plugins with current localization info.
        """
        self.ego_pos.append(ego_pos)
        self.ego_spd.append(ego_spd)
        self.search()

        # the ego pos in platooning_plugin is used for self-localization,
        # so we shouldn't add noise or lag.
        self.platooning_plugin.update_info(ego_pos, ego_spd)

    def get_ego_pos(self):
        """
        Add noise and lag to the current ego position and send to other CAVs.
        This is for simulate noise and lagging during communication.

        Returns
        -------
        processed_ego_pos : carla.Transform
            The ego position after adding noise and lagging.
        """
        if not self.ego_pos:
            return None

        # add lag
        ego_pos = self.ego_pos[0] if len(self.ego_pos) < self.lag else \
            self.ego_pos[np.random.randint(-1 - int(abs(self.lag)), 0)]

        x_noise = np.random.normal(0, self.loc_noise) + ego_pos.location.x
        y_noise = np.random.normal(0, self.loc_noise) + ego_pos.location.y
        z = ego_pos.location.z
        yaw_noise = np.random.normal(0, self.yaw_noise) + ego_pos.rotation.yaw

        noise_location = carla.Location(x=x_noise, y=y_noise, z=z)
        noise_rotation = carla.Rotation(pitch=0, yaw=yaw_noise, roll=0)

        processed_ego_pos = carla.Transform(noise_location, noise_rotation)

        return processed_ego_pos

    def get_ego_speed(self):
        """
        Add noise and lag to the current ego speed.

        Returns
        -------
        processed_ego_speed : float
            The ego speed after adding noise and lagging.
        """
        if not self.ego_spd:
            return None
        # add lag
        ego_speed = self.ego_spd[0] if len(self.ego_spd) < self.lag else \
            self.ego_spd[-1 - int(abs(self.lag))]
        processed_ego_speed = np.random.normal(0, self.speed_noise) + ego_speed

        return processed_ego_speed

    def search(self):
        """
        Search the CAVs nearby.
        """
        vehicle_manager_dict = self.cav_world.get_vehicle_managers()

        for vid, vm in vehicle_manager_dict.items():
            # avoid the Nonetype error at the first simulation step
            if not vm.v2x_manager.get_ego_pos():
                continue
            # avoid add itself as the cav nearby
            if vid == self.vid:
                continue
            distance = compute_distance(
                self.ego_pos[-1].location,
                vm.v2x_manager.get_ego_pos().location)

            if distance < self.communication_range:
                self.cav_nearby.update({vid: vm})
    """
    -----------------------------------------------------------
                 Below is platooning related 
    -----------------------------------------------------------
    """

    def set_platoon(
            self,
            in_id,
            platooning_object=None,
            platooning_id=None,
            leader=False):
        """
        Set platooning status

        Parameters
        ----------
        platooning_object : platoon object)
            Platooning world that contains all platoon information.

        platooning_id : int
            Platoon id the cav belongs to.

        in_id : int
            The position in the platoon, etc. 0 represents leader
            and 1 represents the second position.

        leader : boolean
            Indicate whether this cav is a leader in platoon.

        """
        self.platooning_plugin.set_platoon(
            in_id, platooning_object, platooning_id, leader)

    def set_platoon_status(self, status):
        """
        Set the cav to a different fsm status.

        Parameters
        ----------
        status : str
            fsm status.

        """
        self.platooning_plugin.set_status(status)

    def set_platoon_front(self, vm):
        """
        Set the frontal vehicle to another vehicle

        Parameters
        ----------
        vm : opencda object
            The target vehicle manager.
        """
        self.platooning_plugin.front_vehicle = vm

    def set_platoon_rear(self, vm):
        """
        Set the rear vehicle to another vehicle

        Parameters
        __________
        vm : opencda object
            The target vehicle manager.
        """
        self.platooning_plugin.rear_vechile = vm

    def add_platoon_blacklist(self, pmid):
        """
        Add an existing platoon to current blacklist.

        Parameters
        ----------
        pmid : int
            The target platoon manager ID.
        """
        self.platooning_plugin.platooning_blacklist.append(pmid)

    def match_platoon(self):
        """
        A naive way to find the best position to join a platoon.
        """
        return self.platooning_plugin.match_platoon(self.cav_nearby)

    def in_platoon(self):
        """
        Check whether the vehicle is inside the platoon.

        Returns
        -------
        flag : bool
            Whether this vehicle is inside a platoon.
        """
        return False if self.platooning_plugin.in_id is None else True

    def get_platoon_manager(self):
        """
        Retrieve the platoon manager the cav
        belongs to and the corresponding id.

        Returns
        -------
        platoon_object : opencda object
            The PlatoonManager object.

        in_id : int
            The ego vehicle's in team id.
        """
        return self.platooning_plugin.platooning_object, \
               self.platooning_plugin.in_id

    def get_platoon_status(self):
        """
        Retrieve the FSM status for platooning application

        Returns
        -------
        status : enum
            The vehicle's current platoon status.
        """
        return self.platooning_plugin.status

    def get_platoon_front_rear(self):
        """
        Get the ego vehicle's front and rear cav in the platoon

        Returns
        -------
        front_vehicle : opencda object
            Front vehicle of the ego vehicle in the platoon.

        rear_vehicle : opencda object
            Rear vehicle of the ego vehicle in the platoon.
        """
        return self.platooning_plugin.front_vehicle, \
               self.platooning_plugin.rear_vechile
