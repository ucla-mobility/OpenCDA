# -*- coding: utf-8 -*-

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import importlib


class CavWorld(object):
    """
    A customized world object to save all CDA vehicle
    information and shared ML models. During co-simulation,
    it is also used to save the sumo-carla id mapping.

    Parameters
    ----------
    apply_ml : bool
        Whether apply ml/dl models in this simulation, please make sure
        you have install torch/sklearn before setting this to True.

    Attributes
    ----------
    vehicle_id_set : set
        A set that stores vehicle IDs.

    _vehicle_manager_dict : dict
        A dictionary that stores vehicle managers.

    _platooning_dict : dict
        A dictionary that stores platooning managers.

    _rsu_manager_dict : dict
        A dictionary that stores RSU managers.

    ml_manager : opencda object.
        The machine learning manager class.
    """

    def __init__(self, apply_ml=False):

        self.vehicle_id_set = set()
        self._vehicle_manager_dict = {}
        self._platooning_dict = {}
        self._rsu_manager_dict = {}
        self.ml_manager = None

        if apply_ml:
            # we import in this way so the user don't need to install ml
            # packages unless they require to
            ml_manager = getattr(importlib.import_module(
                "opencda.customize.ml_libs.ml_manager"), 'MLManager')
            # initialize the ml manager to load the DL/ML models into memory
            self.ml_manager = ml_manager()

        # this is used only when co-simulation activated.
        self.sumo2carla_ids = {}

    def update_vehicle_manager(self, vehicle_manager):
        """
        Update created CAV manager to the world.

        Parameters
        ----------
        vehicle_manager : opencda object
            The vehicle manager class.
        """
        self.vehicle_id_set.add(vehicle_manager.vehicle.id)
        self._vehicle_manager_dict.update(
            {vehicle_manager.vid: vehicle_manager})

    def update_platooning(self, platooning_manger):
        """
        Add created platooning.

        Parameters
        ----------
        platooning_manger : opencda object
            The platooning manager class.
        """
        self._platooning_dict.update(
            {platooning_manger.pmid: platooning_manger})

    def update_rsu_manager(self, rsu_manager):
        """
        Add rsu manager.

        Parameters
        ----------
        rsu_manager : opencda object
            The RSU manager class.
        """
        self._rsu_manager_dict.update({rsu_manager.rid: rsu_manager})

    def update_sumo_vehicles(self, sumo2carla_ids):
        """
        Update the sumo carla mapping dict. This is only called
        when cosimulation is conducted.

        Parameters
        ----------
        sumo2carla_ids : dict
            Key is sumo id and value is carla id.
        """
        self.sumo2carla_ids = sumo2carla_ids

    def get_vehicle_managers(self):
        """
        Return vehicle manager dictionary.
        """
        return self._vehicle_manager_dict

    def get_platoon_dict(self):
        """
        Return existing platoons.
        """
        return self._platooning_dict

    def locate_vehicle_manager(self, loc):
        """
        Locate the vehicle manager based on the given location.

        Parameters
        ----------
        loc : carla.Location
            Vehicle location.

        Returns
        -------
        target_vm : opencda object
            The vehicle manager at the give location.
        """

        target_vm = None
        for key, vm in self._vehicle_manager_dict.items():
            x = vm.localizer.get_ego_pos().location.x
            y = vm.localizer.get_ego_pos().location.y

            if loc.x == x and loc.y == y:
                target_vm = vm
                break

        return target_vm
