# -*- coding: utf-8 -*-

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import importlib


class CavWorld(object):
    """
    A customized world object to save all CDA vehicle information and shared ML models.

    Parameters
    -apply_ml : boolean
        whether apply ml/dl models in this simulation, please make sure 
        you have install torch/sklearn before setting this to True.
    
    Attributes
    -vehicle_id_set : set
        A set that stores vehicle IDs.
    -_vehicle_manager_dict : dict
        A dictionary that stores vehicle managers.
    -_platooning_dict : dict
        A dictionary that stores platooning managers. 
    -ml_manager : opencda object.
        The machine learning manager class.
    """

    def __init__(self, apply_ml=False):
        
        self.vehicle_id_set = set()
        self._vehicle_manager_dict = {}
        self._platooning_dict = {}
        self.ml_manager = None

        if apply_ml:
            # we import in this way so the user don't need to install ml packages unless they require to
            ml_manager = getattr(importlib.import_module("opencda.customize.ml_libs.ml_manager"), 'MLManager')
            # initialize the ml manager to load the DL/ML models into memory
            self.ml_manager = ml_manager()

    def update_vehicle_manager(self, vehicle_manager):
        """
        Update created CAV manager to the world.

        Args
            -vehicle_manager (opencda object): The vehicle manager class. 
        """
        self.vehicle_id_set.add(vehicle_manager.vehicle.id)
        self._vehicle_manager_dict.update({vehicle_manager.vid: vehicle_manager})

    def update_platooning(self, platooning_manger):
        """
        Add created platooning.

        Args
            -platooning_manger (opencda object): The platooning manager class.
        """
        self._platooning_dict.update({platooning_manger.pmid: platooning_manger})

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

        Args
            -loc (carla.Location): vehicle location.

        Returns
            -target_vm (vehicle_manager): The vehicle manager at the give location.
        """

        target_vm = None
        for key, vm in self._vehicle_manager_dict.items():
            x = vm.localizer.get_ego_pos().location.x
            y = vm.localizer.get_ego_pos().location.y

            if loc.x == x and loc.y == y:
                target_vm = vm
                break

        return target_vm
