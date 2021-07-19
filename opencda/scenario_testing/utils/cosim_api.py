# -*- coding: utf-8 -*-
"""
Co-simulation scenario manager. The code is modified from CARLA official
cosimulation code.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import logging
import os
import sys


if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from opencda.scenario_testing.utils.sim_api import ScenarioManager
from opencda.co_simulation.sumo_integration.bridge_helper import BridgeHelper
from opencda.co_simulation.sumo_integration.carla_simulation import \
    CarlaSimulation
from opencda.co_simulation.sumo_integration.constants import INVALID_ACTOR_ID
from opencda.co_simulation.sumo_integration.sumo_simulation import \
    SumoSimulation


class CoScenarioManager(ScenarioManager):
    """
    The Scenario manager for co-simulation(CARLA-SUMO).

    Parameters
    ----------
    scenario_params : dict
        The dictionary contains all simulation configurations.

    xodr_path : str
        The xodr file to the customized map, default: None.

    town : str
        Town name if not using customized map, eg. 'Town06'.

    apply_ml : bool
        Whether need to load dl/ml model(pytorch required) in this simulation.

    """
    def __init__(self, scenario_params, apply_ml,
                 xodr_path=None,
                 town=None,
                 cav_world=None,
                 sumo_file_parent_path=None):
        super(CoScenarioManager,  self).__init__(scenario_params,
                                                 apply_ml,
                                                 xodr_path,
                                                 town,
                                                 cav_world)

        # carla side other initializations
        # these following sets are used to track the vehicles controlled
        # by sumo side
        self._active_actors = set()
        self.spawned_actors = set()
        self.destroyed_actors = set()

        # contains all carla traffic lights objects
        self._tls = {}
        for landmark in self.carla_map.get_all_landmarks_of_type('1000001'):
            if landmark.id != '':
                traffic_ligth = self.world.get_traffic_light(landmark)
                if traffic_ligth is not None:
                    self._tls[landmark.id] = traffic_ligth
                else:
                    logging.warning('Landmark %s is not linked to any '
                                    'traffic light', landmark.id)

        # sumo side initialization
        base_name = os.path.basename(sumo_file_parent_path)
        sumo_cfg = os.path.join(sumo_file_parent_path, base_name+'.net.xml')
        # todo use yaml file to generate the route file
        

