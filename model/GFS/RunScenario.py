# -*- coding: utf-8 -*-
"""
Created on Mon Jun 22 15:10:25 2020

@author: Anoop
"""

import sys
import traci
import numpy as np
import pdb
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from src.simulationmanager import SimulationManager
from src.simlib import setUpSimulation

def RunScenarioSUMO(gfs_pl_score, gfs_pl_speed, gfs_m, validation):

    manager = SimulationManager(pCreation=True, iCoordination=False, iZipping=False)
    time = 0
    avg = []
    
    setUpSimulation(r'MergeLaneRamp/MergeLaneProbIDMM.sumocfg', 1.0, validation)
    while time < 5*60:
        
        time += 0.25
            
        traci.simulationStep(time)
        avg_speed_timestep = manager.handleSimulationStepFIS(time, gfs_pl_score, gfs_pl_speed, gfs_m)
        
        if time > 1.0: #To avoid nans in the first step
            avg.append(avg_speed_timestep) 
        
        
    traci.close()  
    if validation == 0:
        return np.mean(avg)
    elif validation == 1:
        return np.mean(avg)
    
