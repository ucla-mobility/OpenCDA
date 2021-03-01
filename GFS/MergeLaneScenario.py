# -*- coding: utf-8 -*-
"""
Created on Tue Dec 1, 2020

@creator: Anoop Sathyan
@authors: Yi Guo, Anoop Sathyan, Jiaqi Ma
"""


import os
import sys
import traci
import pdb

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from src.simlib import setUpSimulation
from src.simulationmanager import SimulationManager


setUpSimulation(r'.\MergeLaneRamp\MergeLaneProbIDMM.sumocfg')
step = 0

manager = SimulationManager(pCreation=True, iCoordination=False, iZipping=False)

time = 0
while time < 30*60:
    # if abs(time-100.0) < 1e-3: #Insert vehicles at t = 100s
    #     manager.add_vehicles(4)
    manager.handleSimulationStep(time)
    # traci.vehicle.setSpeed("flow_0.2", 35)
    # traci.vehicle.setSpeedMode("flow_0.2", 0)
    time += 0.5
    traci.simulationStep(time)    

traci.close()
