# -*- coding: utf-8 -*-
"""
Created on Tue Dec 1, 2020

@creator: Anoop Sathyan
@authors: Yi Guo, Anoop Sathyan, Jiaqi Ma
"""

from src.intersectionController import IntersectionController # need to update for CARLA 
from src.platoon import Platoon # need to update to CARLA
from src.vehicle import Vehicle
from src.simlib import flatten

import traci
import pdb
import numpy as np
import src.utils as ut

# vehicle length
LENGTH = 5
# inter-vehicle distance
DISTANCE = 30
# inter-vehicle distance when leaving space for joining
JOIN_DISTANCE = DISTANCE * 2
# cruising speed
SPEED = 31.29 #130 / 3.6

# Speed Mode
SPD_ALL_CHECK = 31
SPD_DIS_SAFE = 30

# Time step
dt = 0.25

# maximum platoon length
Max_Platoon_Length = 10

# inter-platoon time gap
INTER_TAU = 1.5
INTRA_TAU = 0.6
DISSOLVE_GAP = 0.8

# vehicle type
HDV_TYPE = 'DEFAULT_VEHTYPE'
CDA_TYPE = 'vType_0'

class SimulationManager():

    def __init__(self, pCreation=True, iCoordination=True, iZipping=True):
        self.intersections = []
        self.platoons = list()
        self.platoonCreation = pCreation
        self.vehicles = dict()
        self.mergelist = dict()
        self.dissolvelist = dict()
        self.maxStoppedVehicles = dict()
        self.dissolvetest = 0
        self.relevantPlatoonHist = [] # For data collection
        self.relevantMergeList = [] # For data collection for merge position
        if iCoordination:
            for intersection in traci.trafficlights.getIDList():
                controller = IntersectionController(intersection, iZipping)
                self.intersections.append(controller)

    def createPlatoon(self, vehicles):
        # Creates a platoon with the given vehicles
        platoon = Platoon(vehicles)
        self.platoons.append(platoon)

    def getActivePlatoons(self):
        # Gets all active platoons
        return [p for p in self.platoons if p.isActive()]

    def getAllVehiclesInPlatoons(self):
        # Gets all vehicles in every active platoon
        return flatten(p.getAllVehiclesByName() for p in self.getActivePlatoons())

    def getAverageLengthOfAllPlatoons(self):
        count = 0
        length = len(self.platoons)
        for platoon in self.platoons:
            if platoon._disbandReason != "Merged" and platoon._disbandReason != "Reform required due to new leader":
                count = count + platoon.getNumberOfVehicles()
            else:
                length = length - 1
        return count/length

    def getPlatoonByLane(self, lane):
        # Gets platoons corresponding to a given lane
        return [p for p in self.getActivePlatoons() if lane == p.getLane()]

    def getPlatoonByVehicle(self, v):
        for p in self.getActivePlatoons():
            if v.getName() in p.getAllVehiclesByName():
                return [p]
            #p for p in self.getActivePlatoons() if v in p.getAllVehiclesByName()
        return None
           
    def getReleventPlatoon(self, vehicle):
        # Returns a single platoon that is most relevent to the given
        # vehicle by looking to see if the car in front  is part of a platoon
        # It also checks that the platoon is heading in the right direction
        leadVeh = vehicle.getLeader(50)
        leftLeadVeh = vehicle.getLeftLeader()
        rightLeadVeh = vehicle.getRightLeader()
        leftFollowVeh = vehicle.getLeftFollower()
        rightFollowVeh = vehicle.getRightFollower()
        
        ''' Do not delete the below four lines. It ensures merge lane vehicle only tries to join left vehicles on mainline'''
        cur_lane = vehicle.getLane()
        if vehicle.getPosition()[1] < -5:
            leadVeh = None
            rightLeadVeh = None
            rightFollowVeh = None
        
        if cur_lane == 'gneE4_0':
            ''' preference: left follower better than left leader'''
            # potential left rear platoon
            if leftFollowVeh and leftFollowVeh[1] < 50:  # and direction == 0
        
                leadIns = self.vehicles[leftFollowVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    y_Leader = possiblePlatoon[0]._vehicles[0].getPosition()[1]
                    y_leftFollowVeh = leadIns.getPosition()[1]
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length and abs(y_leftFollowVeh-y_Leader) < 1e-3:  #Second condition is to check if the closest vehicle in the same lane as the platoon leader   # Dec 30, 2020 --> Max_Platoon_Length
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if 'Leading' in possiblePlatoon[0].getLeadVehicle().getState() or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                    possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], -1, leftFollowVeh
            
            # potential left preceding platoon
            if leftLeadVeh and leftLeadVeh[1] < 50: # and direction == 0
                leadIns = self.vehicles[leftLeadVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    y_Leader = possiblePlatoon[0]._vehicles[0].getPosition()[1]
                    y_leftLeadVeh = leadIns.getPosition()[1]
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length and abs(y_leftLeadVeh-y_Leader) < 1e-3:  #Second condition is to check if the closest vehicle in the same lane as the platoon leader   # Dec 30, 2020 --> Max_Platoon_Length
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if 'Leading' in possiblePlatoon[0].getLeadVehicle().getState() or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], 1, leftLeadVeh
        else:            
            if leadVeh and leadVeh[1] < 50: # and direction == 0
                leadIns = self.vehicles[leadVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:     # Dec 30, 2020 --> Max_Platoon_Length
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if 'Leading' in possiblePlatoon[0].getLeadVehicle().getState() or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], 0, leadVeh
            # potential left preceding platoon
            if leftLeadVeh and leftLeadVeh[1] < 50: # and direction == 0
                leadIns = self.vehicles[leftLeadVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:     # Dec 30, 2020 --> Max_Platoon_Length
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if 'Leading' in possiblePlatoon[0].getLeadVehicle().getState() or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], 1, leftLeadVeh
    
            # potential right preceding platoon
            if rightLeadVeh and rightLeadVeh[1] < 50:  # and direction == 0
                leadIns = self.vehicles[rightLeadVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:  # Dec 30, 2020 --> Max_Platoon_Length
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if 'Leading' in possiblePlatoon[0].getLeadVehicle().getState() or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                    possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], 1, rightLeadVeh
    
            # potential left rear platoon
            if leftFollowVeh and leftFollowVeh[1] < 50:  # and direction == 0
        
                leadIns = self.vehicles[leftFollowVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:  # Dec 30, 2020 --> Max_Platoon_Length
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if 'Leading' in possiblePlatoon[0].getLeadVehicle().getState() or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                    possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], -1, leftFollowVeh
    
            # potential left rear platoon
            if rightFollowVeh and rightFollowVeh[1] < 50:  # and direction == 0
                leadIns = self.vehicles[rightFollowVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    if len(possiblePlatoon[
                               0]._vehicles) < Max_Platoon_Length:  # Dec 30, 2020 --> Max_Platoon_Length
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if 'Leading' in possiblePlatoon[0].getLeadVehicle().getState() or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                    possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], -1, rightFollowVeh
    # added the default return value
        return None, -100, leadVeh
                    
    def handleSimulationStepFIS(self, fisMergeLoc, gfs_pl_speed, gfs_m):
        allVehicles = traci.vehicle.getIDList()
        # Check mark vehicles as in-active if they are outside the map

        allSpeeds = []
        for v in list(self.vehicles):
            if v not in allVehicles:  # .getName()
                v_ins = self.getAllVehicles().pop(v)
                if v_ins is not None:
                    v_ins.setInActive()
            # check if the ego vehicle request to dissolve
            # the dissolve super-state may be further initiated by several conditions
            # since in this version we only have one route for mainline traffic
            # here just set certain vehicle(s) to dissolve for illustration purpose.
            else:
                vVeh = self.getVehicleByID(v)
                vVeh_speed = vVeh.getSpeed()
#                if vVeh_speed < 1:           
#                    return -10
                allSpeeds.append(vVeh_speed)
                if v == 'flow_0.10' and self.dissolvetest == 0:
                    v_ins = self.getVehicleByID(v)
                    if v_ins.getPosition()[0] > -1280:
                        v_ins.setState("RequestDissolve")
                        platoon = self.getPlatoonByVehicle(v_ins)
                        plt_leader = platoon[0].getLeadVehicle()
                        if plt_leader.getName() == v:
                            lead = -1
                            # v.setState("Dissolving")
                        else:
                            lead = plt_leader
                            lead.setState("PreParingForDissolve")
                        self.dissolvelist[v] = lead

                        self.dissolvetest = 1
        
        # Update all platoons active status
        for p in self.getActivePlatoons():
            # if a platoon member is inactive
            # all member set to inactive (not platooning) and re-seeking platooning
            # --> stopBehavior: set to default value
            #   v.setColor((255, 255, 255))
            #   # v.setImperfection(0.5) # perfect CAV behavior
            #   v.setMinGap(2.5)
            #   v.setTau(1)
            #   # Take speed back to default behaviour
            #   v.setSpeed(-1)
            p.updateIsActive()

        # The default value of platoonCreation is True
        # Organize platoon within this block
        if self.platoonCreation:
            # See whether there are any vehicles that are not in a platoon with more than one vehicle

            # vehiclesNotInPlatoons = [v for v in allVehicles if v not in self.getAllVehiclesInPlatoons()]
            # if v not in self.getAllVehicles() --> vehicle just entered the network
            #       or v is "SearchingForPlatooning"
            vehiclesNotInPlatoons = [v for v in allVehicles if v not in self.getAllVehicles() or
                                     (v in self.getAllVehicles() and self.getAllVehicles()[v].getState() ==
                                      "SearchingForPlatooning")]

            for vehicleID in vehiclesNotInPlatoons:
                # consider HDV for mixed-traffic
                # each HDV should appear in this loop only once
                
                # if self.getVehicleType(vehicleID) == HDV_TYPE:
                #     vehicle = Vehicle(vehicleID)
                #     vehicle.setState("CDADisabled")
                #     vehicle.setColor((255,255,255))
                #     self.addNewVehicle(vehicleID, vehicle)
                #     continue
                # default state: SearchingForPlatooning
                # what will happen if a vehicle has already been initialized? --> duplicated object
                # list() --> dict()
                # if just entered, initialize and add it to the dict
                
                if vehicleID not in self.getAllVehicles():
                    vehicle = Vehicle(vehicleID)
                    self.addNewVehicle(vehicleID, vehicle)
                    # note that vehicle is appended here
                    # self.vehicles.append(vehicle)
                
                # get vehicle instance from dictionary
                vehicle = self.getVehicleByID(vehicleID)
                
                # If we're not in a starting segment (speed starts as 0)
                
                if not vehicle.isActive():
                    continue
                
                possiblePlatoon, direction, leadVeh = self.getReleventPlatoon(vehicle)
               
                if vehicle.getLane() == 'gneE4_1' and direction == -1:
                    possiblePlatoon = None
                
                if possiblePlatoon: #and direction == 0
                    
                    curPlatoon = self.getPlatoonByVehicle(vehicle)
                    curX, curY = vehicle.getPosition()
                    
                    # same lane join
                    if direction == 0 and vehicle.getLane() != 'gneE4_0':
                        if len(possiblePlatoon._vehicles) == 1:
                            possiblePlatoon.getLeadVehicle().setState("Leading")  # 'Platooning'
                        # --> setStartBehavior
                        # v.setColor(self._color)
                        # v.setImperfection(0)
                        # v.setMinGap(0)
                        # v.setTau(0.6)
                        # v._state = 'Platooning'
                        if curPlatoon:
                            curPlatoon[0].disband()
                        possiblePlatoon.addVehicle(vehicle, len(possiblePlatoon._vehicles))
                        vehicle.setState("Following")

                        # vehicle.setColor((0, 255, 255))

                    # not in the same lane
                    elif curX > -2300.00 and direction != 0:
                        # also add to the platoon list
                        # v.state --> swtich to "PreparingForPlatooning"
                        # get relevant vehicles
                        #   dis to leader
                        #       beyond the leader, front join
                        #       behind the last member, back join
                        #       otherwise, given two relevant members
                        # form a context to bind them
                        # if curPlatoon:
                        #     curPlatoon[0].disband()
                        
                        if vehicle.getLane() == 'gneE4_0':
                            frontVeh, rearVeh = self.getBestMergePosition(vehicle,possiblePlatoon,fisMergeLoc)
                            if frontVeh == -1:
                                pdb.set_trace()
                        else:
                            frontVeh, rearVeh = possiblePlatoon.getMergePosition(leadVeh[0], vehicle, direction)
                            
                        # add to merge list
                        self.mergelist[vehicleID] = (frontVeh, rearVeh)
                        vehicle.setState("PreparingForPlatooning")
                        possiblePlatoon.getLeadVehicle().setState("LeadingWithOperation")
                        # if frontVeh == -1:
                        #     possiblePlatoon.addVehicle(vehicle, 0)
                        # elif rearVeh == -1:
                        #     possiblePlatoon.addVehicle(vehicle, len(possiblePlatoon._vehicles))
                        # else:
                        #     merge_id = possiblePlatoon._vehicles.index(rearVeh)
                        #     possiblePlatoon.addVehicle(vehicle, merge_id)
                else:
                    if not self.getPlatoonByVehicle(vehicle):
                        self.createPlatoon([vehicle, ])
                    # vehicle.setColor((230, 0, 255))
            
        # Regulate individual behavior within this block
        if self.platoonCreation:
            # Handles a single step of the simulation
            # Update all active platoons in the scenario

            # leading
            # following
            # search for platooning --> speed/gap regulation --> keep inter-platoon gap
            # prepareforplatooning --> accelerating, check position
            #   opening gap --> if the merging veh reach the given point
            #               --> platoon member keeps creating the gap
            #   merging
            # operational state: leading/following
            for platoon in self.getActivePlatoons():
                platoon.update()
                vehsInPlatoon = platoon.getAllVehicles()
                # if len(vehsInPlatoon) > 1:
                for v in vehsInPlatoon:
                        
                        
                    #traci.vehicle.setLaneChangeMode(v.getName(), 0b000100000000)
                    if not v.isActive():
                        continue
                    # v._state = 'Platooning'
                    # vehicle needs to follow the given tau (desired tau)
                    # if current tau is larger than the des_tau
                    # speed-up to catch up with preceding vehicle
                    # if v.getName() == platoon.getLeadVehicle().getName():
                    if v.getState() == "Leading" or v.getState() == "LeadingWithOperation":
                        v.setTau(INTER_TAU)
                        v.setSpeedMode(SPD_ALL_CHECK)
#                        if -50 < v.getPosition()[0] < 450:
                        if v.getLane() == 'gneE4_1':
                            rightLeadVeh = v.getRightLeader()
                            if rightLeadVeh:
                                if rightLeadVeh[1] < 150:                                   
                                    des_speed = self.getDesiredSpeed(v,gfs_pl_speed)
                                    #des_speed = max(15.0,des_speed)
                                    v.setSpeed(des_speed)
                                else:
                                    v.setSpeed(SPEED)
                            else:
                                v.setSpeed(SPEED)
                        else:
                            v.setSpeed(SPEED)
                        
                        v.setColor((230, 0, 255))
                        traci.vehicle.setLaneChangeMode(v.getName(), 0b000000000000)  # to-do: encap, lane-change prohibit mode
                    elif v.getState() == "Following":
                        dis = v.getLeader(60)  # return ID and dis of leading vehicle within 60 meters (2s * speed)
                        spd = v.getSpeed()
                        v.setTau(INTRA_TAU)
                        v.setColor((0, 255, 255))
                        traci.vehicle.setLaneChangeMode(v.getName(), 0b000000000000)  # to-do: encap
                        # des_tau = v.getTau()
                        if dis is not None and dis[1] / (spd + 0.00001) > INTRA_TAU + 0.01:
                            # v.setSpeedFactor(1.2)
                            v.setSpeed(SPEED * 1.1)
                            v.setSpeedMode(SPD_DIS_SAFE)
                        elif dis is not None and dis[1] /  (spd + 0.00001) <= INTRA_TAU + 0.01:
                            # v.setSpeedFactor(1.0)
                            v.setSpeed(SPEED)
                            v.setSpeedMode(SPD_ALL_CHECK)
                    elif v.getState() == "PreparingForPlatooning":
                        if v.getLane() == 'gneE4_0':
                            des_speed = self.getDesiredSpeed(v,gfs_m)
                        else:
                            des_speed = None
                        v.setColor((255, 165, 0))
                        if v.getName() in self.mergelist.keys():
                            frontVeh, rearVeh = self.mergelist[v.getName()]
                            
                            if frontVeh != -1:
                                #dis = v.getDistance(frontVeh)
                                dis = frontVeh.getPosition()[0] - v.getPosition()[0]
                                frontSpd = frontVeh.getSpeed() # use preceding veh's speed
                                fronVehPosx = frontVeh.getPosition()[0]
                                if rearVeh != -1:
                                    rearVehPosx = rearVeh.getPosition()[0]
                                else:
                                    rearVehPosx = -1000 # very high negative number
                                if v.getLane() == 'gneE4_0':
                                    if rearVeh != -1:
                                        midPointSpeed = 0.5*(frontSpd+rearVeh.getSpeed())
                                        midPointPosx = 0.5*(frontVeh.getPosition()[0]+rearVeh.getPosition()[0])
                                    else:
                                        midPointSpeed = frontSpd - 0.5
                                        midPointPosx = frontVeh.getPosition()[0] - INTRA_TAU*frontSpd
                                    egoVehPosx = v.getPosition()[0]
                                    dc = egoVehPosx - midPointPosx
                                    des_merge_dist = 200 - egoVehPosx  # x = 200 is set as the point for the merge vehicle to meet the merge position within the platoon
                                    if des_merge_dist > 0:
                                        T_des = des_merge_dist/v.getSpeed()
                                    else:
                                        T_des = dt
                                        
                                    vM = max((midPointSpeed*T_des - dc)/T_des,0.0)
                                    v.setSpeed(vM)
                                    
                                    v.setSpeedMode(SPD_ALL_CHECK)
                                    if rearVehPosx < egoVehPosx < fronVehPosx: # If vehicle between front and rear vehicles
                                        v.setState("InPosition")
                                        
                                else:
                                    if dis > frontSpd * (INTRA_TAU + 0.2):
                                        v.setSpeed(frontSpd * 1.1)
                                        v.setSpeedMode(SPD_ALL_CHECK)
                                    elif dis < LENGTH and not des_speed:
                                        v.setSpeed(frontSpd * 0.95)
                                        v.setSpeedMode(SPD_ALL_CHECK)
                                    else:
                                        if des_speed:
                                            v.setSpeed(des_speed)
                                            
                                        else:
                                            v.setSpeed(frontSpd)
                                        
                                        v.setSpeedMode(SPD_ALL_CHECK)
                                        v.setState("InPosition")
                                if rearVeh != -1:
                                    rearVeh.setState("OpeningGap")
                            elif frontVeh == -1: # must have rearVeh
                                dis = v.getDistance(rearVeh)
                                rearSpd = rearVeh.getSpeed()
                                if dis > rearSpd * (INTRA_TAU - 0.05):
                                    v.setSpeed(SPEED)
                                    v.setSpeedMode(SPD_ALL_CHECK)
                                    v.setState("InPosition")
                                    # to-do: a specific state for this vehicle
                                    rearVeh.setTau(INTRA_TAU)
                                    rearVeh.setState("LeadingWithOperation")
                                else:
                                    if des_speed:
                                        v.setSpeed(des_speed)
                                    else:
                                        v.setSpeed(SPEED * 1.1)
                                    #v.setSpeed(SPEED * 1.1)
                                    
                                    v.setSpeedMode(SPD_ALL_CHECK)
                    elif v.getState() == "InPosition":
                        v.setColor((255, 165, 0))
                        frontVeh, rearVeh = self.mergelist[v.getName()]
                        frontEdge = -100
                        rearEdge = -100
                        if frontVeh != -1:
                            frontEdge = frontVeh.getEdge()
                        if rearVeh != -1:
                            rearEdge = rearVeh.getEdge()
                        egoEdge = v.getEdge()
                        laneID = -1
                        
                        if frontEdge != -100 and egoEdge == frontEdge:
                            laneID = frontVeh.getLaneIndex()
                        elif rearEdge != -100 and egoEdge == rearEdge:
                            laneID = rearVeh.getLaneIndex()
                        
                        if laneID != -1:
                            v.setState("Merging")
                            #v.changeLane(laneID)
                            #traci.vehicle.setLaneChangeMode(v.getName(), 0b000100000000)
                            v.changeLane(laneID)
                            
                            
                    elif v.getState() == "Merging":
                        
                        v.setColor((255, 255, 0))
                        frontVeh, rearVeh = self.mergelist[v.getName()]
                        frontEdge = -100
                        rearEdge = -100
                        if frontVeh != -1:                            
                            frontEdge = frontVeh.getEdge()
                            
                        if rearVeh != -1:
                            rearEdge = rearVeh.getEdge()
                        egoEdge = v.getEdge()
                        egoLane = v.getLaneIndex()
                        laneID = -1
                        
                        if frontEdge != -100 and egoEdge == frontEdge:
                            laneID = frontVeh.getLaneIndex()
                        elif rearEdge != -100 and egoEdge == rearEdge:
                            laneID = rearVeh.getLaneIndex()
                        
                        
                        if laneID != -1:
                            v.changeLane(laneID)
                        
                        if v.getLane() == 'gneE4_0':                            
                            des_speed = self.getDesiredSpeed(v,gfs_m)
                        else:
                            des_speed = None
                        
                        if frontVeh != -1:
                            #dis = v.getDistance(frontVeh)
                            dis = frontVeh.getPosition()[0] - v.getPosition()[0]
                            frontSpd = frontVeh.getSpeed()  # use preceding veh's speed
                            
                            if dis > frontSpd * (INTRA_TAU + 0.2):
                                v.setSpeed(frontSpd * 1.1)
                                v.setSpeedMode(SPD_ALL_CHECK)
                            elif dis < LENGTH and not des_speed:
                                v.setSpeed(frontSpd * 0.95)
                                v.setSpeedMode(SPD_ALL_CHECK)
                            else:
                                if des_speed:
                                    v.setSpeed(des_speed)
                                else:
                                    v.setSpeed(frontSpd)
                                
                                v.setSpeedMode(SPD_ALL_CHECK)

                            if frontEdge != -100 and egoEdge == frontEdge:
                                if egoLane == frontVeh.getLaneIndex():
                                    v.setState("Following")
                                    if rearEdge != -100:
                                        rearVeh.setState("Following")
                                    curPlatoon = self.getPlatoonByVehicle(v)
                                    if 'flow_1' in v._name:
                                        if curPlatoon:
                                            curPlatoon[0].disband()
                                        mainPlatoon = self.getPlatoonByVehicle(frontVeh)
                                        insertPoint = mainPlatoon[0]._vehicles.index(frontVeh)+1
                                        mainPlatoon[0].addVehicle(v,insertPoint)
                                        v.setState("Following")
                                        
                                    else:
                                        # end this operation session and allow another operation
                                        curPlatoon[0].getLeadVehicle().setState("Leading")
                            elif rearEdge != -100 and egoEdge == rearEdge:
                                if egoLane == rearVeh.getLaneIndex():
                                    v.setState("Following")
                                    rearVeh.setState("Following")
                                    curPlatoon = self.getPlatoonByVehicle(v)
                                    if 'flow_1' in v._name:
                                        if curPlatoon:
                                            curPlatoon[0].disband()
                                        mainPlatoon = self.getPlatoonByVehicle(rearVeh)
                                        insertPoint = mainPlatoon[0]._vehicles.index(rearVeh)
                                        mainPlatoon[0].addVehicle(v,insertPoint)
                                        v.setState("Following")
                                        
                                    else:
                                        # end this operation session and allow another operation
                                        curPlatoon[0].getLeadVehicle().setState("Leading")
                        elif frontVeh == -1:
                            dis = v.getDistance(rearVeh)
                            rearSpd = rearVeh.getSpeed()
                            
                            # tweak this parameter
                            if dis > rearSpd * (INTRA_TAU - 0.05):
                                v.setSpeed(SPEED)
                                v.setSpeedMode(SPD_ALL_CHECK)
                            else:
                                if des_speed:
                                    v.setSpeed(des_speed)
                                else:
                                    v.setSpeed(rearSpd * 1.1)
                                
                                v.setSpeedMode(SPD_ALL_CHECK)
                            if rearEdge != -100 and egoEdge == rearEdge:
                                if egoLane == rearVeh.getLaneIndex():
                                    v.setState("Leading")
                                    curPlt = self.getPlatoonByVehicle(v)[0]
                                    curPlt.getAllVehicles().pop(0) # remove v from list
                                    curPlt.addVehicle(v, 0)
                                    # end this operation session and allow another operation
                                    rearVeh.setState("Following")
                    elif v.getState() == "OpeningGap":
                        v.setColor((0, 0, 255))                         
                        
                        if v.getLane() == 'gneE4_1':
                            des_speed = self.getDesiredSpeed(v,gfs_m)
                            v.setSpeed(des_speed)
                            v.setSpeedMode(SPD_ALL_CHECK)
                        else:
                        
                            v.setSpeed(SPEED)
                            v.setSpeedMode(SPD_ALL_CHECK)
                            cur_Tau = traci.vehicle.getTau(v.getName())
                            if cur_Tau < 1.1:
                                cur_Tau += 0.05
                            traci.vehicle.setTau(v.getName(), cur_Tau)
                    elif v.getState() == 'RequestDissolve':
                        
                        v.setSpeed(SPEED)
                        v.setSpeedMode(SPD_ALL_CHECK)
                        plt_lead = self.dissolvelist[v.getName()]
                        if plt_lead != -1:
                            lead = v.getLeader(60)
                            dis = v.getLeader(60)[1]
                            spd = v.getSpeed()
                            if dis / spd >= DISSOLVE_GAP:
                                plt_lead.setState('MaintainDissolveGap')
                                v.setState('Dissolving')
                                # to-do: target lane should be the lane along the new route
                                if v.getLaneIndex == 0:
                                    v.changeLane(1)
                                else:
                                    v.changeLane(0)
                        else:
                            # the dissolving veh is platoon leader
                            v.setState('Dissolving')
                            # to-do: target lane should be the lane along the new route
                            if v.getLaneIndex == 0:
                                v.changeLane(1)
                            else:
                                v.changeLane(0)
                    elif v.getState() == 'PreParingForDissolve':
                        v.setSpeed(SPEED * 1.1)
                        v.setSpeedMode(SPD_ALL_CHECK)
                    elif v.getState() == 'MaintainDissolveGap':  # state only available for platoon leader
                        v.setSpeed(SPEED)
                        v.setSpeedMode(SPD_ALL_CHECK)
                    elif v.getState() == 'Dissolving':
                        v.setColor((200, 66, 245))
                        tarLaneID = v.getTargetLane()
                        #v.changeLane(tarLaneID)
                        v.changeLane(tarLaneID)
                        
                        laneID = v.getLaneIndex()
                        # if complete the lane changeWWW
                        if laneID == tarLaneID:
                            v.setSpeed(SPEED)
                            v.setSpeedMode(SPD_ALL_CHECK)
                            # set all parameters here since the CDA disabled vehicle will not be accessed in the logic
                            v.setState('CDADisabled')
                            v.setColor((122, 130, 111))
                            v.setTau(INTER_TAU)
                            leadVeh = self.dissolvelist[v.getName()]
                            platoon = self.getPlatoonByVehicle(v)[0]
                            platoon.removeVehicle(v)
                            platoon.getLeadVehicle().setState('Leading')
        return np.mean(allSpeeds)                            
    
    def add_vehicles(self, n, real_engine=False):
        """
        Adds a platoon of n vehicles to the simulation, plus an additional one
        farther away that wants to join the platoon
        :param n: number of vehicles of the platoon
        :param real_engine: set to true to use the realistic engine model,
        false to use a first order lag model
        :return: returns the topology of the platoon, i.e., a dictionary which
        indicates, for each vehicle, who is its leader and who is its front
        vehicle. The topology can the be used by the data exchange logic to
        automatically fetch data from leading and front vehicle to feed the CACC
        """
        # add a platoon of n vehicles
        topology = {}
        for i in range(n):
            vid = "v.%d" % i
            ut.add_vehicle(vid, (n - i + 1) * (DISTANCE + LENGTH) + 50, 0, SPEED, DISTANCE,
                        real_engine)
            # change_lane(vid, 0)

        # add a vehicle that wants to join the platoon
        vid = "v.%d" % n
        ut.add_vehicle(vid, (n + 1) * (DISTANCE + LENGTH) + 30, 1, SPEED, DISTANCE, real_engine)
        # change_lane(vid, 1)
        # set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        # set_par(vid, cc.PAR_CACC_SPACING, JOIN_DISTANCE)
        # return topology


    def getAllVehicleNames(self):
        return flatten(v.getName() for v in self.getAllVehicles())

    def getAllVehicles(self):
        return self.vehicles

    def getVehicleByID(self, vehicleID):
        return self.vehicles[vehicleID]

    def addNewVehicle(self, vehicleID, vehicle):
        self.vehicles[vehicleID] = vehicle

    def getVehicleType(self, vehicleID):
        return traci.vehicle.getTypeID(vehicleID).split("@")[0]
    
    def getInputs2FIS(self, veh):
    
        Y_ML = [-8.0,-4.8,-1.6]
        veh_name = veh[1]
        veh_edge = veh[2][veh_name][81]
        veh_speed = veh[2][veh_name][64]  # 66 is the key for position
        veh_pos = veh[2][veh_name][66]  # 66 is the key for position
        
        all_vehs_around = list(veh[2].keys()) # This also includes the current vehicle. So, this is removed in next line
        all_vehs_around.remove(veh_name)
        
        distances = [150]*6  # [left ahead, left behind, same lane ahead, same lane behind, right ahead, right behind]
        speeds = [50]*6
        signals = [0]*6 # signal =0 either means no signal or no vehicle or no lane
        
        if veh_edge == 'gneE1_0' or veh_edge == ':gneJ1_0_0':
            all_vehs_merge = [a for a in all_vehs_around if veh[2][a][81]=='gneE1_0']
            pos_all_vehs_merge = [veh[2][a][66] for a in all_vehs_merge]
        
            DISTx_merge = [b[0]-veh_pos[0] for b in pos_all_vehs_merge]
            
            try:
                dist_ahead = min([c for c in DISTx_merge if c>0])
            except:
                dist_ahead = []
                
            try:
                dist_behind = max([c for c in DISTx_merge if c<0])
            except:
                dist_behind = []
              
            veh_id_ahead = [all_vehs_merge[i] for i,a in enumerate(DISTx_merge) if a == dist_ahead]
            veh_id_behind = [all_vehs_merge[i] for i,a in enumerate(DISTx_merge) if a == dist_behind]
            if dist_ahead:
                distances[2] = dist_ahead
                speeds[2] = veh[2][veh_id_ahead[0]][64]  # 64 refers to speed
                signals[2] = veh[2][veh_id_ahead[0]][91]
                
            if dist_behind:
                distances[3] = abs(dist_behind)
                speeds[3] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                signals[3] = veh[2][veh_id_behind[0]][91]
            
        else:
            veh_lane = Y_ML.index(veh_pos[1])
        
            lanes_cons = [Y_ML[a] for a in [veh_lane-1,veh_lane,veh_lane+1] if a >-1 and a<3]
            pos_all_vehs = [veh[2][a][66] for a in all_vehs_around]
        
            DISTx = [[b[0]-veh_pos[0],b[1]] for b in pos_all_vehs]
            for y in lanes_cons:
                try:
                    dist_ahead = min([c[0] for c in DISTx if c[0]>0 and c[1]==y])
                except:
                    dist_ahead = []
                    
                try:
                    dist_behind = max([c[0] for c in DISTx if c[0]<0 and c[1]==y])
                except:
                    dist_behind = []
                  
                veh_id_ahead = [all_vehs_around[i] for i,a in enumerate(DISTx) if a[0] == dist_ahead]
                veh_id_behind = [all_vehs_around[i] for i,a in enumerate(DISTx) if a[0] == dist_behind]
                
                if y-veh_pos[1] > 0:
                    if dist_ahead:
                        distances[0] = dist_ahead
                        speeds[0] = veh[2][veh_id_ahead[0]][64]  # 64 refers to speed
                        signals[0] = veh[2][veh_id_ahead[0]][91]
                    if dist_behind:
                        distances[1] = abs(dist_behind)
                        speeds[1] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                        signals[1] = veh[2][veh_id_behind[0]][91]
                        
                elif y-veh_pos[1] == 0:
                    if dist_ahead:
                        distances[2] = dist_ahead
                        speeds[2] = veh[2][veh_id_ahead[0]][64]  # 64 refers to speed
                        signals[2] = veh[2][veh_id_ahead[0]][91]
                        
                    if dist_behind:
                        distances[3] = abs(dist_behind)
                        speeds[3] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                        signals[3] = veh[2][veh_id_behind[0]][91]
                        
                elif y-veh_pos[1] < 0:
                    if dist_ahead:
                        distances[4] = dist_ahead
                        speeds[4] = veh[2][veh_id_ahead[0]][64]  # 64 refers to speed
                        signals[4] = veh[2][veh_id_ahead[0]][91]
                        
                    if dist_behind:
                        distances[5] = abs(dist_behind)
                        speeds[5] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                        signals[5] = veh[2][veh_id_behind[0]][91]
        # The below if statements make the distances and speeds of lanes that do not exist around each vehicle to 0
        if veh_edge == 'gneE1_0' or veh_edge == ':gneJ1_0_0':
            distances[0:2] = [a if a < 150 else -1 for a in distances[0:2]]
            distances[4:] = [a if a < 150 else -1 for a in distances[4:]]
            speeds[0:2] = [a if a < 50 else -1 for a in speeds[0:2]]
            speeds[4:] = [a if a < 50 else -1 for a in speeds[4:]]
        elif veh_edge == 'gneE0_0' or veh_edge == 'gneE4_0' or veh_edge == 'gneE5_0' or veh_edge == ':gneJ6_0_1':
            distances[4:] = [a if a < 150 else -1 for a in distances[4:]]
            speeds[4:] = [a if a < 50 else -1 for a in speeds[4:]]
        elif veh_edge == 'gneE0_1' or veh_edge == 'gneE4_2' or veh_edge == 'gneE5_1' or veh_edge == ':gneJ1_1_1' or veh_edge == ':gneJ6_0_2':
            distances[0:2] = [a if a < 150 else -1 for a in distances[0:2]]
            speeds[0:2] = [a if a < 50 else -1 for a in speeds[0:2]]
        allInputs = distances + speeds + signals + list(veh_pos) + [veh_speed]
        return allInputs
    
    def getBestMergePosition(self,merge_veh, platoon, fisMergeLoc):
        
        platoon_vehicles = [a for a in platoon._vehicles if abs(a.getPosition()[1]+4.8) < 1e-3]
        platoon_length = len(platoon_vehicles)
        veh_name = merge_veh._name
        veh_context = merge_veh.getContext()
        
        veh_all = [0, veh_name, veh_context]
        
        inputs = self.getInputs2FIS(veh_all)
        
        all_scores = []
        for j in range(platoon_length+1): # Number of gaps in platoon where vehicle can merge
            
            if j == 0:
                leadPos = [500,500]
                leadSpeed = 50
                rear = platoon_vehicles[j]
                rearPos = rear.getPosition()
                rearSpeed = rear.getSpeed()
                
                    
            elif j == platoon_length:
                lead = platoon_vehicles[j-1]
                leadPos = lead.getPosition()
                leadSpeed = lead.getSpeed()
                rearPos = [500,500]
                rearSpeed = 50
            else:
                lead = platoon_vehicles[j-1]
                rear = platoon_vehicles[j]
                leadPos = lead.getPosition()
                leadSpeed = lead.getSpeed()
                
                rearPos = rear.getPosition()
                rearSpeed = rear.getSpeed()
                
            full_list2 = inputs[0:4] + inputs[6:10] + inputs[12:16] + [inputs[18]] + [inputs[20]] + [leadPos[0]] + [leadSpeed] + [rearPos[0]] + [rearSpeed] #y-coordinate of ego vehicle is ignored
            
            score = fisMergeLoc.eval_op(np.array(full_list2))
            all_scores.append(score)
        
        best_id = all_scores.index(max(all_scores)) # ID of the best position to merge into the platoon
        if best_id == 0:
            leadVeh = -1
            rearVeh = platoon_vehicles[best_id]
        elif best_id == platoon_length:
            leadVeh = platoon_vehicles[best_id-1]
            rearVeh = -1
        else:
            leadVeh = platoon_vehicles[best_id-1]
            rearVeh = platoon_vehicles[best_id]
            
        return  leadVeh, rearVeh
            
            # self.relevantMergeList.append(full_list2)

    def getDesiredSpeed(self,vehicle, gfs):
        
        veh_name = vehicle._name
        cur_speed = vehicle.getSpeed()
        veh_context = vehicle.getContext()
        
        veh_all = [0, veh_name, veh_context]
        
        inputs = self.getInputs2FIS(veh_all)
        outs = gfs.eval_op(inputs)
        accln = 3.55*outs[0]-0.95 # accln in the range[-4.5,2.6]
        des_speed = cur_speed + accln*dt
        
        return des_speed
    