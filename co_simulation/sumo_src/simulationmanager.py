# -*- coding: utf-8 -*-
"""
Created on Tue Dec 1, 2020

@creator: Anoop Sathyan
@authors: Yi Guo, Anoop Sathyan, Jiaqi Ma
"""

import traci
import numpy as np
import co_simulation.sumo_src.utils as ut

from co_simulation.sumo_src.intersectionController import IntersectionController
from co_simulation.sumo_src.platoon import Platoon
from co_simulation.sumo_src.simlib import flatten
from co_simulation.sumo_src.vehicle import Vehicle

# vehicle length
LENGTH = 5
# inter-vehicle distance
DISTANCE = 30
# inter-vehicle distance when leaving space for joining
JOIN_DISTANCE = DISTANCE * 2
# cruising speed
SPEED = 31.29

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
FORMATION_GAP = 1.3

# vehicle type
HDV_TYPE = 'DEFAULT_VEHTYPE'
CDA_TYPE = 'vType_0'

# formation related
FORM_TIME = 15.0
SUSPENDING_TIME = 1.0


class SimulationManager():

    def __init__(self, pCreation=True, iCoordination=True, iZipping=True):
        self.intersections = []
        self.platoons = list()
        self.platoonCreation = pCreation
        self.vehicles = dict()
        self.mergelist = dict()
        self.mergingBuffer = dict()
        self.ticklist = dict()
        self.dissolvelist = dict()
        self.maxStoppedVehicles = dict()
        self.dissolvetest = 0
        self.relevantPlatoonHist = []  # For data collection
        self.relevantMergeList = []  # For data collection for merge position
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
        return count / length

    def getPlatoonByLane(self, lane):
        # Gets platoons corresponding to a given lane
        return [p for p in self.getActivePlatoons() if lane == p.getLane()]

    def getPlatoonByVehicle(self, v):
        for p in self.getActivePlatoons():
            if v.getName() in p.getAllVehiclesByName():
                return [p]
            # p for p in self.getActivePlatoons() if v in p.getAllVehiclesByName()
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
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length and abs(
                            y_leftFollowVeh + 4.8) < 1e-3:
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if 'Leading' in possiblePlatoon[0].getLeadVehicle().getState() \
                                or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                    possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], -1, leftFollowVeh

            # potential left preceding platoon
            if leftLeadVeh and leftLeadVeh[1] < 50:  # and direction == 0
                leadIns = self.vehicles[leftLeadVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    y_Leader = possiblePlatoon[0]._vehicles[0].getPosition()[1]
                    y_leftLeadVeh = leadIns.getPosition()[1]
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length and abs(
                            y_leftLeadVeh + 4.8) < 1e-3:
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if 'Leading' in possiblePlatoon[0].getLeadVehicle().getState() \
                                or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                    possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], 1, leftLeadVeh
        else:
            if leadVeh and leadVeh[1] < 50:  # and direction == 0
                leadIns = self.vehicles[leadVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:  # Dec 30, 2020 --> Max_Platoon_Length
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if possiblePlatoon[0].getLeadVehicle().getState() == "Leading" \
                                or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                    possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], 0, leadVeh
            # potential left preceding platoon
            if leftLeadVeh and leftLeadVeh[1] < 50:  # and direction == 0
                leadIns = self.vehicles[leftLeadVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:  # Dec 30, 2020 --> Max_Platoon_Length
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if possiblePlatoon[0].getLeadVehicle().getState() == "Leading" \
                                or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                    possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], 1, leftLeadVeh

            # potential right preceding platoon
            if rightLeadVeh and rightLeadVeh[1] < 50:  # and direction == 0
                leadIns = self.vehicles[rightLeadVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:  # Dec 30, 2020 --> Max_Platoon_Length
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if possiblePlatoon[0].getLeadVehicle().getState() == "Leading" \
                                or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
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
                        if possiblePlatoon[0].getLeadVehicle().getState() == "Leading" \
                                or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                    possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], -1, leftFollowVeh

            # potential right rear platoon
            if rightFollowVeh and rightFollowVeh[1] < 50:  # and direction == 0
                leadIns = self.vehicles[rightFollowVeh[0]]
                possiblePlatoon = self.getPlatoonByVehicle(leadIns)
                if possiblePlatoon:
                    if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:
                        # ensure the platoon is not engaging another operation
                        # the leader state could be Leading or SearchForPlatooning
                        if possiblePlatoon[0].getLeadVehicle().getState() == "Leading" \
                                or possiblePlatoon[0].getLeadVehicle().getState() == "SearchingForPlatooning":
                            if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                    possiblePlatoon[0].getAllVehicles():
                                return possiblePlatoon[0], -1, rightFollowVeh
        # added the default return value
        return None, -100, leadVeh

    def handleSimulationStepFIS(self, time, fisMergeLoc, gfs_pl_speed, gfs_m):
        allVehicles = traci.vehicle.getIDList()
        # Check mark vehicles as in-active if they are outside the map

        allSpeeds = []
        for v in list(self.vehicles):
            if v not in allVehicles:  # .getName()
                v_ins = self.getAllVehicles().pop(v)
                if v_ins is not None:
                    v_ins.setInActive()
        # Update all platoons active status
        for p in self.getActivePlatoons():
            p.updateIsActive()

        # The default value of platoonCreation is True
        # Organize platoon within this block
        if self.platoonCreation:
            # See whether there are any vehicles that are not in a platoon with more than one vehicle
            vehiclesNotInPlatoons = [v for v in allVehicles if v not in self.getAllVehicles() or
                                     (v in self.getAllVehicles() and self.getAllVehicles()[v].getState() ==
                                      "SearchingForPlatooning")]

            for vehicleID in vehiclesNotInPlatoons:

                # consider HDV for mixed-traffic each HDV should appear in this loop only once
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
                # have not completed the lane change, align with the center of lane
                if vehicle.getName() in self.mergingBuffer.keys():
                    tmp_lane, tmp_direction = self.mergingBuffer[vehicle.getName()]

                    if vehicle.getLaneIndex() == tmp_lane and abs(vehicle.getLatPos()) <= 0.01:
                        self.mergingBuffer.pop(vehicle.getName(), None)
                    else:
                        # have not aligned to the original lane
                        vehicle.changeLane(tmp_lane, tmp_direction)
                        continue

                possiblePlatoon, direction, leadVeh = self.getReleventPlatoon(vehicle)

                if vehicle.getLane() == 'gneE4_1' and direction == -1:
                    possiblePlatoon = None

                # no possible platooning opportunity, then no cooperative merge
                if vehicle.getLane() == 'gneE4_0' and not possiblePlatoon:
                    vehicle.changeLane(1, 0)
                    self.mergelist[vehicle.getName()] = (-1, -1)
                    self.ticklist[vehicle.getName()] = time
                    vehicle.setState("Merging")

                if possiblePlatoon:  # and direction == 0

                    curPlatoon = self.getPlatoonByVehicle(vehicle)
                    curX, curY = vehicle.getPosition()

                    # same lane join
                    if direction == 0 and vehicle.getLane() != 'gneE4_0':
                        if len(possiblePlatoon._vehicles) == 1:
                            possiblePlatoon.getLeadVehicle().setState("Leading")
                        if curPlatoon:
                            curPlatoon[0].disband()
                        possiblePlatoon.addVehicle(vehicle, len(possiblePlatoon._vehicles))
                        vehicle.setState("Following")

                        # vehicle.setColor((0, 255, 255))

                    # not in the same lane
                    elif 200.00 < curX < 2300 and direction != 0:
                        # if in the suspending time
                        if vehicleID in self.ticklist.keys():
                            tmp_time = self.ticklist[vehicleID]
                            if time - tmp_time <= SUSPENDING_TIME:
                                traci.vehicle.setColor(vehicleID, (255, 0, 0))
                                continue

                        if vehicle.getLane() == 'gneE4_0':
                            frontVeh, rearVeh = self.getBestMergePosition(vehicle, possiblePlatoon, fisMergeLoc)
                        else:
                            if curPlatoon:
                                curPlatoon[0].disband()
                            frontVeh, rearVeh = possiblePlatoon.getMergePosition(leadVeh[0], vehicle, direction)

                        # add to merge list
                        self.mergelist[vehicleID] = (frontVeh, rearVeh)
                        vehicle.setState("PreparingForPlatooning")
                        possiblePlatoon.getLeadVehicle().setState("LeadingWithOperation")

                else:
                    if not self.getPlatoonByVehicle(vehicle):
                        self.createPlatoon([vehicle, ])
                    # vehicle.setColor((230, 0, 255))

        # Regulate individual behavior within this block
        if self.platoonCreation:
            # Handles a single step of the simulation
            # Update all active platoons in the scenario
            for platoon in self.getActivePlatoons():
                platoon.updateIsActive()
                vehsInPlatoon = platoon.getAllVehicles()
                for v in vehsInPlatoon:
                    if not v.isActive():
                        continue
                    else:
                        # all platoon members are set to lane-change prohibition
                        traci.vehicle.setLaneChangeMode(v.getName(), 0b001000000000)
                    if v.getState() == "Leading" or v.getState() == "LeadingWithOperation":
                        v.setTau(INTER_TAU)
                        v.setSpeedMode(SPD_ALL_CHECK)
                        if 1150 < v.getPosition()[0] < 1650:

                            des_speed = self.getDesiredSpeed(v, gfs_pl_speed)
                            # des_speed = max(15.0,des_speed)
                            v.setSpeed(des_speed)
                        else:
                            v.setSpeed(SPEED)

                        v.setColor((230, 0, 255))
                        traci.vehicle.setLaneChangeMode(v.getName(),
                                                        0b001000000000)  # TODO: encap, lane-change prohibit mode
                    elif v.getState() == "Following":
                        dis = v.getLeader(60)
                        spd = v.getSpeed()
                        v.setTau(INTRA_TAU)
                        v.setColor((0, 255, 255))
                        traci.vehicle.setLaneChangeMode(v.getName(), 0b001000000000)  # to-do: encap
                        # des_tau = v.getTau()
                        if dis is not None and dis[1] / (spd + 0.00001) > INTRA_TAU + 0.01:
                            # v.setSpeedFactor(1.2)
                            v.setSpeed(SPEED * 1.1)
                            v.setSpeedMode(SPD_DIS_SAFE)
                        elif dis is not None and dis[1] / (spd + 0.00001) <= INTRA_TAU + 0.01:
                            # v.setSpeedFactor(1.0)
                            v.setSpeed(SPEED)
                            v.setSpeedMode(SPD_ALL_CHECK)
                    elif v.getState() == "PreparingForPlatooning":
                        if v.getLane() == 'gneE4_0':
                            des_speed = self.getDesiredSpeed(v, gfs_m)
                        else:
                            des_speed = None
                        v.setColor((255, 165, 0))
                        if v.getName() in self.mergelist.keys():
                            frontVeh, rearVeh = self.mergelist[v.getName()]

                            if frontVeh != -1:
                                # dis = v.getDistance(frontVeh)
                                dis = frontVeh.getPosition()[0] - v.getPosition()[0]
                                frontSpd = frontVeh.getSpeed()  # use preceding veh's speed
                                fronVehPosx = frontVeh.getPosition()[0]
                                if rearVeh != -1:
                                    rearVehPosx = rearVeh.getPosition()[0]
                                else:
                                    rearVehPosx = -1000  # very high negative number
                                if v.getLane() == 'gneE4_0':
                                    if rearVeh != -1:
                                        midPointSpeed = 0.5 * (frontSpd + rearVeh.getSpeed())
                                        midPointPosx = 0.5 * (frontVeh.getPosition()[0] + rearVeh.getPosition()[0])
                                    else:
                                        midPointSpeed = frontSpd - 0.5
                                        midPointPosx = frontVeh.getPosition()[0] - INTRA_TAU * frontSpd
                                    egoVehPosx = v.getPosition()[0]
                                    dc = egoVehPosx - midPointPosx
                                    des_merge_dist = 1450 - egoVehPosx
                                    if des_merge_dist > 0:
                                        T_des = des_merge_dist / (v.getSpeed() + 0.0001)
                                    else:
                                        T_des = dt

                                    vM = max((midPointSpeed * T_des - dc) / (T_des + 0.0001), 0.0)
                                    v.setSpeed(vM)

                                    v.setSpeedMode(SPD_ALL_CHECK)
                                    if rearVehPosx < egoVehPosx < fronVehPosx:
                                        v.setState("InPosition")

                                else:
                                    if dis > frontSpd * (INTRA_TAU + 0.2):
                                        v.setSpeed(frontSpd * 1.1)
                                        # beware of this change
                                        tmp_leader = v.getLeader()
                                        if tmp_leader and tmp_leader[1] < 50:
                                            tmp_ego_spd = v.getSpeed()
                                            if tmp_leader[1] / (tmp_ego_spd + 0.0001) <= INTRA_TAU:
                                                v.setSpeedMode(SPD_ALL_CHECK)
                                            else:
                                                v.setSpeedMode(SPD_DIS_SAFE)
                                        else:
                                            v.setSpeedMode(SPD_DIS_SAFE)
                                    elif dis < LENGTH and not des_speed:
                                        v.setSpeed(frontSpd * 0.9)
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
                                    self.ticklist[v.getName()] = -1
                                else:
                                    self.ticklist[v.getName()] = time
                            elif frontVeh == -1:  # must have rearVeh
                                dis = v.getDistance(rearVeh)
                                rearSpd = rearVeh.getSpeed()
                                if v.getLane() == 'gneE4_0':
                                    midPointSpeed = rearSpd + 0.5
                                    midPointPosx = rearVeh.getPosition()[0] + INTRA_TAU * rearSpd
                                    rearVehPosx = rearVeh.getPosition()[0]
                                    egoVehPosx = v.getPosition()[0]
                                    dc = egoVehPosx - midPointPosx
                                    des_merge_dist = 1450 - egoVehPosx
                                    if des_merge_dist > 0:
                                        T_des = des_merge_dist / (v.getSpeed() + 0.0001)
                                    else:
                                        T_des = dt

                                    vM = max((midPointSpeed * T_des - dc) / (T_des + 0.0001), 0.0)
                                    v.setSpeed(vM)

                                    v.setSpeedMode(SPD_ALL_CHECK)

                                    if rearVehPosx < egoVehPosx:  # If vehicle between front and rear vehicles
                                        v.setState("InPosition")
                                else:
                                    if dis > rearSpd * (INTRA_TAU - 0.05):
                                        v.setSpeed(SPEED)
                                        v.setSpeedMode(SPD_ALL_CHECK)
                                        v.setState("InPosition")
                                        # front join --> tick now
                                        # record time
                                        self.ticklist[v.getName()] = time
                                        rearVeh.setTau(INTRA_TAU)
                                        rearVeh.setState("LeadingWithOperation")
                                    else:
                                        if des_speed:
                                            v.setSpeed(des_speed)
                                        else:
                                            v.setSpeed(SPEED * 1.1)
                                        # v.setSpeed(SPEED * 1.1)

                                        v.setSpeedMode(SPD_ALL_CHECK)
                    elif v.getState() == "InPosition":
                        v.setColor((255, 165, 0))
                        frontVeh, rearVeh = self.mergelist[v.getName()]
                        frontEdge = -100
                        rearEdge = -100
                        if v.getName() in self.ticklist.keys():
                            tick_time = self.ticklist[v.getName()]
                            # within the given time limit
                            if tick_time == -1 or time - tick_time <= FORM_TIME:
                                if frontVeh != -1:
                                    frontEdge = frontVeh.getEdge()
                                if rearVeh != -1:
                                    rearEdge = rearVeh.getEdge()
                                egoEdge = v.getEdge()
                                laneID = -1
                                # check if the gap was created
                                if tick_time == -1:
                                    if rearVeh != -1:
                                        tmp_spd = rearVeh.getSpeed()
                                        tmp_dis = rearVeh.getLeader(60)[1]
                                        if tmp_dis / (tmp_spd + 0.0001) >= FORMATION_GAP - 0.05:
                                            self.ticklist[v.getName()] = time
                                if frontEdge != -100 and egoEdge == frontEdge:
                                    laneID = frontVeh.getLaneIndex()
                                elif rearEdge != -100 and egoEdge == rearEdge:
                                    laneID = rearVeh.getLaneIndex()
                                # Modified - Only change state, not sent merging command
                                if laneID >= -1:
                                    v.setState("Merging")
                            # abandon the formation
                            else:
                                v.setState("SearchingForPlatooning")
                                v.setTargetLane(v.getLaneIndex())
                                traci.vehicle.setLaneChangeMode(v.getName(), 0b001000000000)
                                if frontVeh != -1:
                                    curPlatoon = self.getPlatoonByVehicle(frontVeh)
                                    # end this operation session and allow another operation
                                    curPlatoon[0].getLeadVehicle().setState("Leading")
                                    if v in curPlatoon[0].getAllVehicles():
                                        curPlatoon[0].removeVehicle(v)
                                if rearVeh != -1:
                                    rearVeh.setState("Following")
                                    curPlatoon = self.getPlatoonByVehicle(rearVeh)
                                    # end this operation session and allow another operation
                                    curPlatoon[0].getLeadVehicle().setState("Leading")
                                    if v in curPlatoon[0].getAllVehicles():
                                        curPlatoon[0].removeVehicle(v)
                                # set a suspending time
                                self.ticklist[v.getName()] = time
                    # Feb 09, 2021, consider incomplete case
                    elif v.getState() == "Merging":
                        v.setColor((255, 255, 0))
                        frontVeh, rearVeh = self.mergelist[v.getName()]
                        if v.getName() in self.ticklist.keys():
                            tick_time = self.ticklist[v.getName()]
                            # within the given time limit
                            if tick_time == -1 or time - tick_time <= FORM_TIME:
                                frontEdge = -100
                                rearEdge = -100
                                if frontVeh != -1:
                                    frontEdge = frontVeh.getEdge()
                                if rearVeh != -1:
                                    rearEdge = rearVeh.getEdge()
                                # check if the gap was created
                                if tick_time == -1:
                                    if rearVeh != -1:
                                        tmp_spd = rearVeh.getSpeed()
                                        tmp_dis = rearVeh.getLeader(60)[1]
                                        if tmp_dis / (tmp_spd + 0.0001) >= FORMATION_GAP - 0.05:
                                            self.ticklist[v.getName()] = time
                                egoEdge = v.getEdge()
                                egoLane = v.getLaneIndex()
                                laneID = -1
                                if v.getLane() == 'gneE4_0':
                                    des_speed = self.getDesiredSpeed(v, gfs_m)
                                    if frontVeh == -1:
                                        front_speed = 40
                                    else:
                                        front_speed = frontVeh.getSpeed()
                                    if rearVeh == -1:
                                        rear_speed = 0
                                    else:
                                        rear_speed = rearVeh.getSpeed()
                                else:
                                    des_speed = None
                                if frontVeh != -1:
                                    # dis = v.getDistance(frontVeh)
                                    dis = frontVeh.getPosition()[0] - v.getPosition()[0]
                                    frontSpd = frontVeh.getSpeed()  # use preceding veh's speed
                                    if dis > frontSpd * (INTRA_TAU + 0.2):
                                        v.setSpeed(frontSpd * 1.1)
                                        tmp_leader = v.getLeader()
                                        # if the lane change is initiated
                                        if v.getName() in self.mergingBuffer.keys():
                                            tmp_laneID, tmp_direction = self.mergingBuffer[v.getName()]
                                            v.changeLane(tmp_laneID, tmp_direction)
                                        if tmp_leader and tmp_leader[1] < 50:
                                            tmp_ego_spd = v.getSpeed()
                                            if tmp_leader[1] / (tmp_ego_spd + 0.0001) <= INTRA_TAU:
                                                v.setSpeedMode(SPD_ALL_CHECK)
                                            else:
                                                v.setSpeedMode(SPD_DIS_SAFE)
                                        else:
                                            v.setSpeedMode(SPD_DIS_SAFE)
                                    elif dis <= LENGTH + (v.getSpeed() * INTRA_TAU) and not des_speed:
                                        v.setSpeed(frontSpd * 0.9)
                                        v.setSpeedMode(SPD_ALL_CHECK)
                                    else:
                                        if des_speed:
                                            v.setSpeed(des_speed)
                                        else:
                                            v.setSpeed(frontSpd)
                                        v.setSpeedMode(SPD_ALL_CHECK)
                                        frontEdge = frontVeh.getEdge()
                                        if egoEdge == frontEdge:
                                            laneID = frontVeh.getLaneIndex()
                                        if laneID > -1:
                                            if rearVeh == -1:
                                                v.changeLane(laneID, 1)
                                                if v.getName() not in self.mergingBuffer.keys():
                                                    self.mergingBuffer[v.getName()] = (laneID, 1)
                                            else:
                                                v.changeLane(laneID, 0)
                                                if v.getName() not in self.mergingBuffer.keys():
                                                    self.mergingBuffer[v.getName()] = (laneID, 1)
                                    if frontEdge != -100 and egoEdge == frontEdge:
                                        if egoLane == frontVeh.getLaneIndex():
                                            # update time every time
                                            self.ticklist[v.getName()] = time
                                            if laneID > -1:
                                                if rearVeh == -1:
                                                    v.changeLane(laneID, 1)
                                                else:
                                                    v.changeLane(laneID, 0)
                                            if abs(v.getLatPos()) <= 0.01:
                                                v.setState("Following")
                                                v.setTau(INTRA_TAU)
                                                if rearEdge != -100:
                                                    rearVeh.setState("Following")
                                                    rearVeh.setTau(INTRA_TAU)
                                                curPlatoon = self.getPlatoonByVehicle(v)
                                                if 'flow_1' in v._name and (
                                                        v.getLane() == 'gneE4_1' or v.getEdge() == 'gneE6_0'):
                                                    if curPlatoon:
                                                        curPlatoon[0].disband()
                                                    mainPlatoon = self.getPlatoonByVehicle(frontVeh)
                                                    insertPoint = mainPlatoon[0]._vehicles.index(frontVeh) + 1
                                                    mainPlatoon[0].addVehicle(v, insertPoint)
                                                    v.setState("Following")
                                                else:
                                                    # end this operation session and allow another operation
                                                    curPlatoon[0].getLeadVehicle().setState("Leading")
                                                self.ticklist.pop(v.getName(), None)
                                                self.mergingBuffer.pop(v.getName(), None)
                                                traci.vehicle.setLateralAlignment(v.getName(), 'center')
                                                # curPlatoon[0].getLeadVehicle().setTau(INTER_TAU)
                                    elif rearEdge != -100 and egoEdge == rearEdge:
                                        if egoLane == rearVeh.getLaneIndex():
                                            # update time every time
                                            self.ticklist[v.getName()] = time
                                            if laneID > -1:
                                                if rearVeh == -1:
                                                    v.changeLane(laneID, 1)
                                                    if v.getName() not in self.mergingBuffer.keys():
                                                        self.mergingBuffer[v.getName()] = (laneID, 1)
                                                else:
                                                    v.changeLane(laneID, 0)
                                                    if v.getName() not in self.mergingBuffer.keys():
                                                        self.mergingBuffer[v.getName()] = (laneID, 1)
                                            if abs(v.getLatPos()) <= 0.01:
                                                v.setState("Following")
                                                v.setTau(INTRA_TAU)
                                                rearVeh.setState("Following")
                                                rearVeh.setTau(INTRA_TAU)
                                                curPlatoon = self.getPlatoonByVehicle(v)
                                                # end this operation session and allow another operation
                                                curPlatoon[0].getLeadVehicle().setState("Leading")
                                                self.ticklist.pop(v.getName(), None)
                                                self.mergingBuffer.pop(v.getName(), None)
                                                traci.vehicle.setLateralAlignment(v.getName(), 'center')
                                            # curPlatoon[0].getLeadVehicle().setTau(INTER_TAU)
                                elif frontVeh == -1:
                                    if rearVeh == -1:
                                        if v.getLane() == 'gneE4_1' or v.getEdge() == 'gneE6_0':
                                            curPlatoon = self.getPlatoonByVehicle(v)
                                            if curPlatoon[0].getLeadVehicle() == v.getName():
                                                curPlatoon[0].disband()
                                                v.setState("SearchForPlatooning")
                                                self.ticklist.pop(v.getName(), None)
                                            continue
                                        else:
                                            continue
                                    dis = v.getDistance(rearVeh)
                                    rearSpd = rearVeh.getSpeed()
                                    rearVeh.setTau(INTRA_TAU)
                                    # tweak this parameter
                                    # if distance larger than a threshold, reduce speed
                                    if dis > rearSpd * (INTRA_TAU + 1.0):
                                        v.setSpeed(SPEED * 0.9)  # 0.95 --> 0.9
                                        v.setSpeedMode(SPD_ALL_CHECK)
                                        v.changeLane(rearVeh.getLaneIndex(), 1)
                                    # Between  INTRA_TAU - 0.05 and INTER_TAU - 0.5
                                    elif dis > rearSpd * (INTRA_TAU - 0.05):
                                        v.setSpeed(SPEED)
                                        v.setSpeedMode(SPD_ALL_CHECK)
                                        v.changeLane(rearVeh.getLaneIndex(), 1)
                                    else:
                                        if des_speed:
                                            v.setSpeed(des_speed)
                                        else:
                                            v.setSpeed(rearSpd * 1.1)
                                            tmp_leader = v.getLeader()
                                            if tmp_leader and tmp_leader[1] < 50:
                                                tmp_ego_spd = v.getSpeed()
                                                if tmp_leader[1] / (tmp_ego_spd + 0.0001) <= INTRA_TAU:
                                                    v.setSpeedMode(SPD_ALL_CHECK)
                                                else:
                                                    v.setSpeedMode(SPD_DIS_SAFE)
                                            else:
                                                v.setSpeedMode(SPD_DIS_SAFE)
                                    if rearEdge != -100 and egoEdge == rearEdge:
                                        if egoLane == rearVeh.getLaneIndex():
                                            # update time every time
                                            self.ticklist[v.getName()] = time
                                            if laneID > -1:
                                                if rearVeh == -1:
                                                    v.changeLane(laneID, 1)
                                                else:
                                                    v.changeLane(laneID, 0)
                                            if abs(v.getLatPos()) <= 0.01:
                                                v.setState("Leading")
                                                curPlt = self.getPlatoonByVehicle(v)[0]
                                                curPlt.getAllVehicles().pop(1)  # remove v from list
                                                curPlt.addVehicle(v, 0)
                                                # end this operation session and allow another operation
                                                rearVeh.setState("Following")
                                                self.ticklist.pop(v.getName(), None)
                                                self.mergingBuffer.pop(v.getName(), None)
                                                traci.vehicle.setLateralAlignment(v.getName(), 'center')
                            # abandon the formation
                            else:
                                v.setState("SearchingForPlatooning")
                                v.setTargetLane(v.getLaneIndex())
                                traci.vehicle.setLaneChangeMode(v.getName(), 0b001000000000)
                                if frontVeh != -1:
                                    curPlatoon = self.getPlatoonByVehicle(frontVeh)
                                    # end this operation session and allow another operation
                                    curPlatoon[0].getLeadVehicle().setState("Leading")
                                    if v in curPlatoon[0].getAllVehicles():
                                        curPlatoon[0].removeVehicle(v)
                                if rearVeh != -1:
                                    rearVeh.setState("Following")
                                    curPlatoon = self.getPlatoonByVehicle(rearVeh)
                                    # end this operation session and allow another operation
                                    curPlatoon[0].getLeadVehicle().setState("Leading")
                                    if v in curPlatoon[0].getAllVehicles():
                                        curPlatoon[0].removeVehicle(v)

                                # set a suspending time
                                self.ticklist[v.getName()] = time
                                if abs(v.getLatPos()) > 0.01:
                                    self.mergingBuffer[v.getName()] = (v.getLaneIndex(), 1)
                                else:
                                    self.mergingBuffer.pop(v.getName(), None)
                    elif v.getState() == "OpeningGap":
                        v.setColor((0, 0, 255))

                        if v.getLane() == 'gneE4_1':
                            des_speed = self.getDesiredSpeed(v, gfs_m)
                            v.setSpeed(des_speed)
                            v.setSpeedMode(SPD_ALL_CHECK)
                        else:

                            v.setSpeed(SPEED)
                            v.setSpeedMode(SPD_ALL_CHECK)
                            cur_Tau = traci.vehicle.getTau(v.getName())
                            if cur_Tau < FORMATION_GAP:
                                cur_Tau += 0.05
                            v.setTau(cur_Tau)
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
                        # v.changeLane(tarLaneID)
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

        Y_ML = [-8.0, -4.8, -1.6]
        veh_name = veh[1]
        veh_edge = veh[2][veh_name][81]
        veh_speed = veh[2][veh_name][64]  # 66 is the key for position
        veh_pos = veh[2][veh_name][66]  # 66 is the key for position

        all_vehs_around = list(
            veh[2].keys())  # This also includes the current vehicle. So, this is removed in next line
        all_vehs_around.remove(veh_name)

        distances = [150] * 6  # [left ahead, left behind, same lane ahead, same lane behind, right ahead, right behind]
        speeds = [50] * 6
        signals = [0] * 6  # signal =0 either means no signal or no vehicle or no lane

        if veh_edge == 'gneE1_0' or veh_edge == ':gneJ1_0_0':
            all_vehs_merge = [a for a in all_vehs_around if veh[2][a][81] == 'gneE1_0']
            pos_all_vehs_merge = [veh[2][a][66] for a in all_vehs_merge]

            DISTx_merge = [b[0] - veh_pos[0] for b in pos_all_vehs_merge]

            try:
                dist_ahead = min([c for c in DISTx_merge if c > 0])
            except:
                dist_ahead = []

            try:
                dist_behind = max([c for c in DISTx_merge if c < 0])
            except:
                dist_behind = []

            veh_id_ahead = [all_vehs_merge[i] for i, a in enumerate(DISTx_merge) if a == dist_ahead]
            veh_id_behind = [all_vehs_merge[i] for i, a in enumerate(DISTx_merge) if a == dist_behind]
            if dist_ahead:
                distances[2] = dist_ahead
                speeds[2] = veh[2][veh_id_ahead[0]][64]  # 64 refers to speed
                signals[2] = veh[2][veh_id_ahead[0]][91]

            if dist_behind:
                distances[3] = abs(dist_behind)
                speeds[3] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                signals[3] = veh[2][veh_id_behind[0]][91]

        else:
            # tmp_lane = [abs(p-veh_pos[1]) for p in Y_ML]
            # veh_lane = tmp_lane.index(min(tmp_lane))
            # tmp_y_value = Y_ML[veh_lane]
            # veh_lane = Y_ML.index[idx]
            veh_lane = Y_ML.index(veh_pos[1])

            lanes_cons = [Y_ML[a] for a in [veh_lane - 1, veh_lane, veh_lane + 1] if a > -1 and a < 3]
            pos_all_vehs = [veh[2][a][66] for a in all_vehs_around]

            DISTx = [[b[0] - veh_pos[0], b[1]] for b in pos_all_vehs]
            for y in lanes_cons:
                try:
                    dist_ahead = min([c[0] for c in DISTx if c[0] > 0 and c[1] == y])
                except:
                    dist_ahead = []

                try:
                    dist_behind = max([c[0] for c in DISTx if c[0] < 0 and c[1] == y])
                except:
                    dist_behind = []

                veh_id_ahead = [all_vehs_around[i] for i, a in enumerate(DISTx) if a[0] == dist_ahead]
                veh_id_behind = [all_vehs_around[i] for i, a in enumerate(DISTx) if a[0] == dist_behind]

                if y - veh_pos[1] > 0:
                    if dist_ahead:
                        distances[0] = dist_ahead
                        speeds[0] = veh[2][veh_id_ahead[0]][64]  # 64 refers to speed
                        signals[0] = veh[2][veh_id_ahead[0]][91]
                    if dist_behind:
                        distances[1] = abs(dist_behind)
                        speeds[1] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                        signals[1] = veh[2][veh_id_behind[0]][91]

                elif y - veh_pos[1] == 0:
                    if dist_ahead:
                        distances[2] = dist_ahead
                        speeds[2] = veh[2][veh_id_ahead[0]][64]  # 64 refers to speed
                        signals[2] = veh[2][veh_id_ahead[0]][91]

                    if dist_behind:
                        distances[3] = abs(dist_behind)
                        speeds[3] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                        signals[3] = veh[2][veh_id_behind[0]][91]

                elif y - veh_pos[1] < 0:
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

    def getBestMergePosition(self, merge_veh, platoon, fisMergeLoc):

        platoon_vehicles = [a for a in platoon._vehicles if abs(a.getPosition()[1] + 4.8) < 1e-3]
        platoon_length = len(platoon_vehicles)
        veh_name = merge_veh._name
        veh_context = merge_veh.getContext()

        veh_all = [0, veh_name, veh_context]

        inputs = self.getInputs2FIS(veh_all)

        all_scores = []
        for j in range(platoon_length + 1):  # Number of gaps in platoon where vehicle can merge

            if j == 0:
                leadPos = [500, 500]
                leadSpeed = 50
                rear = platoon_vehicles[j]

                rearPos = rear.getPosition()
                rearSpeed = rear.getSpeed()


            elif j == platoon_length:
                lead = platoon_vehicles[j - 1]
                leadPos = lead.getPosition()
                leadSpeed = lead.getSpeed()
                rearPos = [500, 500]
                rearSpeed = 50
            else:
                lead = platoon_vehicles[j - 1]
                rear = platoon_vehicles[j]
                leadPos = lead.getPosition()
                leadSpeed = lead.getSpeed()

                rearPos = rear.getPosition()
                rearSpeed = rear.getSpeed()

            full_list2 = inputs[0:4] + inputs[6:10] + inputs[12:16] + [inputs[18]] + [inputs[20]] + [leadPos[0]] + [
                leadSpeed] + [rearPos[0]] + [rearSpeed]  # y-coordinate of ego vehicle is ignored

            score = fisMergeLoc.eval_op(np.array(full_list2))
            all_scores.append(score)

        best_id = all_scores.index(max(all_scores))  # ID of the best position to merge into the platoon
        if best_id == 0:
            leadVeh = -1
            rearVeh = platoon_vehicles[best_id]
        elif best_id == platoon_length:
            leadVeh = platoon_vehicles[best_id - 1]
            rearVeh = -1
        else:
            leadVeh = platoon_vehicles[best_id - 1]
            rearVeh = platoon_vehicles[best_id]

        return leadVeh, rearVeh

    def getDesiredSpeed(self, vehicle, gfs):

        veh_name = vehicle._name
        cur_speed = vehicle.getSpeed()
        veh_context = vehicle.getContext()

        veh_all = [0, veh_name, veh_context]

        inputs = self.getInputs2FIS(veh_all)
        outs = gfs.eval_op(inputs)
        accln = 3.55 * outs[0] - 0.95  # accln in the range[-4.5,2.6]
        des_speed = cur_speed + accln * dt

        return des_speed
