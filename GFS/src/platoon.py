# -*- coding: utf-8 -*-
"""
Created on Tue Dec 1, 2020

@creator: Anoop Sathyan
@authors: Yi Guo, Anoop Sathyan, Jiaqi Ma
"""

import logging
import traci
import random
import pdb

class Platoon():

    def __init__(self, startingVehicles):
        """Create a platoon, setting default values for all variables"""
        #logging.info("Creating a new platoon with: %s", startingVehicles)
        self._vehicles = list(startingVehicles)

        self._active = True
        self._color = (230, 0, 255)
        self._currentSpeed = self.getLeadVehicle().getSpeed()
        self._disbandReason = None
        self._eligibleForMerging = False
        self._lane = self.getLeadVehicle().getLane()
        self._lanePosition = self.getLeadVehicle().getLanePosition()
        self._controlledLanes = set()
        self._targetSpeed = 31.29  # -1

        self.getLeadVehicle().setColor(self._color)
        self.startBehaviour(startingVehicles[1:])

    def addControlledLanes(self, lanes):
        for lane in lanes:
            self._controlledLanes.add(lane)

    def addVehicle(self, vehicle, index):
        """Adds a single vehicle to this platoon at the index provided"""
        self._vehicles.insert(index, vehicle)
        #self._vehicles.append(vehicle)
        self.startBehaviour([vehicle, ])
        # logging.info("Adding %s to platoon %s, New length: %s",
        #              vehicle.getName(), self.getID(), len(self._vehicles))

    def canMerge(self):
        """
        Returns True if this platoon can currently merge with another
        """
        return self._eligibleForMerging

    def checkVehiclePathsConverge(self, vehicles):
        # Check that the given vehicles are going to follow the lead
        # vehicle into the next edge
        leadVehicleRoute = self.getLeadVehicle().getRemainingRoute()
        if len(leadVehicleRoute) > 1:
            leadVehicleNextEdge = leadVehicleRoute[1]
            for vehicle in vehicles:
                if leadVehicleNextEdge not in vehicle.getRoute():
                    return False
        return True

    def disband(self):
        """Marks a platoon as dead and returns vehicles to normal"""
        self.stopBehaviour()
        self._active = False
        #logging.info("Disbanding platoon: %s", self.getID())

    def getAcceleration(self):
        return max([v.getAcceleration() for v in self.getAllVehicles()])

    def getAllVehicles(self):
        """Retrieve the list of all the vehicles in this platoon"""
        return self._vehicles

    def getAllVehiclesByName(self):
        """Retrieve the list of all the vehicles in this platoon by name"""
        return [v.getName() for v in self.getAllVehicles()]

    def getSpeed(self):
        return self.getLeadVehicle().getSpeed()

    def getID(self):
        """Generates and returns a unique ID for this platoon"""
        return "%s" % (self.getLeadVehicle().getName())

    def getLane(self):
        return self._lane

    def getLanesOfAllVehicles(self):
        return [v.getLane() for v in self.getAllVehicles() if v.isActive()]

    def getLanePositionFromFront(self, lane=None):
        if lane:
            vehiclesInLane = [v for v in self.getAllVehicles() if v.getLane() == lane]
            if vehiclesInLane:
                return traci.lane.getLength(lane) - vehiclesInLane[0].getLanePositionFromFront()
        else:
            return traci.lane.getLength(self._lane) - self._lanePosition

    def getLeadVehicle(self):
        return self._vehicles[0]

    def getLength(self):
        """ Gets the total length of the platoon
            Done by taking the distance between the vehicle's front
            bumper and the end of the lane
        """
        laneLen = traci.lane.getLength(self._lane)
        front = laneLen - self.getLeadVehicle().getLanePosition()
        rear = laneLen - self._vehicles[-1].getLanePosition()
        rearVehicleLength = self._vehicles[-1].getLength() * 2
        return rear - front + rearVehicleLength

    def getLengthOfSingleVehicle(self):
        return self.getLeadVehicle().getLength()

    def getMaxSpeed(self):
        """ Gets the maximum speed of the platoon
        """
        return max([v.getMaxSpeed() for v in self.getAllVehicles()])

    def getMergePosition(self, relVeh, vehicle, direction):
        """
        Get the merge position and relevant vehicle(s)
        :param leadVeh: the closet veh's ID in the target platoon
        :param vehicle: the ego vehicle instance
        :return: relevant vehicle(s): frontVeh, rearVeh
        """

        # retrieve all member of the platoon
        frontVeh = -1
        rearVeh = -1
        for idx in range(len(self._vehicles)):
            if relVeh == self._vehicles[idx].getName():
                # get leading vehicle
                if direction == 1:
                    frontVeh = self._vehicles[idx]
                    # invovle a VF in this operation
                    if idx < len(self._vehicles) - 1:
                        # get rear veh ID first
                        rearVeh = self._vehicles[idx+1]
                        # insert the new member
                        self.addVehicle(vehicle, idx+1)
                        return frontVeh, rearVeh
                    else:
                        # insert the new member
                        self.addVehicle(vehicle, idx + 1)
                        return frontVeh, rearVeh
                elif direction == -1:
                    rearVeh = self._vehicles[idx]
                    # invovle a VL in this operation
                    # if the target following veh is a platoon leader
                    if idx == 0:
                        # get rear veh ID first
                        frontVeh = -1

                        # insert the new member
                        # cannot insert the member at idx = 0 since the getLeadVechicle() will be affected.
                        self.addVehicle(vehicle, idx + 1)
                        return frontVeh, rearVeh
                    else:
                        frontVeh = self._vehicles[idx-1]
                        # insert the new member
                        self.addVehicle(vehicle, idx)
                        return frontVeh, rearVeh

    def removeVehicle(self, v):
        if v in self._vehicles:
            self._vehicles.remove(v)


    def getNumberOfVehicles(self):
        return len(self._vehicles)

    def getTargetSpeed(self):
        return self._targetSpeed

    def isActive(self):
        """Is the platoon currently active within the scenario"""
        return self._active


    def setEligibleForMerging(self, canMerge):
        self._eligibleForMerging = canMerge


    def mergePlatoon(self, platoon):
        """Merges the given platoon into the current platoon"""
        if self.checkVehiclePathsConverge(platoon.getAllVehicles()) and platoon.getLane() == self.getLane():
            platoon.disband()
            platoon._disbandReason = "Merged"
            for vehicle in platoon.getAllVehicles():
                self.addVehicle(vehicle)
                # catch up if the tau is larger than the given tau
                vehicle.setSpeedFactor(1.2)
                vehicle.setSpeedMode(30)
        self._eligibleForMerging = False
        platoon._eligibleForMerging = False

    def removeControlledLanes(self, lanes):
        """
        Removes the lanes from the platoon that were previously being controlled by an
        intersection controller.
        """
        for lane in lanes:
            self._controlledLanes.remove(lane)

    def removeTargetSpeed(self):
        """
        Removes the target speed from this platoon
        """
        self._targetSpeed = -1

    def setTargetSpeed(self, speed):
        """
        Sets a manual target speed for this platoon (normally determined by the lead
        vehicle but this will override it). This will remain until removeTargetSpeed
        is called.
        """
        self._targetSpeed = speed

    def setGap(self, gap):
        """
        Set the gap between vehicles in the platoon
        """
        for veh in self.getAllVehicles():
            veh.setTau(gap)

    def setSpeedMode(self, speedMode):
        """
        Set the speed mode for every vehicle in the platoon.
        Speed mode is a SUMO function that allows different behaviour once the setSpeed
        function has been called. Including ignoring safe speed threholds and junctions.
        Used here so that vehicles don't brake unncessarily when closing distances are tight
        during CIM.
        """
        for v in self.getAllVehicles():
            v.setSpeedMode(speedMode)

    def startBehaviour(self, vehicles):
        """A function to start platooning a specific set of vehicles"""
        if self.isActive():
            for v in vehicles:
                # v.setColor(self._color)
                v.setImperfection(0)
                v.setMinGap(0)
                # v.setTau(0.6)
                traci.vehicle.setTau(v.getName(), 0.6)
                # v._state = 'Platooning'
            self.update()

    def stopBehaviour(self):
        """Stops vehicles exhibiting platoon behaviour, if they are
        still present within the map"""
        for v in self._vehicles:
            if v.isActive():
                v.setColor((255, 255, 255))
                # v.setImperfection(0.5) # perfect CAV behavior
                v.setMinGap(0)
                # v.setTau(0.6)
                traci.vehicle.setTau(v.getName(), 0.6)
                # Take speed back to default behaviour
                v.setSpeed(self._targetSpeed)
                v.setSpeedFactor(1.0)
                v.setState('SearchingForPlatooning')

    def updateIsActive(self):
        """
        Is Active Update, if not disband
        """
        if not all([v.isActive() for v in self._vehicles]):
            self._disbandReason = "One vehicle not active"
            self.disband()
            return True

    def update(self):
        """
        Performs updates to maintain the platoon
        1. set platoon location information using lead vehicle
        2. set the speed of all vehicles in the convoy,
           using the lead vehicle's current speed
        3. is this platoon still alive (in the map),
           should it be labelled as inactive?   
        """
        self.updateIsActive()

        if self.isActive():
            potentialNewLeader = self.getLeadVehicle().getLeader()
            if potentialNewLeader and potentialNewLeader[0] in self.getAllVehiclesByName():
                # Something has gone wrong disband the platoon
                self._disbandReason = "Reform required due to new leader"
                self.disband()

            # Location Info Update
            self._lane = self.getLeadVehicle().getLane()
            self._lanePosition = self.getLeadVehicle().getLanePosition()

            # Speed Update
            # leadVehicleSpeed = self.getLeadVehicle().getSpeed()
            # if self._currentSpeed != 0 and leadVehicleSpeed == 0:
            #     self._eligibleForMerging = True
            # self._currentSpeed = leadVehicleSpeed
            # if self._targetSpeed != -1:
            #     self._updateSpeed(self._targetSpeed)
            # else:
            #     if self.getLeadVehicle().getLane() not in self._controlledLanes:
            #         self.getLeadVehicle().setSpeed(-1)
            #     self._updateSpeed(self._currentSpeed, False)

            # Route updates
            # Check that all cars still want to continue onto the
            # next edge, otherwise disband the platoon
            if not self._currentSpeed == 0:
                if not self.checkVehiclePathsConverge(self.getAllVehicles()):
                    self._disbandReason = "Platoon paths now diverge"
                    self.disband()            

    def _updateSpeed(self, speed, inclLeadingVeh=True):
        """ Sets the speed of all vehicles in the platoon
            If inclLeadingVeh set to false, then the leading vehicle is
            excluded from the speed change.
            Also checks that the platoon is bunched together, this allows
            for vehicles to "catch-up"
        """
        if inclLeadingVeh and self.getLeadVehicle().getLane() not in self._controlledLanes:
            self.getLeadVehicle().setSpeed(speed)

        leadVehEdge = self.getLeadVehicle().getEdge()
        targetLane = self.getLeadVehicle().getLaneIndex()

        # Non leading vehicles should follow the speed of the vehicle in front
        vehicles = self._vehicles
        for veh in vehicles:
            try:
                if veh.getEdge() == leadVehEdge:
                    veh.setTargetLane(targetLane)
            except traci.TraCIException:
                logging.error("Could not change lane of %s", veh.getName())
            # Only set the speed if the vehicle is not in a lane controlled by a third party.
            if veh.getLane() not in self._controlledLanes:
                # If we're in range of the leader and they are moving
                # follow thier speed
                # Otherwise follow vehicle speed limit rules to catch up
                leadVeh = veh.getLeader()
                if leadVeh and leadVeh[1] <= 5 and self._currentSpeed != 0:
                    veh.setSpeed(speed)
                else:
                    veh.setSpeed(-1)
