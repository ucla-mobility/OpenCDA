from src.intersectionController import IntersectionController
from src.platoon import Platoon
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
SPEED = 31.29  # 130 / 3.6

# Speed Mode
SPD_ALL_CHECK = 31
SPD_DIS_SAFE = 30

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
        # # add the direction
        # direction = 0
        # if leadVeh is None or leadVeh[1] < 50:
        #     leadVeh = vehicle.getLeftLeader()
        #     direction = 1
        #     if leadVeh is None or leadVeh[1] < 50:
        #         leadVeh = vehicle.getRightLeader()
        #         direction = -1

        if leadVeh and leadVeh[1] < 50:  # and direction == 0
            leadIns = self.vehicles[leadVeh[0]]
            possiblePlatoon = self.getPlatoonByVehicle(leadIns)
            if possiblePlatoon:
                if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:  # Dec 30, 2020 --> Max_Platoon_Length
                    # ensure the platoon is not engaging another operation
                    # the leader state could be Leading or SearchForPlatooning
                    if possiblePlatoon[0].getLeadVehicle().getState() == "Leading" or possiblePlatoon[
                        0].getLeadVehicle().getState() == "SearchingForPlatooning":
                        if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in possiblePlatoon[
                            0].getAllVehicles():
                            return possiblePlatoon[0], 0, leadVeh
        # potential left preceding platoon
        if leftLeadVeh and leftLeadVeh[1] < 50:  # and direction == 0
            leadIns = self.vehicles[leftLeadVeh[0]]
            possiblePlatoon = self.getPlatoonByVehicle(leadIns)
            if possiblePlatoon:
                if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:  # Dec 30, 2020 --> Max_Platoon_Length
                    # ensure the platoon is not engaging another operation
                    # the leader state could be Leading or SearchForPlatooning
                    if possiblePlatoon[0].getLeadVehicle().getState() == "Leading" or possiblePlatoon[
                        0].getLeadVehicle().getState() == "SearchingForPlatooning":
                        if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in possiblePlatoon[
                            0].getAllVehicles():
                            return possiblePlatoon[0], 1, leftLeadVeh

        # potential right preceding platoon
        if rightLeadVeh and rightLeadVeh[1] < 50:  # and direction == 0
            leadIns = self.vehicles[rightLeadVeh[0]]
            possiblePlatoon = self.getPlatoonByVehicle(leadIns)
            if possiblePlatoon:
                if len(possiblePlatoon[0]._vehicles) < Max_Platoon_Length:  # Dec 30, 2020 --> Max_Platoon_Length
                    # ensure the platoon is not engaging another operation
                    # the leader state could be Leading or SearchForPlatooning
                    if possiblePlatoon[0].getLeadVehicle().getState() == "Leading" or possiblePlatoon[
                        0].getLeadVehicle().getState() == "SearchingForPlatooning":
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
                    if possiblePlatoon[0].getLeadVehicle().getState() == "Leading" or possiblePlatoon[
                        0].getLeadVehicle().getState() == "SearchingForPlatooning":
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
                    if possiblePlatoon[0].getLeadVehicle().getState() == "Leading" or possiblePlatoon[
                        0].getLeadVehicle().getState() == "SearchingForPlatooning":
                        if possiblePlatoon[0].checkVehiclePathsConverge([vehicle]) and vehicle not in \
                                possiblePlatoon[0].getAllVehicles():
                            return possiblePlatoon[0], -1, rightFollowVeh
        # added the default return value
        return None, -100, leadVeh

    def checkPlatoonsNearbyAndUpdate(self, platoon):

        vehicleInplatoon = platoon.getAllVehicles()
        if len(vehicleInplatoon) == 1:
            vehicle = vehicleInplatoon[0]
        else:
            vehicles = [v for v in vehicleInplatoon if v._state == 'OpenGap']
            vehicle = vehicles[0]
        veh_context = vehicle.getContext()

        veh_all = [0.0, vehicle._name, veh_context]  # The first argument to getClosestDistances is time.

        if vehicle._state == 'SearchingForPlatooning':
            [DistSpeedsSignals, closestSurroundVehs] = vehicle.getClosestDistances(veh_all)
            RelevantDists = [a for a in DistSpeedsSignals[0:6:2] if
                             -1 < a < vehicle.sensorRange]  # Only even indices of distances as they are the forward vehicles
            RelevantVehs = [closestSurroundVehs[i] for i in [0, 2, 4] if
                            -1 < DistSpeedsSignals[i] < vehicle.sensorRange]  # 0,2,4 are indices of forward vehicles

            pos_platoons = [self.getPlatoonByVehicle(a[0])[0] if self.getPlatoonByVehicle(a[0]) else 100 for a in
                            RelevantVehs]  # Possible platoons
            RelevantLeaders = [b.getLeadVehicle()._name for b in pos_platoons if b != 100]
            if RelevantLeaders:
                vehicle._state = 'InitPlatoonAgmtSeeking'  ## Change state of the vehicle

        if vehicle._state == 'InitPlatoonAgmtSeeking':  ## Find best platoon to join
            sort_idx = np.argsort(np.array(RelevantDists))
            for sort_id in sort_idx:
                speedDiff = abs(Vehicle(RelevantVehs[sort_id][0]).getSpeed() - vehicle.getSpeed())
                VehsInPlatoon = pos_platoons[sort_id].getAllVehicles()
                NamesOfVehsInPlatoon = [a._name for a in VehsInPlatoon]

                NumVehsInPlatoon = len(VehsInPlatoon)  # Max set to 5 below
                if RelevantDists[sort_id] < 100 and speedDiff < 4 and NumVehsInPlatoon < Max_Platoon_Length:
                    vehicle._state = 'PrepareForPlatooning'
                    vehicle._closestPlatoon = pos_platoons[sort_id]
                    vehicle._IDofClosestVehicleInPlatoon = NamesOfVehsInPlatoon.index(
                        RelevantVehs[sort_id][0])  # Index of closest vehicle in platoon
                    break
            if vehicle._state != 'PrepareForPlatooning':  # If vehicle state has not switched, then switch back to searching mode
                vehicle._state = 'SearchingForPlatooning'

        if vehicle._state == 'PrepareForPlatooning':
            idxClosestVehInPlatoon = vehicle._IDofClosestVehicleInPlatoon  # Index of closest vehicle in platoon
            closestPlatoon = vehicle._closestPlatoon
            VehsInPlatoon = closestPlatoon.getAllVehicles()

            NumVehsInPlatoon = len(VehsInPlatoon)

            cur_lane = vehicle.getLane()
            vehX, vehY = vehicle.getPosition()
            closVehX, closVehY = VehsInPlatoon[idxClosestVehInPlatoon].getPosition()
            targetLane = vehicle.getLaneIndex() + int(np.sign(closVehY - vehY))
            if cur_lane[4] == '0' or cur_lane[4] == '5':
                targetLane = min(max(targetLane, 0), 1)  # Only 2 lanes in gneE0 and gneE5
            elif cur_lane[4] == '4':
                targetLane = min(max(targetLane, 0), 2)  # 3 lanes in gneE4
            elif cur_lane[4] == '1':
                targetLane = 0  # Only 1 lanes in gneE1 which is the merge
            else:
                targetLane = vehicle.getLaneIndex()

            if idxClosestVehInPlatoon == NumVehsInPlatoon - 1:  # Closest vehicle is last vehicle in platoon
                # pdb.set_trace()
                nn = 100
                # vehicle.setTargetLane(targetLane)
            else:
                CooperatingVeh = VehsInPlatoon[idxClosestVehInPlatoon + 1]
                CooperatingVeh._state = 'OpenGap'
                CooperatingVeh._joinVeh = vehicle

                vehicle.setSpeed(VehsInPlatoon[0].getSpeed() * 1.2)  # accelerate
                if abs(vehicle.getSpeed() - VehsInPlatoon[0].getSpeed()) < 0.5:
                    vehicle._state = 'MergeIntoPlatoon'
                    vehicle._targetLane = targetLane

        if vehicle._state == 'MergeIntoPlatoon':
            idxClosestVehInPlatoon = vehicle._IDofClosestVehicleInPlatoon
            closestPlatoon = vehicle._closestPlatoon
            VehsInPlatoon = closestPlatoon.getAllVehicles()

            desiredSpeed = 0.5 * (VehsInPlatoon[idxClosestVehInPlatoon].getSpeed() + VehsInPlatoon[
                idxClosestVehInPlatoon + 1].getSpeed())  # Mean of the two vehicles
            vehicle.setSpeed(desiredSpeed)
            vehicle.setTargetLane(vehicle._targetLane)

            if vehicle.getLaneIndex() == vehicle._targetLane:  # Merged into new platoon

                platoon.disband()

                vehicle._state = 'Platooning'
                # vehicle.setSpeed(VehsInPlatoon[0].getSpeed())
                VehsInPlatoon[idxClosestVehInPlatoon + 1]._state = 'Platooning'
                # VehsInPlatoon[idxClosestVehInPlatoon+1].setSpeed(VehsInPlatoon[0].getSpeed())
                vehicle._closestPlatoon.addVehicle(vehicle, idxClosestVehInPlatoon + 1)

        if vehicle._state == 'OpenGap':
            if vehicle._joinVeh._state == 'MergeIntoPlatoon' or vehicle._joinVeh._state == 'PrepareForPlatooning':
                vehicle.setSpeed(vehicle.getSpeed() - 0.5)
            else:
                vehicle._state == 'Platooning'
                vehicle.setSpeed(SPEED)
            # pdb.set_trace()

            # vehicle.setSpeed(VehsInPlatoon[0].getSpeed())

    def handleSimulationStep(self):
        allVehicles = traci.vehicle.getIDList()
        # Check mark vehicles as in-active if they are outside the map
        stoppedCount = dict()
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

            # Get information concerning the number of vehicles queueing on each lane
            # if v.isActive() and v.getSpeed() == 0:
            #     lane = v.getEdge()
            #     if lane in stoppedCount:
            #         stoppedCount[lane] = stoppedCount[lane] + 1
            #     else:
            #         stoppedCount[lane] = 1

            # # vehicle needs to follow the given tau (desired tau)
            # # if current tau is larger than the des_tau
            # # speed-up to catch up with preceding vehicle
            # dis = v.getLeader(150) # return ID and dis of leading vehicle within 150 meters
            # spd = v.getSpeed()
            # des_tau = v.getTau()
            # if dis is not None and dis[1]/spd > des_tau + 0.05:
            #     # v.setSpeedFactor(1.2)
            #     v.setSpeed(SPEED*1.2)
            #     v.setSpeedMode(0)
            # else:
            #     # v.setSpeedFactor(1.0)
            #     v.setSpeed(SPEED)
            #     v.setSpeedMode(SPD_ALL_CHECK)

        # Gather statistics for amount of vehicles stopped per lane
        # for lane in stoppedCount:
        #     if lane in self.maxStoppedVehicles:
        #         if stoppedCount[lane] > self.maxStoppedVehicles[lane]:
        #             self.maxStoppedVehicles[lane] = stoppedCount[lane]
        #     else:
        #         self.maxStoppedVehicles[lane] = stoppedCount[lane]

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
                if self.getVehicleType(vehicleID) == HDV_TYPE:
                    vehicle = Vehicle(vehicleID)
                    vehicle.setState("CDADisabled")
                    vehicle.setColor((255, 255, 255))
                    self.addNewVehicle(vehicleID, vehicle)
                    continue
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
                # vehicleLane = vehicle.getLane()
                # vehicle.setColor((255, 255, 255))
                # traci.vehicle.setColor(vehicle.getName(), (230, 0, 255))
                # If we're not in a starting segment (speed starts as 0)

                if not vehicle.isActive():
                    continue

                possiblePlatoon, direction, leadVeh = self.getReleventPlatoon(vehicle)
                if possiblePlatoon:  # and direction == 0

                    curPlatoon = self.getPlatoonByVehicle(vehicle)
                    curX, curY = vehicle.getPosition()
                    # same lane join
                    if direction == 0:
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
                    elif curX > -2300.00:
                        # also add to the platoon list
                        # v.state --> swtich to "PreparingForPlatooning"
                        # get relevant vehicles
                        #   dis to leader
                        #       beyond the leader, front join
                        #       behind the last member, back join
                        #       otherwise, given two relevant members
                        # form a context to bind them
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
                        v.setSpeed(SPEED)
                        v.setColor((230, 0, 255))
                        traci.vehicle.setLaneChangeMode(v.getName(),
                                                        0b000000000000)  # to-do: encap, lane-change prohibit mode
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
                        elif dis is not None and dis[1] / (spd + 0.00001) <= INTRA_TAU + 0.01:
                            # v.setSpeedFactor(1.0)
                            v.setSpeed(SPEED)
                            v.setSpeedMode(SPD_ALL_CHECK)
                    elif v.getState() == "PreparingForPlatooning":
                        v.setColor((255, 165, 0))
                        if v.getName() in self.mergelist.keys():
                            frontVeh, rearVeh = self.mergelist[v.getName()]
                            if frontVeh != -1:
                                dis = v.getDistance(frontVeh)
                                frontSpd = frontVeh.getSpeed()  # use preceding veh's speed
                                if dis > frontSpd * (INTRA_TAU + 0.2):
                                    v.setSpeed(frontSpd * 1.1)
                                    v.setSpeedMode(SPD_ALL_CHECK)
                                elif dis < LENGTH:
                                    v.setSpeed(frontSpd * 0.95)
                                    v.setSpeedMode(SPD_ALL_CHECK)
                                else:
                                    v.setSpeed(frontSpd)
                                    v.setSpeedMode(SPD_ALL_CHECK)
                                    v.setState("InPosition")
                                    if rearVeh != -1:
                                        rearVeh.setState("OpeningGap")
                            elif frontVeh == -1:  # must have rearVeh
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
                                    v.setSpeed(SPEED * 1.1)
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
                        if frontVeh != -1:
                            dis = v.getDistance(frontVeh)
                            frontSpd = frontVeh.getSpeed()  # use preceding veh's speed
                            if dis > frontSpd * (INTRA_TAU + 0.2):
                                v.setSpeed(frontSpd * 1.1)
                                v.setSpeedMode(SPD_ALL_CHECK)
                            elif dis < LENGTH:
                                v.setSpeed(frontSpd * 0.95)
                                v.setSpeedMode(SPD_ALL_CHECK)
                            else:
                                v.setSpeed(frontSpd)
                                v.setSpeedMode(SPD_ALL_CHECK)

                            if frontEdge != -100 and egoEdge == frontEdge:
                                if egoLane == frontVeh.getLaneIndex():
                                    v.setState("Following")
                                    if rearEdge != -100:
                                        rearVeh.setState("Following")
                                    curPlatoon = self.getPlatoonByVehicle(v)
                                    # end this operation session and allow another operation
                                    curPlatoon[0].getLeadVehicle().setState("Leading")
                            elif rearEdge != -100 and egoEdge == rearEdge:
                                if egoLane == rearVeh.getLaneIndex():
                                    v.setState("Following")
                                    rearVeh.setState("Following")
                                    curPlatoon = self.getPlatoonByVehicle(v)
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
                                v.setSpeed(frontSpd * 1.1)
                                v.setSpeedMode(SPD_ALL_CHECK)
                            if rearEdge != -100 and egoEdge == rearEdge:
                                if egoLane == rearVeh.getLaneIndex():
                                    v.setState("Leading")
                                    curPlt = self.getPlatoonByVehicle(v)[0]
                                    curPlt.getAllVehicles().pop(1)  # remove v from list
                                    curPlt.addVehicle(v, 0)
                                    # end this operation session and allow another operation
                                    rearVeh.setState("Following")
                    elif v.getState() == "OpeningGap":
                        v.setColor((0, 0, 255))
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
                                    v.setTargetLane(1)
                                else:
                                    v.setTargetLane(0)
                        else:
                            # the dissolving veh is platoon leader
                            v.setState('Dissolving')
                            # to-do: target lane should be the lane along the new route
                            if v.getLaneIndex == 0:
                                v.setTargetLane(1)
                            else:
                                v.setTargetLane(0)
                    elif v.getState() == 'PreParingForDissolve':
                        v.setSpeed(SPEED * 1.1)
                        v.setSpeedMode(SPD_ALL_CHECK)
                    elif v.getState() == 'MaintainDissolveGap':  # state only available for platoon leader
                        v.setSpeed(SPEED)
                        v.setSpeedMode(SPD_ALL_CHECK)
                    elif v.getState() == 'Dissolving':
                        v.setColor((200, 66, 245))
                        tarLaneID = v.getTargetLane()
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

                # # only one vehicle in the platoon
                # else:
                #     platoon.setEligibleForMerging(True)

                # if platoon.canMerge() and platoon.isActive():
                #     lead = platoon.getLeadVehicle().getLeader()
                #     if lead and lead[1] < 60:
                #         leadPlatoon = self.getPlatoonByVehicle(lead[0])
                #         if leadPlatoon:
                #             leadPlatoon[0].mergePlatoon(platoon)

            # platoons = self.getActivePlatoons()
            # platoonsWithLengthOne = [a for a in platoons if len(a.getAllVehicles()) ==1] # Platoons with only one vehicle
            #
            # for plat in platoonsWithLengthOne:
            #     leader = plat.getLeadVehicle()
            #     leader.setState("SearchingForPlatooning") # instead of platooning for single vehicle platoon

            # vehs = plat.getAllVehicles()
            # checkOpenGap = [v for v in vehs if v._state == 'OpenGap']
            #
            # if len(vehs) == 1 or len(checkOpenGap) > 0:
            #     self.checkPlatoonsNearbyAndUpdate(plat)

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