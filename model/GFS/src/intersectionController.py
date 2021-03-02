import traci
import logging
from src.simlib import flatten

class IntersectionController():

    def __init__(self, intersection, zip=True):
        lanes = traci.trafficlights.getControlledLanes(intersection)
        self.lanesServed = set(lanes)
        self.name = intersection
        self.platoons = []
        self.platoonsZipped = set()
        self.platoonZips = []
        self.zip = zip

    def addPlatoon(self, platoon):
        """
        Adds a platoon to this intersection controller
        """
        self.platoons.append(platoon)
        if self.zip:
            platoon.addControlledLanes(self.lanesServed)

    def calculateNewReservedTime(self, pv, reservedTime):
        """
        Calculates the time that is needed to be reserved for a given platoon or vehicle (pv)
        """
        # If this platoon is the first to post a reservation, the distance to the junction needs to be included
        if reservedTime == 0:
            lenThruJunc = self._getLanePosition(pv) + pv.getLength()
        else:
            lenThruJunc = pv.getLength() * 2 if self.zip else 1
        return (0.5 if self.zip else 1.5) + reservedTime + (lenThruJunc / (pv.getSpeed() or 1))

    def _eligibleZippings(self, platoon):
        if len(self.platoonZips) >= 1:
            z = self.platoonZips[-1]
            if platoon.getLanePositionFromFront() - z[-1].getLanePositionFromFront() < 10 and platoon.getLanePositionFromFront() - z[-1].getLanePositionFromFront() > 0:
                return z

    def removeIrreleventPlatoons(self):
        """
        Function to remove any platoons from the intersection that have either left the sphere of influence or left the map
        """
        # Check if we need to remove any before adding new ones to the controller
        for p in self.platoons:
            if not p.isActive() or all([l not in self.lanesServed for l in p.getLanesOfAllVehicles()]):
                self.removePlatoon(p)
                if self.zip:
                    for zip in self.platoonZips:
                        for platoon in zip:
                            if platoon == p:
                                zip.remove(platoon)
                        if not zip:
                            self.platoonZips.remove(zip)

    def findAndAddReleventPlatoons(self, platoons):
        """
        Finds platoons in the given list that can be managed by this controller, then
        adds them
        """
        def platoonPosition(platoon):
            return self._getLanePosition(platoon)

        platoons.sort(key=platoonPosition)
        for p in platoons:
            if p.getLane() in self.lanesServed and p not in self.platoons:
                self.addPlatoon(p)

    def getVehicleZipOrderThroughJunc(self):
        """
        Gets the order that a platoon should [pass through the junction if zipping is enabled
        """
        def distSort(elem):
            return elem.getLanePositionFromFront()

        if self.zip:
            prelimList = flatten([self._zipPlatoons(z) for z in self.platoonZips])
            prelimList.sort(key=distSort)
            return prelimList

    def _generatePlatoonZips(self):
        """
        Generates all the zips for the platoons in the scenario
        """
        for p in self.platoons:
            if p not in self.platoonsZipped:
                eligibleZipping = self._eligibleZippings(p)
                if eligibleZipping:
                    eligibleZipping.append(p)
                else:
                    self.platoonZips.append([p])
                self.platoonsZipped.add(p)

    def _getLanePosition(self, v):
        """
        Gets a platoon's lane position in relation to this intersection
        (gives 0 if the platoon is on an edge not controlled by this controller)
        """
        if v.isActive():
            if v.getLane() in self.lanesServed:
                return v.getLanePositionFromFront()
        return 1000

    def getNewSpeed(self, pv, reservedTime):
        """
        Gets the speed the platoon or vehicle should adhere to in order to pass through the intersection safely
        """
        distanceToTravel = self._getLanePosition(pv)
        currentSpeed = pv.getSpeed()
        # If we are in the last 20 metres, we assume no more vehicles will join the platoon
        # and then set the speed to be constant. This is because if we did not speed tends
        # towards 0 (as the distance we give is to the junction and not to the end of the platoon's
        # route.
        if distanceToTravel > 20:
            pv.setSpeedMode(23)
            speed = distanceToTravel / (reservedTime or 1)
            speed = max([speed, 0.5])
            if speed >= currentSpeed:
                speed = pv.getMaxSpeed()
        elif currentSpeed == 0:
            speed = pv.getMaxSpeed()
        else:
            return pv.getMaxSpeed()
        if reservedTime == 0:
            return pv.getMaxSpeed()
        return speed

    def removePlatoon(self, platoon):
        """
        Removes a platoon from this controller and then resets its behaviour to default
        """
        self.platoons.remove(platoon)
        # Resume normal speed behaviour
        platoon.removeTargetSpeed()
        platoon.setSpeedMode(31)
        if self.zip:
            platoon.removeControlledLanes(self.lanesServed)

    def update(self):
        """
        Performs various functions to update the junction's state.
        1. Ensures that all vehicles being managed by the junction, have thier automatic
           stopping behaviour deactived (otherwise they are too cautious at the intersection)
        2. Removes platoons that are no longer in the sphere of influence of the function
        3. Updates the speed of all platoons being managed by the controller.
        """
        reservedTime = 0
        if self.zip:
            self._generatePlatoonZips()
            for v in self.getVehicleZipOrderThroughJunc():
                if v.isActive() and v.getLane() in self.lanesServed:
                    speed = self.getNewSpeed(v, reservedTime)
                    v.setSpeed(speed)
                    reservedTime = self.calculateNewReservedTime(v, reservedTime)
        else:
            for p in self.platoons:
                # Update the speeds of the platoon if it has not passed the junction
                if p.getLane() in self.lanesServed:
                    speed = self.getNewSpeed(p, reservedTime)
                    if speed == -1:
                        p.removeTargetSpeed()
                    else:
                        p.setTargetSpeed(speed)
                    reservedTime = self.calculateNewReservedTime(p, reservedTime)
        self._logIntersectionStatus(reservedTime)

    def _logIntersectionStatus(self, reservation=None):
        """
        A function that logs the status of this intersection.
        """
        if self.platoons:
            logging.info("------------%s Information------------", self.name)
            if self.zip:
                for v in self.getVehicleZipOrderThroughJunc():
                    if v.isActive():
                        setSpeed = v._previouslySetValues['setSpeed'] if 'setSpeed' in v._previouslySetValues else "None"
                        logging.info("Vehicle: %s, Target: %s, Current: %s", v.getName(), setSpeed, v.getSpeed())
                logging.info("------------Platoon Zips------------")
                for zip in self.platoonZips:
                    logging.info("Zip: %s", [p.getID() for p in zip])
            else:
                for p in self.platoons:
                    logging.info("Platoon: %s, Target: %s, Current: %s ", p.getID(), p.getTargetSpeed(), p.getSpeed())
            if reservation:
                logging.info("Total time reserved: %s", reservation)

    def _zipPlatoons(self, platoons):
        """
        Zips all the vehicles in the given platoons into one continuous set
        """
        ret = []
        iterations = max([len(p.getAllVehicles()) for p in platoons])
        for i in range(0, iterations):
            for p in platoons:
                if len(p.getAllVehicles()) > i:
                    if p.getAllVehicles()[i].getLane() in self.lanesServed:
                        ret.append(p.getAllVehicles()[i])
        return ret