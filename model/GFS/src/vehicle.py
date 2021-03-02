"""
Created on Tue Dec 1, 2020

@creator: Anoop Sathyan
@authors: Yi Guo, Anoop Sathyan, Jiaqi Ma
"""

import traci
import traci.constants as tc
import src.utils as ut
import math

class Vehicle():
    
    def __init__(self, vehicle):
        self._active = True
        self._acceleration = traci.vehicle.getAcceleration(vehicle)
        self._length = traci.vehicle.getLength(vehicle)
        self._maxSpeed = traci.vehicle.getMaxSpeed(vehicle)
        self._name = vehicle
        self._route = traci.vehicle.getRoute(vehicle)
        self._previouslySetValues = dict()
        self._targetLane = 0
        #self.setTau(0.05)
        self.setTau(0.6) # updated commented --> 0.6
        self.setMinGap(0.0)
        self.sensorRange = 150
        self.setSpeedFactor(1.0)
        
        #self.setSpeedMode(0)
        self._state = 'SearchingForPlatooning'
        traci.vehicle.subscribeContext(self._name,tc.CMD_GET_VEHICLE_VARIABLE, self.sensorRange, [tc.VAR_SPEED, tc.VAR_POSITION, tc.VAR_LANE_ID, tc.VAR_LANE_INDEX, tc.VAR_SIGNALS])

    def getAcceleration(self):
        return self._acceleration

    def isActive(self):
        return self._active
    
    def getContext(self):
        self.Context = traci.vehicle.getContextSubscriptionResults(self._name)
        return self.Context

    def getDistance(self, vehicle):
        x1, y1 = traci.vehicle.getPosition(self.getName())
        x2, y2 = traci.vehicle.getPosition(vehicle.getName())
        # v_data = get_par(v1, cc.PAR_SPEED_AND_ACCELERATION)
        # (v, a, u, x1, y1, t) = cc.unpack(v_data)
        # v_data = get_par(v2, cc.PAR_SPEED_AND_ACCELERATION)
        # (v, a, u, x2, y2, t) = cc.unpack(v_data)
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) - 4

    def getEdge(self):
        return traci.vehicle.getRoadID(self.getName())

    def getLane(self):
        return traci.vehicle.getLaneID(self.getName())

    def getLaneIndex(self):
        return traci.vehicle.getLaneIndex(self.getName())

    def getLanePosition(self):
        return traci.vehicle.getLanePosition(self.getName())

    def getLanePositionFromFront(self):
        return traci.lane.getLength(self.getLane()) - self.getLanePosition()

    # change the default range value to 20
    def getLeader(self, range=20):
        return traci.vehicle.getLeader(self.getName(), range)

    def getLeftFollower(self):
        res = traci.vehicle.getLeftFollowers(self.getName())
        if len(res) > 0:
            return res[0]
        else:
            return None

    def getLeftLeader(self):
        res = traci.vehicle.getLeftLeaders(self.getName())
        if len(res) > 0:
            return res[0]
        else:
            return None

    def getRightFollower(self):
        res = traci.vehicle.getRightFollowers(self.getName())
        if len(res) > 0:
            return res[0]
        else:
            return None

    def getRightLeader(self):
        res = traci.vehicle.getRightLeaders(self.getName())
        if len(res) > 0:
            return res[0]
        else:
            return None


    def getLength(self):
        return self._length

    def getMaxSpeed(self):
        return self._maxSpeed

    def getName(self):
        return self._name
    
    def getPosition(self):
        return traci.vehicle.getPosition(self.getName())

    def getRemainingRoute(self):
        return self._route[traci.vehicle.getRouteIndex(self.getName()):]

    def getRoute(self):
        return self._route

    def getSpeed(self):
        return traci.vehicle.getSpeed(self.getName())

    def getState(self):
        return self._state

    def getLatPos(self):
        return traci.vehicle.getLateralLanePosition(self.getName())

    def getTargetLane(self):
        return self._targetLane

    def getTau(self):
        return traci.vehicle.getTau(self.getName())

    def setColor(self, color):
        self._setAttr("setColor", color)

    def setInActive(self):
        self._active = False

    def setImperfection(self, imperfection):
        self._setAttr("setImperfection", imperfection)

    def setMinGap(self, minGap):
        self._setAttr("setMinGap", minGap)

    def setState(self, state):
        self._state = state

    def setTargetLane(self, lane):
        traci.vehicle.changeLane(self.getName(), lane, 0.5)

    def setTau(self, tau):
        self._setAttr("setTau", tau)

    def setSpeed(self, speed):
        self._setAttr("setSpeed", speed)

    def setSpeedMode(self, speedMode):
        self._setAttr("setSpeedMode", speedMode)

    def setSpeedFactor(self, speedFactor):
        self._setAttr("setSpeedFactor", speedFactor)

    # def setTargetLane(self, targetLane):
    #     self._targetLane = targetLane

    def _setAttr(self, attr, arg):
        # Only set an attribute if the value is different from the previous value set
        # This improves performance
        if self.isActive():
            if attr in self._previouslySetValues:
                if self._previouslySetValues[attr] == arg:
                    return
            self._previouslySetValues[attr] = arg
            getattr(traci.vehicle, attr)(self.getName(), arg)

    def changeLane(self, laneID, mode_num):
        # for different platooning scenario
        # cut-in --> default LC
        # front/rear join, Avoid_collision_LC

        if mode_num == 0:
            traci.vehicle.setLaneChangeMode(self.getName(), ut.DEFAULT_LC)
        elif mode_num == 1:
            traci.vehicle.setLaneChangeMode(self.getName(), ut.AVOID_COL_LC)
        traci.vehicle.changeLane(self.getName(), laneID, 10000.0)
    
    def getClosestDistances(self, veh):
    
        Y_ML = [-8.0,-4.8,-1.6]
        veh_name = veh[1]
        veh_edge = veh[2][veh_name][81]
        veh_speed = veh[2][veh_name][64]  # 66 is the key for position
        veh_pos = veh[2][veh_name][66]  # 66 is the key for position
        
        all_vehs_around = list(veh[2].keys()) # This also includes the current vehicle. So, this is removed in next line
        all_vehs_around.remove(veh_name)
        
        distances = [self.sensorRange]*6  # [left ahead, left behind, same lane ahead, same lane behind, right ahead, right behind]
        speeds = [50]*6
        signals = [0]*6 # signal =0 either means no signal or no vehicle or no lane
        
        vehIDs = [self.sensorRange]*6
        
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
                vehIDs[2] = veh_id_ahead
                
            if dist_behind:
                distances[3] = abs(dist_behind)
                speeds[3] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                signals[3] = veh[2][veh_id_behind[0]][91]
                vehIDs[3] = veh_id_behind
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
                        vehIDs[0] = veh_id_ahead
                        
                    if dist_behind:
                        distances[1] = abs(dist_behind)
                        speeds[1] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                        signals[1] = veh[2][veh_id_behind[0]][91]
                        vehIDs[1] = veh_id_behind
                        
                elif y-veh_pos[1] == 0:
                    if dist_ahead:
                        distances[2] = dist_ahead
                        speeds[2] = veh[2][veh_id_ahead[0]][64]  # 64 refers to speed
                        signals[2] = veh[2][veh_id_ahead[0]][91]
                        vehIDs[2] = veh_id_ahead
                        
                    if dist_behind:
                        distances[3] = abs(dist_behind)
                        speeds[3] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                        signals[3] = veh[2][veh_id_behind[0]][91]
                        vehIDs[3] = veh_id_behind
                        
                elif y-veh_pos[1] < 0:
                    if dist_ahead:
                        distances[4] = dist_ahead
                        speeds[4] = veh[2][veh_id_ahead[0]][64]  # 64 refers to speed
                        signals[4] = veh[2][veh_id_ahead[0]][91]
                        vehIDs[4] = veh_id_ahead
                        
                    if dist_behind:
                        distances[5] = abs(dist_behind)
                        speeds[5] = veh[2][veh_id_behind[0]][64]  # 64 refers to speed
                        signals[5] = veh[2][veh_id_behind[0]][91]
                        vehIDs[5] = veh_id_behind
        # The below if statements make the distances and speeds of lanes that do not exist around each vehicle to 0
        if veh_edge == 'gneE1_0' or veh_edge == ':gneJ1_0_0':
            distances[0:2] = [a if a < self.sensorRange else -1 for a in distances[0:2]]
            distances[4:] = [a if a < self.sensorRange else -1 for a in distances[4:]]
            speeds[0:2] = [a if a < 50 else -1 for a in speeds[0:2]]
            speeds[4:] = [a if a < 50 else -1 for a in speeds[4:]]
        elif veh_edge == 'gneE0_0' or veh_edge == 'gneE4_0' or veh_edge == 'gneE5_0' or veh_edge == ':gneJ6_0_1':
            distances[4:] = [a if a < self.sensorRange else -1 for a in distances[4:]]
            speeds[4:] = [a if a < 50 else -1 for a in speeds[4:]]
        elif veh_edge == 'gneE0_1' or veh_edge == 'gneE4_2' or veh_edge == 'gneE5_1' or veh_edge == ':gneJ1_1_1' or veh_edge == ':gneJ6_0_2':
            distances[0:2] = [a if a < self.sensorRange else -1 for a in distances[0:2]]
            speeds[0:2] = [a if a < 50 else -1 for a in speeds[0:2]]
        allInputs = distances + speeds + signals + list(veh_pos) + [veh_speed]
        return allInputs, vehIDs