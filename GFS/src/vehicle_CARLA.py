"""
Created on Tue Dec 1, 2020

@creator: Anoop Sathyan
@authors: Yi Guo, Anoop Sathyan, Jiaqi Ma
"""

import src.utils as ut
import math
import random
import numpy as np
import carla

'''
    1. control functions need to implement in CARLA
    # --------------- set target lane --------------------
    # need to address how to implement this in CARLA 
    # def setTargetLane(self, lane):
    #     traci.vehicle.changeLane(self.getName(), lane, 0.1)
    # ----------------------- change lane -----------------------------
    # need to implement this in CARLA
    # def changeLane(self, laneID):
    #     traci.vehicle.setLaneChangeMode(self.getName(), ut.DEFAULT_LC)
    #     traci.vehicle.changeLane(self.getName(), laneID, 0.1)
    # --------------- set speed --------------------
    # need to implement this in CARLA 
    # def setSpeed(self, speed):
    #     self._setAttr("setSpeed", speed)
    # need to implement this in CARLA
    # def setSpeedMode(self, speedMode):
    #     self._setAttr("setSpeedMode", speedMode)
    # -------------- set multiple attributes -----------------
    # not used in CARLA, change to set attributes individually (originally used to improve SUMO efficiency)
    # def _setAttr(self, attr, arg):
    #     # Only set an attribute if the value is different from the previous value set
    #     # This improves performance
    #     if self.isActive():
    #         if attr in self._previouslySetValues:
    #             if self._previouslySetValues[attr] == arg:
    #                 return
    #         self._previouslySetValues[attr] = arg
    #         getattr(traci.vehicle, attr)(self.getName(), arg)
    # ------------- change color -----------------
    # need to verify later, carla not allow changing color on the go, color need to be set when spawn
    # def setColor(self, color):
    #     self._setAttr("setColor", color)
    # ------------ driving imperfection factor -------------
    # not used by Carla, also disabled in platoon class
    # def setImperfection(self, imperfection):
    #     self._setAttr("setImperfection", imperfection)
    
'''

class Vehicle(object):
    
    def __init__(self, vehicle):
        self._vehicle = vehicle 
        self._active = True
        self._acceleration = vehicle.get_acceleration() # traci.vehicle.getAcceleration(vehicle)

        boundingbox = vehicle.bounding_box
        self._width = 2 * abs(boundingbox.location.y - boundingbox.extent.y)
        self._length = 2 * abs(boundingbox.location.x - boundingbox.extent.x) # traci.vehicle.getLength(vehicle)
        self._maxSpeed = vehicle.get_speed_limit()# traci.vehicle.getMaxSpeed(vehicle)
        self._name = vehicle.id
        self._start_pos = (self._vehicle.get_location().x, self._vehicle.get_location().y)
        # determine route based on initial position (y>6 main, else merge)
        self._route = ('gneE0', 'gneE4', 'gneE5') if self._start_pos[1] > -6 else ('gneE1', 'gneE4', 'gneE5')
        self._previouslySetValues = dict()
        self._targetLane = 0
        # ! creat sumo attributes for carla 
        #self.set_Tau(0.05)
        self._sensorRange = 150
        self._Tau = 0.6 # self.set_Tau(0.6) # updated commented --> 0.6
        self._MinGap = 0.0 # self.set_MinGap(0.0) 
        self._SpeedFactor = 1.0 # self.set_SpeedFactor(1.0)
        # carla attributes
        self._world = vehicle.get_world()
        self._map = vehicle.get_world().get_map()
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        # set a place holder for signal == 0
        self._signal = 0

        #self.setSpeedMode(0)
        self._state = 'SearchingForPlatooning'

        # need to implement this in CARLA, GFS uses this input to get all listed info as a dict
        # !!! need to change this in CARLA
        # traci.vehicle.subscribeContext(self._name,tc.CMD_GET_VEHICLE_VARIABLE, self._sensorRange, [tc.VAR_SPEED, tc.VAR_POSITION, tc.VAR_LANE_ID, tc.VAR_LANE_INDEX, tc.VAR_SIGNALS])

    # !!! need to change this in CARLA -->
    def getContext(self):
        '''
        read {'v_id': speed(64), position(66), lane_ID(81), lane_index(82), signals(91)} data for all vehs within 150m
        data structure: float, (float, float), 'edge_lane', int, int
        :return: context
        '''
        context = {}
        vehicles = self._world.get_actors().filter('vehicle.*')
        x,y = self._vehicle.get_location().x, self._vehicle.get_location().x
        for v in vehicles:
            x1, y1 = v.get_location().x, v.get_location().y
            distance = math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
            if distance <= self._sensorRange:
                # sense within range
                vID = str(v.id)
                speed = math.sqrt(v.get_velocity().x**2 + v.get_velocity().y**2 + v.get_velocity().z**2)
                position = (v.get_location().x, v.get_location().y)
                lane = Vehicle(v).getLane()
                lane_index = Vehicle(v).getLaneIndex()
                # !!! need to check with Anoop what does signal do !!!
                signal = v.get_light_state()
                if signal == 'NONE':
                    # no light
                    signal_bin = 0b0000
                elif signal == 'RightBlinker':
                    # right signal
                    signal_bin = 0b0001
                elif signal == 'LeftBlinker':
                    # left signal
                    signal_bin = 0b0010
                elif signal == 'Brake':
                    # brake light
                    signal_bin = 0b1000
                else:
                    # others (not relevent for GFS, mark as 0)
                    signal_bin = 0b0000

                # append to context dictionary
                context[vID] = {64:speed, 66:position, 81:lane, 82:lane_index, 91:signal_bin}
        # self.Context = traci.vehicle.getContextSubscriptionResults(self._name)
        return context

    # getters
    def getLength(self):
        return self._length

    def getMaxSpeed(self):
        return self._maxSpeed

    def getName(self):
        return self._name

    def getPosition(self):
        x, y = self._vehicle.get_location().x, self._vehicle.get_location().y
        return (x, y)
        # return traci.vehicle.getPosition(self.getName())

    def getLaneLength(self, lane):
        # there is no CARLA function to check lane length, need to hardcode for the map
        # lane: roadID_laneID
        if "gneE0" in lane:
            # upstream (sumo: 0,1 --> carla: -2,-1)
            laneLength = 1200.0
        elif 'gneE1' in lane:
            # merge lane (sumo: 0 --> carla: -1)
            laneLength = 1281.0
        elif 'gneE4' in lane:
            # merging area (sumo: 0,1,2 --> carla: -3,-2,-1)
            laneLength = 418.5
        elif 'gneE5' in lane:
            # down stream (sumo: 0,1 --> carla: -2,-1)
            laneLength = 1200.0
        else:
            # extra edge to block acceleration lane (sumo: 0 --> carla: -1)
            laneLength = 1.0
        return laneLength

    def getAcceleration(self):
        return self._acceleration

    def isActive(self):
        return self._active

    def getSignal(self):
        return self._signal

    def getDistance(self, vehicle):
        x1, y1 = self._vehicle.get_location().x, self._vehicle.get_location().y #traci.vehicle.getPosition(self.getName())
        x2, y2 = vehicle.getPosition() #traci.vehicle.getPosition(vehicle.getName())
        # v_data = get_par(v1, cc.PAR_SPEED_AND_ACCELERATION)
        # (v, a, u, x1, y1, t) = cc.unpack(v_data)
        # v_data = get_par(v2, cc.PAR_SPEED_AND_ACCELERATION)
        # (v, a, u, x2, y2, t) = cc.unpack(v_data)
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def getEdge(self):
        # use identical roadID in OpenDrive
        return self._current_waypoint.road_id
        # return traci.vehicle.getRoadID(self.getName())

    def getLane(self):
        # carla: -1,-2,-3 vs SUMO: gneE0_0,gneE0_1,gneE0_2
        carla_ln = self._current_waypoint.lane_id
        carla_edge = self._current_waypoint.road_id
        if carla_edge == 'gneE0':
            # upstream (sumo: 0,1 --> carla: -2,-1)
            output_ln = carla_ln + 2
        elif carla_edge == 'gneE1':
            # merge lane (sumo: 0 --> carla: -1)
            output_ln = carla_ln + 1
        elif carla_edge == 'gneE4':
            # merging area (sumo: 0,1,2 --> carla: -3,-2,-1)
            output_ln = carla_ln + 3
        elif carla_edge == 'gneE5':
            # down stream (sumo: 0,1 --> carla: -2,-1)
            output_ln = carla_ln + 2
        else:
            # extra edge to block acceleration lane (sumo: 0 --> carla: -1)
            output_ln = carla_ln + 1
        lane_ID = carla_edge+'_'+output_ln
        return lane_ID
        # return traci.vehicle.getLaneID(self.getName())

    def getLaneIndex(self):
        # carla: -1,-2,-3 vs SUMO: gneE0_0,gneE0_1,gneE0_2
        carla_ln = self._current_waypoint.lane_id
        carla_edge = self._current_waypoint.road_id
        if carla_edge == 'gneE0':
            # upstream (sumo: 0,1 --> carla: -2,-1)
            output_ln = carla_ln + 2
        elif carla_edge == 'gneE1':
            # merge lane (sumo: 0 --> carla: -1)
            output_ln = carla_ln + 1
        elif carla_edge == 'gneE4':
            # merging area (sumo: 0,1,2 --> carla: -3,-2,-1)
            output_ln = carla_ln + 3
        elif carla_edge == 'gneE5':
            # down stream (sumo: 0,1 --> carla: -2,-1)
            output_ln = carla_ln + 2
        else:
            # extra edge to block acceleration lane (sumo: 0 --> carla: -1)
            output_ln = carla_ln + 1
        return output_ln
        #return traci.vehicle.getLaneIndex(self.getName())

    def getLeader(self, range=20):
        # return traci.vehicle.getLeader(self.getName(), range)
        # SUMO: Return the "leading vehicle id" together with the "distance". 
        # sense the surronding vehicles 
        vehicles = self._world.get_actors().filter('vehicle.*')
        x, y = self.getPosition()
        closest_leader = None 
        min_dist = 9999
        for v in vehicles:
            x1, y1 = v.get_location().x, v.get_location().y
            distance = math.sqrt((x-x1)**2 + (y-y1)**2)
            v_is_infront = (x1 > x) and (y-1.5 < y1 < y+1.5)   # check lane
            if v_is_infront and distance < min_dist:
                min_dist = distance
                closest_leader = v

        return None if closest_leader is None else (closest_leader.id, min_dist)

    def getLeftFollower(self):
        # res = traci.vehicle.getLeftFollowers(self.getName())
        # if len(res) > 0:
        #     return res[0] --> only return the ID 
        # else:
        #     return None
        vehicles = self._world.get_actors().filter('vehicle.*')
        x, y = self.getPosition()
        closest_follower_left = None 
        min_dist = 9999
        for v in vehicles:
            x1, y1 = v.get_location().x, v.get_location().y
            distance = math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
            v_is_left_back = (x1 < x) and (y1 > y+1.5)   # check lane
            if v_is_left_back and distance < min_dist:
                min_dist = distance
                closest_follower_left = v
        return None if closest_follower_left is None else closest_follower_left.id

    def getLeftLeader(self):
        # res = traci.vehicle.getLeftLeaders(self.getName())
        # if len(res) > 0:
        #     return res[0]--> only return the ID
        # else:
        #     return None

        vehicles = self._world.get_actors().filter('vehicle.*')
        x, y = self.getPosition()
        closest_leader_left = None 
        min_dist = 9999
        for v in vehicles:
            x1, y1 = v.get_location().x, v.get_location().y
            distance = math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
            v_is_left_front = (x1 > x) and (y1 > y+1.5)   # check lane
            if v_is_left_front and distance < min_dist:
                min_dist = distance
                closest_leader_left = v
        return None if closest_leader_left is None else closest_leader_left.id

    def getRightFollower(self):
        # res = traci.vehicle.getRightFollowers(self.getName())
        # if len(res) > 0:
        #     return res[0]--> only return the ID
        # else:
        #     return None

        vehicles = self._world.get_actors().filter('vehicle.*')
        x, y = self.getPosition()
        closest_follower_right = None 
        min_dist = 9999
        for v in vehicles:
            x1, y1 = v.get_location().x, v.get_location().y
            distance = math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
            v_is_right_back = (x1 < x) and (y1 < y-1.5)   # check lane
            if v_is_right_back and distance < min_dist:
                min_dist = distance
                closest_follower_right = v
        return None if closest_follower_right is None else closest_follower_right.id

    def getRightLeader(self):
        # res = traci.vehicle.getRightLeaders(self.getName())
        # if len(res) > 0:
        #     return res[0]--> only return the ID
        # else:
        #     return None
        vehicles = self._world.get_actors().filter('vehicle.*')
        x, y = self.getPosition()
        closest_leader_right = None 
        min_dist = 9999
        for v in vehicles:
            x1, y1 = v.get_location().x, v.get_location().y
            distance = math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
            v_is_right_front = (x1 > x) and (y1 < y-1.5)   # check lane
            if v_is_right_front and distance < min_dist:
                min_dist = distance
                closest_leader_right = v
        return None if closest_leader_right is None else closest_leader_right.id

    # used in platoon
    def getRoute(self):
        return self._route

    def getRemainingRoute(self):
        x,y = self.getPosition()
        if x >= 0:
            # finish E0/E1
            route_index = 1
        elif x >= 427:
            # finish E4
            route_index = 2
        return self._route[route_index:]

    # used in platoon and intersection controller
    def getLanePosition(self):
        # distance from vehicle to the beginning of the edge
        veh_ln = self.getLane()
        if "gneE0" in veh_ln:
            # upstream (sumo: 0,1 --> carla: -2,-1)
            start_pos = (-1204.72283593, -3.0)
        elif 'gneE1' in veh_ln:
            # merge lane (sumo: 0 --> carla: -1)
            start_pos = (-1204.72283593, -458.83698208)
        elif 'gneE4' in veh_ln:
            # merging area (sumo: 0,1,2 --> carla: -3,-2,-1)
            start_pos = (0.0, -3.0)
        elif 'gneE5' in veh_ln:
            # down stream (sumo: 0,1 --> carla: -2,-1)
            start_pos = (426.99172801, -3.0)
        else:
            # extra edge to block acceleration lane (sumo: 0 --> carla: -1)
            start_pos = (424.44184706, -9.95843474)
        x,y = self.getPosition()
        return math.sqrt((x-start_pos[0])**2 + (y-start_pos[1])**2)

    # used in platoon and intersection controller
    def getLanePositionFromFront(self):
        return self.getLaneLength(self.getLane()) - self.getLanePosition()

    # setters
    def getSpeed(self):
        speed_3D = self._vehicle.get_velocity()
        speed = math.sqrt(speed_3D.x**2 + speed_3D.y**2 + speed_3D.z**2) # in m/s
        return speed
        # return traci.vehicle.getSpeed(self.getName())

    def getState(self):
        return self._state

    def getTargetLane(self):
        return self._targetLane

    def getTau(self):
        return self._Tau
        #return traci.vehicle.get_Tau(self.getName())

    def setInActive(self):
        self._active = False

    def setMinGap(self, minGap):
        self._MinGap = minGap 
        # self._setAttr("set_MinGap", _minGap)

    def setState(self, state):
        self._state = state

    # not used in CARLA
    def setSpeedFactor(self, speedFactor):
        self._SpeedFactor = speedFactor
        # self._setAttr("set_SpeedFactor", _speedFactor)

    # duplicate name; not applicable in CARLA
    def setTargetLane(self, targetLane):
        self._targetLane = targetLane

    def setTau(self, tau):
        self._Tau = tau
        # self._setAttr("set_Tau", _tau)
    
    def getClosestDistances(self, veh):
    
        Y_ML = [-8.0,-4.8,-1.6]
        veh_name = veh[1]
        veh_edge = veh[2][veh_name][81]
        veh_speed = veh[2][veh_name][64]  # 66 is the key for position
        veh_pos = veh[2][veh_name][66]  # 66 is the key for position
        
        all_vehs_around = list(veh[2].keys()) # This also includes the current vehicle. So, this is removed in next line
        all_vehs_around.remove(veh_name)
        
        distances = [self._sensorRange]*6  # [left ahead, left behind, same lane ahead, same lane behind, right ahead, right behind]
        speeds = [50]*6
        signals = [0]*6 # signal =0 either means no signal or no vehicle or no lane
        
        vehIDs = [self._sensorRange]*6
        
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
            distances[0:2] = [a if a < self._sensorRange else -1 for a in distances[0:2]]
            distances[4:] = [a if a < self._sensorRange else -1 for a in distances[4:]]
            speeds[0:2] = [a if a < 50 else -1 for a in speeds[0:2]]
            speeds[4:] = [a if a < 50 else -1 for a in speeds[4:]]
        elif veh_edge == 'gneE0_0' or veh_edge == 'gneE4_0' or veh_edge == 'gneE5_0' or veh_edge == ':gneJ6_0_1':
            distances[4:] = [a if a < self._sensorRange else -1 for a in distances[4:]]
            speeds[4:] = [a if a < 50 else -1 for a in speeds[4:]]
        elif veh_edge == 'gneE0_1' or veh_edge == 'gneE4_2' or veh_edge == 'gneE5_1' or veh_edge == ':gneJ1_1_1' or veh_edge == ':gneJ6_0_2':
            distances[0:2] = [a if a < self._sensorRange else -1 for a in distances[0:2]]
            speeds[0:2] = [a if a < 50 else -1 for a in speeds[0:2]]
        allInputs = distances + speeds + signals + list(veh_pos) + [veh_speed]
        return allInputs, vehIDs