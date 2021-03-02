# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module implements gfs_pl_speed agent that calculate platoon index and platoon speed.
gfs_pl_speed model included:
1.  frontVeh, rearVeh = self.getBestMergePosition(vehicle,possiblePlatoon,fisMergeLoc) --> merger (mg)
2.  des_speed = self.getDesiredSpeed(v,gfs_pl_speed_pl_speed) --> platoon leader (pl)
3.  des_speed = self.getDesiredSpeed(v,gfs_pl_speed_m) --> merger 

gfs_pl_speed input:
1. controlled vehicle (pl, mg)
2. possibleplatoon --> a possible platoon to join (platoon need to have all vehicle information)
3. fisMergeLoc --> gfs_pl_speed model weight 
4. gfs_pl_speed_pl_speed --> gfs_pl_speed model weight 
5. gfs_pl_speed_m --> gfs_pl_speed model weight 

gfs_pl_speed output:
1. frontVeh_mg
2. rearVeh_mg
3. des_speed_pl
4. des_speed_mg

"""

import math
import numpy as np

class GFSController(object):
    """
    A class controller to estimate proper platooning position and speed
    """

    def __init__(self, fisMergeLoc, gfs_pl_speed, gfs_m_speed, platooning_manager, sensor_range, dt):
        """
        Construct class
        :param vehicle: carla Actor
        :param world: platooning world object
        :param behavior: driving style.
        :param communication_range:
        :param buffer_size: queue size for behavior planning
        :param sample_resolution: the minimum distance between any waypoint in the routing
        :param cda_enabled:  whether the vehicle equipped with cda feature
        """
        self.fisMergeLoc = fisMergeLoc
        self.gfs_pl_speed = gfs_pl_speed
        self.gfs_m_speed = gfs_m_speed
        self.platooning_manager = platooning_manager
        self.sensor_range = sensor_range
        self.dt = dt

    def getBestMergePosition(self, merge_veh, platooning_manager):
        
        vehicle_managers = [v_manager for v_manager in platooning_manager.vehicle_manager_list]
        # check y coordinates to make sure vehicle is in middle lane
        platoon_vehicles = [manager.vehicle for manager in vehicle_managers if abs(manager.vehicle.get_location().y-7.5) < 1.5]
        platoon_length = len(platoon_vehicles)
        # name should be vID
        veh_name = merge_veh.id
        # get all context information 
        veh_context = self.getContext(merge_veh)
        # re-arrange simulation data
        veh_all = [0, veh_name, veh_context]
        # re-arrange as gfs_pl_speed input
        inputs = self.getInputs2FIS(veh_all)
        # start defuzz
        all_scores = []
        for j in range(platoon_length+1): # Number of gaps in platoon where vehicle can merge
            
            if j == 0:
                leadPos = [500,500]
                leadSpeed = 50
                rear = platoon_vehicles[j]
                rearPos = self.getPosition(rear)
                rearSpeed = self.getSpeed(rear)
                    
            elif j == platoon_length:
                lead = platoon_vehicles[j-1]
                leadPos = self.getPosition(lead)
                leadSpeed = self.getSpeed(lead)
                rearPos = [500,500]
                rearSpeed = 50
            else:
                lead = platoon_vehicles[j-1]
                rear = platoon_vehicles[j]
                leadPos = self.getPosition(lead)
                leadSpeed = self.getSpeed(lead)
                rearPos = self.getPosition(rear)
                rearSpeed = self.getSpeed(rear)
                
            full_list2 = inputs[0:4] + inputs[6:10] + inputs[12:16] + [inputs[18]] + \
                         [inputs[20]] + [leadPos[0]] + [leadSpeed] + [rearPos[0]] + [rearSpeed] #y-coordinate of ego vehicle is ignored
            
            score = self.fisMergeLoc.eval_op(np.array(full_list2))
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
            
        return leadVeh, rearVeh

    def getDesiredSpeed_pl(self, vehicle):
        
        veh_name = vehicle.id
        cur_speed = self.getSpeed(vehicle)
        veh_context = self.getContext(vehicle)
        
        veh_all = [0, veh_name, veh_context]
        
        inputs = self.getInputs2FIS(veh_all)
        outs = self.gfs_pl_speed.eval_op(inputs)
        accln = 3.55*outs[0]-0.95 # accln in the range[-4.5,2.6]
        des_speed = cur_speed + accln*self.dt
        
        return des_speed

    def getDesiredSpeed_m(self, vehicle):
        
        veh_name = vehicle.id
        cur_speed = self.getSpeed(vehicle)
        veh_context = self.getContext(vehicle)
        
        veh_all = [0, veh_name, veh_context]
        
        inputs = self.getInputs2FIS(veh_all)
        outs = self.gfs_m_speed.eval_op(inputs)
        accln = 3.55*outs[0]-0.95 # accln in the range[-4.5,2.6]
        des_speed = cur_speed + accln*self.dt
        
        return des_speed

    def getInputs2FIS(self, veh):
        # the Y_ML need to be more modular based on simulator difference
        # Y_ML = [-8.0,-4.8,-1.6] # these are the y-coordinates for mainline vehicles (3 lanes)
        Y_ML = [-10.5, -7.5, -4.5] # carla y coordinates for lane centers

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
            # add more tolerance for veh_pos
            if  -10.5 - 1.5 < veh_pos[1] < -10.5 + 1.5:
                # acc lane
                 v_y = -10.5
            elif  -7.5 - 1.5 < veh_pos[1] < -7.5 + 1.5:
                # mid lane
                v_y = -7.5
            elif -4.5 - 1.5 < veh_pos[1] < -4.5 + 1.5:
                # left lane
                v_y = -4.5
            # find veh lane
            # veh_lane = Y_ML.index(veh_pos[1])
            veh_lane = Y_ML.index(v_y)
        
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

    def getContext(self, vehicle):
        '''
        read {'v_id': speed(64), position(66), lane_ID(81), lane_index(82), signals(91)} data for all vehs within 150m
        data structure: float, (float, float), 'edge_lane', int, int
        :return: context
        '''
        context = {}
        world = vehicle.get_world()
        vehicles = world.get_actors().filter('vehicle.*')
        x,y = vehicle.get_location().x, -1*vehicle.get_location().y
        for v in vehicles:
            x1, y1 = v.get_location().x, -1*v.get_location().y
            distance = math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
            if distance <= self.sensor_range:
                # sense within range
                vID = v.id
                speed = math.sqrt(v.get_velocity().x**2 + v.get_velocity().y**2 + v.get_velocity().z**2)
                position = self.getPosition(v) # this function reverse the y-coordinates sign
                lane = self.getLane(v)
                lane_index = self.getLaneIndex(v)
                signal_bin = self.getSignal(v)
                # append to context dictionary
                context[vID] = {64:speed, 66:position, 81:lane, 82:lane_index, 91:signal_bin}
        # self.Context = traci.vehicle.getContextSubscriptionResults(self._name)
        return context

    def getLane(self, vehicle):
        # carla: -1,-2,-3 vs SUMO: gneE0_0,gneE0_1,gneE0_2
        curr_map = vehicle.get_world().get_map()
        current_waypoint = curr_map.get_waypoint(vehicle.get_location())
        carla_ln = current_waypoint.lane_id
        carla_edge = current_waypoint.road_id

        # edges
        if carla_edge == 63:
            edge_name = 'gneE0'
            # upstream (sumo: 0,1 --> carla: -2,-1)
            output_ln = carla_ln + 2
        elif carla_edge == 61:
            edge_name = 'gneE1'
            # merge lane (sumo: 0 --> carla: -1)
            output_ln = carla_ln + 1
        elif carla_edge == 60:
            edge_name = 'gneE4'
            # merging area (sumo: 0,1,2 --> carla: -3,-2,-1)
            output_ln = carla_ln + 3
        elif carla_edge == 62:
            edge_name = 'gneE5'
            # down stream (sumo: 0,1 --> carla: -2,-1)
            output_ln = carla_ln + 2
        elif carla_edge == 64:
            edge_name = 'gneE2'
            output_lane = 0

        # junctions
        elif carla_edge == 65:
            edge_name = ':gneJ6_0'
            # (-1 --> 0)
            output_ln = carla_ln + 1
        elif carla_edge == 66:
            edge_name = ':gneJ6_0' # this junction uses the same name in SUMO
            # (-1, -2 --> 2, 1)
            output_ln = carla_ln + 3
        elif carla_edge == 67:
            edge_name = ':gneJ1_0'
            # (-1 --> 0)
            output_ln = carla_ln + 1
        elif carla_edge == 68:
            edge_name = ':gneJ1_1'
            # (-1, -2 --> 1, 0)
            output_ln = carla_ln + 2
        else:
            # extra edge to block acceleration lane (sumo: 0 --> carla: -1)
            output_ln = 0
            edge_name = 'None'
        lane_ID = edge_name+'_'+str(output_ln)
        return lane_ID

    def getLaneIndex(self, vehicle):
        # carla: -1,-2,-3 vs SUMO: gneE0_0,gneE0_1,gneE0_2
        curr_map = vehicle.get_world().get_map()
        current_waypoint = curr_map.get_waypoint(vehicle.get_location())
        carla_ln = current_waypoint.lane_id
        carla_edge = current_waypoint.road_id
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

    def getSignal(self, vehicle):

        signal = vehicle.get_light_state()
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
            # others (not relevent for gfs_pl_speed, mark as 0)
            signal_bin = 0b0000

        return signal_bin

    def getPosition(self, vehicle):
        '''
        CARLA and SUMO/Opendrive uses opposite coordinate, hence the y value has negative sign
        '''
        x,y = vehicle.get_location().x, -1*vehicle.get_location().y
        return (x,y)

    def getSpeed(self, vehicle):
        speed_3d = vehicle.get_velocity()
        speed = math.sqrt(speed_3d.x**2 + speed_3d.y**2 + speed_3d.z**2)
        return speed