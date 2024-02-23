# -*- coding: utf-8 -*-

"""
HDMap manager
"""
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import uuid

import cv2
import carla
import numpy as np
import pandas as pd
import networkx as nx
from matplotlib.path import Path
from shapely.geometry import Polygon
from sqlite3 import connect
from sklearn.neighbors import KNeighborsClassifier

from  opencda.core.plan.behavior_fsm import BehaviorFSM
from opencda.core.sensing.perception.sensor_transformation import \
    world_to_sensor
from opencda.core.map.map_utils import \
    lateral_shift, list_loc2array, list_wpt2array, convert_tl_status
from opencda.core.map.map_drawing import \
    cv2_subpixel, draw_agent, draw_road, draw_lane
from opencda.core.plan.behavior_fsm_states import BehaviorSuperStates, BehaviorStates
from opencda.core.common.misc import distance_vehicle, draw_trajetory_points, \
    cal_distance_angle, compute_distance

class vehicle(object):
    '''
    Mock vehicle class for debug
    '''
    def __init__(self):
        # Create superstates
        self.world = None
        self.id = '10086'
    def get_world(self):
        return self.world


class RegulationManager(object):
    """
    This class is used to manage HD Map. We emulate the style of Lyft dataset.

    Parameters
    ----------
    vehicle : Carla.vehicle
        The ego vehicle.

    carla_map : Carla.Map
        The carla simulator map.

    config : dict
        All the map manager parameters.

    Attributes
    ----------
    world : carla.world
        Carla simulation world.

    center : carla.Transform
        The rasterization map's center pose.

    """

    def __init__(self, vehicle,
                 current_superstate, current_state,
                 next_superstate, next_state, config):
        # ego vehicle info
        self.vehicle = vehicle
        self.world = vehicle.get_world()
        self.agent_id = vehicle.id
        # this is not going to change, so read this one time and reuse this variable later
        self.vehicle_type = self.get_vehicle_type()

        # FSM info
        self.current_superstate = current_superstate
        self.current_state = current_state
        self.next_superstate = next_superstate
        self.next_state = next_state

        # data base variables
        self.regulation_database_path = config['database_directory']
        self.sample_input_path = config['input_directory']
        self.reg_database = pd.read_csv(self.regulation_database_path)  # Load the CSV data into a pandas DataFrame
        self.sample_input = pd.read_csv(self.sample_input_path)

        # variables to interpret scene
        self.current_scene = {}
        self.next_step_scene = {}
        self.input_vector = []

        # leading vehicle
        self.is_hazard = False
        self.is_obstacle_vehicles = False
        self.objects = {}

    def check_curve_ahead(self, previous_wpt, future_wpt):
        # check lateral offset from previous waypoint to current waypoint
        vec_norm, angle = cal_distance_angle(previous_wpt.transform.location,
                                             future_wpt.transform.location,
                                             future_wpt.transform.rotation.yaw)

        # distance in the lateral direction
        lateral_diff = abs(
            vec_norm *
            math.sin(
                math.radians(
                    angle - 1 if angle > 90 else angle + 1)))

        # find position of the vehicle
        boundingbox = self.vehicle.bounding_box
        veh_width = 2 * abs(boundingbox.location.y - boundingbox.extent.y)
        large_lateral_change = veh_width < lateral_diff
        return large_lateral_change

    def check_elevation_ahead(self, previous_wpt, future_wpt):
        prv_z = previous_wpt.transform.location.z
        nxt_z = future_wpt.transform.location.z
        elevation_diff = abs(prv_z - nxt_z)
        is_large_elevation_change = elevation_diff >= 1.0
        return is_large_elevation_change

    def count_total_lanes(self, curr_wpt):
        tmp_left_wpt = curr_wpt
        tmp_right_wpt = curr_wpt
        left_count = 0
        right_count = 0
        while tmp_left_wpt.get_left_lane() is not None:
            tmp_left_wpt = tmp_left_wpt.get_left_lane()
            left_count += 1
        while tmp_right_wpt.get_right_lane() is not None:
            tmp_right_wpt = tmp_right_wpt.get_right_lane()
            right_count += 1
        return left_count+right_count

    def distance_to_intersection(self, curr_wpt):
        """
        Check the next waypoints is near the intersection. This is done by
        check the distance between the waypoints and the traffic light.

        Parameters
        ----------
        objects : dict
            The dictionary contains all objects info.

        waypoint_buffer : deque
            The waypoint buffer.

        Returns
        -------
        is_junc : boolean
            Whether there is any future waypoint in the junction shortly.
        """
        # Get all the actors in the world
        actor_list = self.world.get_actors()
        # Filter for traffic lights
        traffic_lights = actor_list.filter('traffic.traffic_light')
        distances = []
        wpt = curr_wpt
        for tl in traffic_lights:
            distance = \
                tl.get_location().distance(wpt.transform.location)
            distances.append(distance)
        # cast to feat
        distance = min(distances)*3.28084
        return distance

    def update_step(self, is_hazard, is_obstacle_vehicle_exist, objects):
        '''
        Update the detection results
        Parameters
        ----------
        objects

        Returns
        -------

        '''
        self.is_hazard = is_hazard
        self.is_obstacle_vehicles = is_obstacle_vehicle_exist
        self.objects = objects

    def get_current_weather(self):
        '''
        Find the current weather parameter.
        Returns
        -------

        '''
        return self.world.get_weather()

    def get_surrounding_traffic(self):
        '''
        Find emergency vehicle, front vehicle and avg speed
        Returns
        -------

        '''
        obstacle_vehicles = self.objects['vehicles']
        nearby_speeds = []
        emergency_vehicles = []
        leading_vehicles = []

        if len(obstacle_vehicles) < 1:
            return None, None, None
        else:
            for vehicle in obstacle_vehicles:
                # check emergency
                v_type = vehicle.type_id()
                if 'firetruck' or 'police' or 'ambulance' in v_type:
                    emergency_vehicles.append(vehicle)

                # consider speed
                nearby_speeds.append(vehicle.get_velocity())

                # check lane
                map = self.world.getmap()
                ego_lane_id = map.get_waypoint(self.vehicle.get_location()).lane_id
                vehicle_labe_id = map.get_waypoint(vehicle.get_location()).lane_id
                if ego_lane_id == vehicle_labe_id:
                    leading_vehicles.append(vehicle)

        return emergency_vehicles, nearby_speeds, leading_vehicles

    def find_nearest_veh(self, vehicle_list):
        min_dist = 999
        nearest_veh = vehicle_list[0]
        for vehicle in vehicle_list:
            dist = self.vehicle.get_location().distance(vehicle.get_location())
            if dist < min_dist:
                nearest_veh = vehicle
                min_dist = dist
        return min_dist, nearest_veh

    def interpret_scene(self):
        # 0. read current ego info
        ego_location = self.vehicle.get_location()
        ego_waypoint = self.world.get_map().get_waypoint(ego_location)
        ego_speed = self.vehicle.get_velocity()

        # 1. current scenario
        if self.current_superstate.name == 'LANE_FOLLOWING':
            self.current_scene['Current Scenario'] = 'Traveling'
        elif self.current_superstate.name == 'INTERSECTION':
            self.current_scene['Current Scenario'] = 'Traveling' if self.vehicle.get_velocity() != 0 else 'Stopped'
        elif self.current_superstate.name == 'OVERTAKING':
            self.current_scene['Current Scenario'] = 'Overtaking'

        # 2. future scenario
        if self.next_superstate.name == 'LANE_FOLLOWING':
            self.next_step_scene['Planned Scenario'] = 'Traveling'
        elif self.next_superstate.name == 'INTERSECTION':
            self.next_step_scene['Planned Scenario'] = 'Traveling' if \
                self.vehicle.get_velocity() != 0 else 'Stopped'
        elif self.next_superstate.name == 'OVERTAKING':
            self.next_step_scene['Planned Scenario'] = 'Overtaking'

        # 3. Vehicle Type (matched with carla vehicle blueprint base_type
        self.current_scene['Vehicle Type'] = self.vehicle_type

        # 4. Vehicle speed (currently using m/s)
        self.current_scene['Vehicle Speed'] = self.vehicle.get_velocity()

        # 5. Vehicle Lane Position (From Right) todo: need to change to match openDrive
        self.current_scene['Vehicle Lane Position (From Right)'] = ego_waypoint.lane_id

        # 6. 'AV' --> always true
        self.current_scene['AV'] = True

        # 7. 'Road Type' (highway, this is not relavent: oneway opposite, oneway same)
        speed_limit = self.vehicle.get_speed_limit() #km/h
        if speed_limit >= 80:
            self.current_scene['Road Type'] = 'highway'
        else:
            self.current_scene['Road Type'] = 'local'

        # 8.  'Road Surface Type' todo: currently all asphalt in CARLA
        self.current_scene['Road Surface Type'] = 'Asphalt'

        # 9. 'Upcoming Road Geometry'
        nxt_wpt = ego_waypoint.next(max(ego_speed/3.6, 10.0))[0]
        if self.check_curve_ahead(ego_waypoint, nxt_wpt):
            self.current_scene['Upcoming Road Geometry'] = 'Curve'
        if self.check_elevation_ahead(ego_waypoint, nxt_wpt):
            self.current_scene['Upcoming Road Geometry'] = 'Hill'
        if nxt_wpt is None:
            self.current_scene['Upcoming Road Geometry'] = 'Dead-end'
        if not self.check_curve_ahead(ego_waypoint, nxt_wpt) \
                and self.check_elevation_ahead(ego_waypoint, nxt_wpt) and \
                nxt_wpt is not None:
            self.current_scene['Upcoming Road Geometry'] = 'Straightway'

        # 10. 'Lane Count'
        total_lanes = self.count_total_lanes(ego_waypoint)
        self.current_scene['Lane Count'] = total_lanes

        # 11-12. todo: merge dashed yellow position, and lane left stripping (is it marking?)
        '''   
        'Dashed Yellow Lane Position (left of lane)',
        'Solid Yellow Lane Position (left of lane)',
        'Current Lane Left Striping', 'Current Lane Right Striping',
        '''
        # 13. Current Road Observed Max Speed Limit
        speed_limit = self.vehicle.get_speed_limit()
        self.current_scene['Current Road Observed Max Speed Limit'] = speed_limit

        # 14. Min limit (todo: CARLA do not have it, set to 0 for now)
        self.current_scene['Current Road Observed Min Speed Limit'] = 0.0

        # 15. 'Intersection Presence'
        self.current_scene['Intersection Presence'] = ego_waypoint.is_junction

        # 16. Distance From Intersection (ft)
        self.current_scene['Distance From Intersection (ft)'] = self.distance_to_intersection(ego_waypoint)

        # 17. Current Lane Signal Status At Intersection (note: set to None if no intersection)
        if self.vehicle.is_at_traffic_light:
            light_state = self.vehicle.get_light_state()
            self.current_scene['Current Lane Signal Status At Intersection'] = light_state
        else:
            self.current_scene['Current Lane Signal Status At Intersection'] = None

        # 18 and 19. Signage At Intersection
        land_marks = ego_waypoint.get_landmarks(30)
        if land_marks:
            closest_landmark = land_marks[0]
            self.current_scene['Signage At Intersection'] = True
            self.current_scene['Intersection Signage Type'] = closest_landmark.value
        else:
            self.current_scene['Signage At Intersection'] = False
            self.current_scene['Intersection Signage Type'] = None

        # 20. Presence Of Leading Vehicle --> use condition for car following in behavioral manager
        self.current_scene['Presence Of Leading Vehicle'] = self.is_obstacle_vehicles

        # 21. impeding traffic --> use condition for car following in behavioral manager
        self.current_scene['Leading Vehicle Impeding Traffic'] = self.is_hazard
        self.current_scene['Current Lane Obstacle Presence'] = self.is_hazard

        # surrounding vehicles
        emergency_vehicles, nearby_speeds, leading_vehicles = self.get_surrounding_traffic()
        if emergency_vehicles:
            self.current_scene['Emergency Vehicle Presence'] = True
            # find dist
            min_emg_dist, nearest_emg_veh = self.find_nearest_veh(emergency_vehicles)
            self.current_scene['Distance from emergency vehicle (ft)'] = min_emg_dist*3.28
        else:
            self.current_scene['Emergency Vehicle Presence'] = False

        if nearby_speeds:
            self.current_scene['Surrounding Traffic Speed'] = np.mean(np.array(nearby_speeds))

        if leading_vehicles:
            min_leading_dist, nearest_leading_veh = self.find_nearest_veh(leading_vehicles)
            self.current_scene['Distance from leading vehicle (ft)'] = min_leading_dist * 3.28

        # weather
        self.current_scene['Weather Condition'] = self.get_current_weather()
        if 'Wet' or 'Cloudy' or 'Rain' or 'Sunset'\
                in self.current_scene['Weather Condition']:
            self.current_scene['Weather Induced Road Condition'] = True
        else:
            self.current_scene['Weather Induced Road Condition'] = False

        # demo scenarios avoid bike
        for v in self.objects['vehicles']:
            self.current_scene['Cyclist Nearby'] = True if 'bike' in v.type_id else False
            self.current_scene['Cyclist ID'] = v.id


        '''
        Use city object label:
        'Presence Of Railroad Crossing', --> CARLA city object label
        'Distance From Railroad Crossing (ft)',  --> CARLA city object label
        'Speed Bump Presence',  --> CARLA city object label
        'Bridge, Viaduct, or Tunnel Presence',  --> CARLA city object label
        'Distance From Bridge, Viaduct, or Tunnel (ft)',  --> CARLA city object label
        
        what data to put as empty ?
        '''

        '''
        Use general API
        'Distance to leading vehicle'
        'Surrounding Traffic Speed', --> how to define, use average ? 
        'Distance From Emergency Vehicle (ft)' --> find the nearset ?
        '''

        '''
        CARLA do not have:
        'Crosswalk Presence',
        'Distance From Crosswalk (ft)', 
        'Intersection/Crosswalk Pedestrian Presence',
        'Cyclist/Scooterist Presence', 
        'Construction Zone Presence',
        'Construction Zone Presence In Current Lane',
        'Distance From Construction Zone (ft)', 
        'Presence Of School Zone',
        'Distance From School Zone (ft)',
        'Illumination Condition' 
        
        Research question: 
        *** How to extract info in the real environment. (Perception, localization... ) ***
        step 1: simulation (project)
        step 2: integrate in the localization pipe-line
        step 3: categorize all labels (vage, precise, from other vehicles, etc...)
        
        For Matt: 
        1. agree with the current label(category) formulation 
        2. finish the decomposition (current 3 chapters, and the rest of it)
        3. test if existing language model (GPT) can do this work. --> demonstrate it is doable.
            (I can also do this ..., use multiple examples and prompts)
        
        
        '''
    def get_vehicle_type(self):
        '''
        Find vehicle type of the ego vehicle.
        Returns
        -------

        '''
        vehicle_type_id = self.vehicle.type_id
        ego_vehicle_bp = \
            self.world.get_blueprint_library().find(vehicle_type_id)
        return ego_vehicle_bp.base_type



# local tests
# current_superstate = BehaviorSuperStates.LANE_FOLLOWING
# current_state = BehaviorStates.GO_STRAIGHT
# next_superstate = BehaviorSuperStates.LANE_FOLLOWING
# next_state = BehaviorStates.GO_STRAIGHT
#
# v_test = vehicle()
# config = {}
# config['database_directory'] = '/home/xuhan/OpenCDA/opencda/assets/ADS_regulation_database/ADS_Rules_Database.csv'
# config['input_directory'] = '/home/xuhan/OpenCDA/opencda/assets/ADS_regulation_database/ADS_Rules_Input.csv'
# RegulationManager = RegulationManager(vehicle,
#                                       current_superstate, current_state,
#                                       next_superstate, next_state, config)
# print(RegulationManager.sample_input.columns)
# print('regulation manager initiation done...')