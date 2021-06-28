# OpenCDA Python API Reference
This reference contains all the details of the OpenCDA Python API. 

## OpenCDA Module: co_simulation
Co-simulation with SUMO, will be released in v0.2.

## OpenCDA Module: core
This is the core module of the OpenCDA framework that regulates the motion relatetd controls. 

### opencda.core.actuation
This is the actuation function stack that controls CARLA ego vehicle. 

#### opencda.core.actuation.control_manager
Controller interface.

- **<font color="#f8805a">Classes</font>**

`ControlManager(control_config)`
:   Interface to select different types of controller.
    
    Construct class
    Args:
        control_config(dict): Controller params.

    ### Methods

    `run_step(self, target_speed, waypoint)`
    :

    `update_info(self, ego_pos, ego_speed)`
    :

#### opencda.core.actuation.pid_controller
PID Control Class.

- **<font color="#f8805a">Classes</font>**

`Controller(args)`
:   VehiclePIDController is the combination of two PID controllers
    (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    
    Construct class
    :param args: the parameters for pid controller

    ### Methods

    `dynamic_pid(self)`
    :   Compute kp, kd, ki based on current speed
        :return:

    `lat_run_step(self, target_location)`
    :   Generate the throttle command based on current speed and target speed
        :param target_location: target waypoint
        :return: steering scalar

    `lon_run_step(self, target_speed)`
    :   Generate the throttle command based on current speed and target speed
        :param target_speed: target speed in km/h
        :return: throttle scalar

    `run_step(self, target_speed, waypoint)`
    :   Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint
        at a given target_speed.
        
            :param target_speed: desired vehicle speed
            :param waypoint: target location encoded as a waypoint
            :return: control command

    `update_info(self, ego_pos, ego_spd)`
    :   Update ego position and speed to controller.
        :param ego_pos: ego position, carla.transform
        :param ego_spd: ego speed, km/h
        :return:
### opencda.core.application 
V2V, V2I application are all managed with this class.  

#### opencda.core.application - Platooning
Here, we provide an implementation of Autonomous vehicle platooning based on finite state machine.

##### opencda.core.application.platooning.fsm
Finite State Machine

- **<font color="#f8805a">Classes</font>**

`FSM(value, names=None, *, module=None, qualname=None, type=None, start=1)`
:   An enumeration.

    ### Ancestors (in MRO)

    * enum.Enum

    ### Class variables

    `ABONDON`
    :

    `BACK_JOINING`
    :

    `CUT_IN_TO_BACK`
    :

    `DISABLE`
    :

    `FRONT_JOINING`
    :

    `JOINING`
    :

    `JOINING_FINISHED`
    :

    `LEADING_MODE`
    :

    `MAINTINING`
    :

    `MOVE_TO_POINT`
    :

    `OPEN_GAP`
    :

    `SEARCHING`
    :

##### opencda.core.application.platooning.platoon_behavior_agent
Behavior manager for platooning specifically

- **<font color="#f8805a">Classes</font>**

`PlatooningBehaviorAgent(vehicle, vehicle_manager, v2x_manager, behavior_yaml, platoon_yaml, carla_map)`
:   The behavior agent for platooning
    
    Construct class
    :param vehicle: carla actor todo:remove this later
    :param vehicle_manager: vehicle manager of this agent.
    :param v2x_manager: communication manager
    :param behavior_yaml: configure yaml file for normal behavior agent
    :param platoon_yaml:  configure yaml file for platoon behavior agent
    :param carla_map: Carla HD Map
    :return

    ### Ancestors (in MRO)

    * opencda.core.plan.behavior_agent.BehaviorAgent

    ### Methods

    `calculate_gap(self, distance)`
    :   Calculate the current vehicle and frontal vehicle's time/distance gap
        :param distance:  distance between the ego vehicle and frontal vehicle
        :return:

    `joining_finish_manager(self, insert_vehicle='front')`
    :   Called when a joining is finish to update the platoon manager list.
        :param insert_vehicle: indicate use the front or rear vehicle index to update the platoon manager list.
        :return:

    `platooning_following_manager(self, inter_gap)`
    :   Car following behavior in platooning with gap regulation
        :param inter_gap: the gap designed for platooning
        :return:

    `platooning_merge_management(self, frontal_vehicle_vm)`
    :   Merge the vehicle into the platooning
        :param frontal_vehicle_vm:
        :return:

    `run_step(self, target_speed=None, collision_detector_enabled=True, lane_change_allowed=True)`
    :   Run a single step for navigation under platooning agent. Finite state machine is used to switch between
        different platooning states.
        Args:
            target_speed (float): Target speed in km/h
            collision_detector_enabled (bool): Whether collision detection enabled.
            lane_change_allowed (bool): Whether lane change is allowed.
        
        Returns:

    `run_step_back_joining(self)`
    :   Back-joining Algorithm
        :return: control command and whether back joining finished

##### opencda.core.application.platooning.platoon_debug_helper
Analysis + visualization functions for platooning.

- **<font color="#f8805a">Classes</font>**

`PlatoonDebugHelper(actor_id)`
:   This class aims to save statistics for platoon behaviour
    Attributes:
        time_gap_list (list): The list containing intra-time-gap(s) of all time-steps
        dist_gap_list(list): The list containing distance gap(s) of all time-steps

    ### Ancestors (in MRO)

    * opencda.core.plan.planer_debug_helper.PlanDebugHelper

##### opencda.core.application.platooning.platooning_manager
Platooning Manager.

- **<font color="#f8805a">Classes</font>**

`PlatooningManager(config_yaml, cav_world)`
:   Platooning manager for vehicle managers
    
    Construct class
    :param config_yaml:
    :param cav_world: CAV world object

    ### Methods

    `add_member(self, vehicle_manager, leader=False)`
    :   Add memeber to the current platooning
        :param leader: whether this cav is a leader
        :param vehicle_manager:
        :return:

    `cal_center_loc(self)`
    :   Calculate center location of the platoon
        :return:

    `destroy(self)`
    :   Destroy vehicles
        :return:

    `evaluate(self)`
    :

    `reset_speed(self)`
    :   After joining request accepted for certain steps, the platoon will return to the origin speed.
        :return:

    `response_joining_request(self, request_loc)`
    :   Identify whether to accept the joining request based on capacity.
        Args:
            request_loc (carla.Location): request vehicle location.
        
        Returns:

    `run_step(self)`
    :   Run a step for each vehicles.
        :return:

    `set_destination(self, destination)`
    :   Set desination of the vehicle managers in the platoon.
        TODO: Currently we assume all vehicles in a platoon will have the same destination
        :return:

    `set_lead(self, vehicle_manager)`
    :   Set the leader of the platooning
        :param vehicle_manager:
        :return:

    `set_member(self, vehicle_manager, index, lead=False)`
    :   Set member at specific index
        :param lead:
        :param vehicle_manager:
        :param index:
        :return:

    `update_information(self)`
    :   Update CAV world information for every member in the list.
        :return:

    `update_member_order(self)`
    :   Update the members' front and rear vehicle.
        This should be called whenever new member added to the platoon list
        :return:

##### opencda.core.application.platooning.platooning_plugin
Platooning plugin for communication and track FSM.

- **<font color="#f8805a">Classes</font>**

`PlatooningPlugin(search_range, cda_enabled)`
:   Platooning Plugin
    
    Construct class
    :param search_range:
    :param cda_enabled:

    ### Methods

    `match_platoon(self, cav_world)`
    :   A naive way to find the best position to join a platoon
        :param cav_world: an object containing all existing platoons
        :return: platoon found or not, closest platoon member team id, and a list containing the vehicle managers

    `reset(self)`
    :   Reset to the origin status
        :return:

    `search_platoon(self, ego_pos, cav_world)`
    :   Search platoon candidate in the range
        :param ego_pos:
        :param cav_world:
        :return: the uuid of platoon member, platoon object

    `set_platoon(self, in_id, platooning_object=None, platooning_id=None, leader=False)`
    :   Set platooning status
        :param platooning_object: platooning manager todo: remove this later
        :param platooning_id: platoon id the cav belongs to
        :param in_id: the position in the platoon, etc. 0 represents leader and 1 represents the second position
        :param leader: indicate whether this cav is a leader in platoon
        :return:

    `set_status(self, status)`
    :   Set FSM status
        :param status:
        :return:

    `update_info(self, ego_pos, ego_spd)`
    :   Update the ego position and speed
        :param ego_pos: ego position, carla.Transform
        :param ego_spd: ego speed, km/h
        :return:
### opencda.core.common 
This module regulates simulation related objects in the CARLA simulation world. 

#### opencda.core.common.cav_world
Class that regulates the CARLA simulation world.

- **<font color="#f8805a">Classes</font>**

`CavWorld(apply_ml=False)`
:   A customized world object to save all CDA vehicle information and shared ML models
    :return:
    
    Construct class.
    Args:
        apply_ml (bool): whether apply ml/dl models in this simulation, please make sure
                         you have install torch/sklearn before setting this to True.

    ### Methods

    `get_platoon_dict(self)`
    :   Return existing platoons
        :return:

    `get_vehicle_managers(self)`
    :   Return vehicle manager dictionary
        :return:

    `locate_vehicle_manager(self, loc)`
    :   Locate the vehicle manager based on the given location.
        Args:
        loc (carla.Location): vehicle location.
        
        Returns:
        (VehicleManager): The vehicle manager at the give location.

    `update_platooning(self, platooning_manger)`
    :   Add created platooning
        :param platooning_manger:
        :return:

    `update_vehicle_manager(self, vehicle_manager)`
    :   Update created CAV manager to the world
        :param vehicle_manager:
        :return:

#### opencda.core.common.misc
Module with auxiliary functions.

- **<font color="#f8805a">Functions</font>**

`cal_distance_angle(target_location, current_location, orientation)`
:   Calculate the vehicle current relative distance to target location
    :param target_location:
    :param current_location:
    :param orientation:
    :return: distance and angle

    
`compute_distance(location_1, location_2)`
:   Euclidean distance between 3D points
    
        :param location_1: 3D points
        :param location_2: 3D points

    
`compute_magnitude_angle(target_location, current_location, orientation)`
:   Compute relative angle and distance between a target_location and a current_location
    
        :param target_location: location of the target object
        :param current_location: location of the reference object
        :param orientation: orientation of the reference object
        :return: a tuple composed by the distance to the object and the angle between both objects

    
`distance_vehicle(waypoint, vehicle_transform)`
:   Returns the 2D distance from a waypoint to a vehicle
    
        :param waypoint: actual waypoint
        :param vehicle_transform: transform of the target vehicle

    
`draw_trajetory_points(world, waypoints, z=0.25, color=<carla.libcarla.Color object>, lt=5, size=0.1)`
:   Draw a list of trajetory points
    :param size:
    :param lt:
    :param color:
    :param world:
    :param waypoints:
    :param z:
    :return:

    
`draw_waypoints(world, waypoints, z=0.5)`
:   Draw a list of waypoints at a certain height given in z.
    
        :param world: carla.world object
        :param waypoints: list or iterable container with the waypoints to draw
        :param z: height in meters

    
`get_acc(vehicle, meters=False)`
:   Compute speed of a vehicle in Km/h.
    
        :param meters: use m/s or km/h
        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h

    
`get_speed(vehicle, meters=False)`
:   Compute speed of a vehicle in Km/h.
    
        :param meters: use m/s or km/h
        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h

    
`is_within_distance(target_location, current_location, orientation, max_distance, d_angle_th_up, d_angle_th_low=0)`
:   Check if a target object is within a certain distance from a reference object.
    A vehicle in front would be something around 0 deg, while one behind around 180 deg.
    
        :param target_location: location of the target object
        :param current_location: location of the reference object
        :param orientation: orientation of the reference object
        :param max_distance: maximum allowed distance
        :param d_angle_th_up: upper thereshold for angle
        :param d_angle_th_low: low thereshold for angle (optional, default is 0)
        :return: True if target object is within max_distance ahead of the reference object

    
`is_within_distance_ahead(target_transform, current_transform, max_distance)`
:   Check if a target object is within a certain distance in front of a reference object.
    
    :param target_transform: location of the target object
    :param current_transform: location of the reference object
    :param orientation: orientation of the reference object
    :param max_distance: maximum allowed distance
    :return: True if target object is within max_distance ahead of the reference object

    
`positive(num)`
:   Return the given number if positive, else 0
    
        :param num: value to check

    
`vector(location_1, location_2)`
:   Returns the unit vector from location_1 to location_2
    
        :param location_1, location_2: carla.Location objects

#### opencda.core.common.v2x_manager
Communication manager for cooperation.

- **<font color="#f8805a">Classes</font>**

`V2XManager(cav_world, config_yaml)`
:   V2X Manager for platooning, cooperative perception and so on
    
    Construct class
    :param config_yaml: configuration yaml file

    ### Methods

    `add_platoon_blacklist(self, pmid)`
    :   Add an existing platoon to current blacklist
        :param pmid: platoon id
        :return:

    `get_platoon_front_rear(self)`
    :   Get the ego vehicle's front and rear cav in the platoon
        :return:

    `get_platoon_manager(self)`
    :   Retrieve the platoon manager the cav belongs to and the corresponding id
        :return:

    `get_platoon_status(self)`
    :   Retrive the FSM status for platooning application
        :return:

    `in_platoon(self)`
    :   Check whether the vehicle is inside the platoon
        :return: bool, flag indication whether in a platoon

    `match_platoon(self)`
    :   A naive way to find the best position to join a platoon
        :return:

    `set_platoon(self, in_id, platooning_object=None, platooning_id=None, leader=False)`
    :   Set platooning status
        :param platooning_object: platooning world that contains all platoon information todo: remove this later
        :param platooning_id: platoon id the cav belongs to
        :param in_id: the position in the platoon, etc. 0 represents leader and 1 represents the second position
        :param leader: indicate whether this cav is a leader in platoon
        :return:

    `set_platoon_front(self, vm)`
    :   Set the frontal vehicle to another vehicle
        :param vm: vehicle manager
        :return:

    `set_platoon_rear(self, vm)`
    :   Set the rear vehicle to another vehicle
        :param vm:
        :return:

    `set_platoon_status(self, status)`
    :   Set the cav to a different fsm status
        :param status: fsm status
        :return:

    `update_info(self, ego_pos, ego_spd)`
    :   Update all communication plugins with current localization info

#### opencda.core.common.vehicle_manager
Basic class of CAV.

- **<font color="#f8805a">Classes</font>**


`VehicleManager(vehicle, config_yaml, application, carla_map, cav_world)`
:   A class manager to embed different modules with vehicle together
    
    Construction class
    :param vehicle: carla actor
    :param config_yaml: a dictionary that contains the parameters of the vehicle
    :param application: application category, support:['single','platoon'] currently
    :param carla_map: Carla HD Map
    :param cav_world: CAV world object

    ### Methods

    `destroy(self)`
    :   Destroy the actor vehicle
        :return:

    `run_step(self, target_speed=None)`
    :   Execute one step of navigation.
        :return:

    `set_destination(self, start_location, end_location, clean=False, end_reset=True)`
    :   Wrapper function to set global route
        :param start_location:
        :param end_location:
        :param clean:
        :param end_reset:
        :return:

    `update_info(self)`
    :   Call perception and localization module to retrieve surrounding info an ego position.
        :return:

### opencda.core.plan 
The main planing module for OpenCDA. 

#### opencda.core.plan.behavior_agent
This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights,
traffic signs, and has different possible configurations.

- **<font color="#f8805a">Classes</font>**

`BehaviorAgent(vehicle, carla_map, config_yaml)`
:   A modulized version of BehaviorAgent
    
    Construct class
    :param vehicle: carla actor
    :param config_yaml: a dictionary containing all initialization params
    provide customized function under customize/controller

    ### Descendants

    * opencda.core.application.platooning.platoon_behavior_agent.PlatooningBehaviorAgent

    ### Methods

    `add_white_list(self, vm)`
    :   Add vehicle manager to
        Args:
            vm ():
        
        Returns:

    `car_following_manager(self, vehicle, distance, target_speed=None)`
    :   Module in charge of car-following behaviors when there's
        someone in front of us.
        
            :param target_speed:
            :param vehicle: car to follow
            :param distance: distance from vehicle
            :return control: carla.VehicleControl

    `collision_manager(self, rx, ry, ryaw, waypoint, adjacent_check=False)`
    :   This module is in charge of warning in case of a collision
        :param adjacent_check: whether it is a check for adjacent lane
        :param rx: x coordinates of plan path
        :param ry: y coordinates of plan path
        :param ryaw: yaw angle
        :param waypoint: current waypoint of the agent
        :return vehicle_state: True if there is a vehicle nearby, False if not
        :return vehicle: nearby vehicle
        :return distance: distance to nearby vehicle

    `get_local_planner(self)`
    :   return the local planner

    `lane_change_management(self)`
    :   Identify whether a potential hazard exits if operating lane change.
        Returns:
            bool: whether the lane change is dangerous

    `overtake_management(self, obstacle_vehicle)`
    :   Overtake behavior.
        :param obstacle_vehicle: the vehicle
        :return:

    `reroute(self, spawn_points)`
    :   This method implements re-routing for vehicles approaching its destination.
        It finds a new target and computes another path to reach it.
        
            :param spawn_points: list of possible destinations for the agent

    `run_step(self, target_speed=None, collision_detector_enabled=True, lane_change_allowed=True)`
    :   Execute one step of navigation
        :param collision_detector_enabled: whether to enable collision detection.
        :param target_speed:  a manual order to achieve certain speed.
        :param lane_change_allowed: whether lane change is allowed. This is passed from platoon behavior agent.
        :return: control: carla.VehicleControl

    `set_destination(self, start_location, end_location, clean=False, end_reset=True, clean_history=False)`
    :   This method creates a list of waypoints from agent's position to destination location
        based on the route returned by the global router.
        
            :param end_reset: indicates whether the new destination is a temporary destination
            :param start_location: initial position
            :param end_location: final position
            :param clean: boolean to clean the waypoint queue
            :param clean_history:

    `traffic_light_manager(self, waypoint)`
    :   This method is in charge of behaviors for red lights and stops.
        
        WARNING: What follows is a proxy to avoid having a car brake after running a yellow light.
        This happens because the car is still under the influence of the semaphore,
        even after passing it. So, the semaphore id is temporarely saved to
        ignore it and go around this issue, until the car is near a new one.
        
            :param waypoint: current waypoint of the agent

    `update_information(self, ego_pos, ego_speed, objects)`
    :   Update the perception and localization information to the behavior agent.
        Args:
            ego_pos (carla.Transform): ego position from localization module.
            ego_speed (float): km/h, ego speed.
            objects (dictionary): Objects detection results from perception module.

    `white_list_match(self, obstacles)`
    :   Match the detected obstacles with the white list. Remove the obstacles that are in white list.
        The white list contains all position of target platoon member for joining.
        Args:
            obstacles (list):  a list of carla.Vehicle or ObstacleVehicle
        
        Returns:
            (list): the new list of obstacles.

#### opencda.core.plan.collision_check
This module is used to check collision possibility.
- **<font color="#f8805a">Classes</font>**


`CollisionChecker(time_ahead=1.2, circle_radius=1.3, circle_offsets=None)`
:   Construction method
    :param time_ahead: how many seconds we look ahead in advance for collision check
    :param circle_offsets: the offset between collision checking circle and the trajectory point
    :param circle_radius: The radius of the collision checking circle

    ### Methods

    `adjacent_lane_collision_check(self, ego_loc, target_wpt, overtake, world)`
    :   Generate a straight line in the adjacent lane for collision detection during
        overtake/lane change. Todo: current version may not work well on curved lane
        Args:
            ego_loc (carla.Location): Ego Location.
            target_wpt (carla.Waypoint): the check point in the adjacent at a far distance.
            overtake (bool): indicate whether this is an overtake or normal lane change behavior.
            world (carla.World): CARLA Simulation world, used to draw debug lines.
        
        Returns:
            list: the x coordinates of the collision check line in the adjacent lane
            list: the y coordinates of the collision check line in the adjacent lane
            list: the yaw angle of the the collision check line in the adjacent lane

    `collision_circle_check(self, path_x, path_y, path_yaw, obstacle_vehicle, speed, adjacent_check=False)`
    :   Use circled collision check to see whether potential hazard on the forwarding path
        :param adjacent_check: always give full path for adjacent lane check
        :param speed: ego vehicle speed in m/s
        :param path_yaw: a list of yaw angles
        :param path_x: a list of x coordinates
        :param path_y: a loist of y coordinates
        :param obstacle_vehicle: potention hazard vehicle on the way
        :return:

    `is_in_range(self, ego_pos, target_vehicle, candidate_vehicle, carla_map)`
    :   Check whether there is a obstacle vehicle between target_vehicle and ego_vehicle during back_joining
        :param carla_map: carla map
        :param ego_pos: Ego vehicle position
        :param target_vehicle: The vehicle that is suppose to be catched
        :param candidate_vehicle: The possible obstacle vehicle blocking the ego vehicle and target vehicle
        :return:

#### opencda.core.plan.drive_profile_plotting
Tools to plot velocity, acceleration, curvation.

- **<font color="#f8805a">Functions</font>**

`draw_acceleration_profile_single_plot(acceleration)`
:   Draw velocity profiles in a single plot
    :param acceleration:
    :return:

    
`draw_dist_gap_profile_singel_plot(gap_list)`
:   Draw distance gap profiles in a single plot
    :param gap_list: time gap
    :return:

    
`draw_sub_plot(velocity_list, acceleration_list, time_gap_list, distance_gap_list)`
:   This is a specific function that draws 4 in 1 images for trajectory following task
    :param velocity_list:
    :param distance_gap_list:
    :param time_gap_list:
    :param acceleration_list:
    :return:

    
`draw_time_gap_profile_singel_plot(gap_list)`
:   Draw inter gap profiles in a single plot
    :param gap_list: time gap
    :return:

    
`draw_ttc_profile_single_plot(ttc_list)`
:   Draw ttc.
    :param ttc_list: ttc
    :return:

    
`draw_velocity_profile_single_plot(velocity_list)`
:   Draw velocity profiles in a single plot
    :param velocity_list:
    :return:

    
`dump_data(data)`
:   Dump data to json file
    :param data: dictionary containing all stats
    :return:

#### opencda.core.plan.global_route_planner
This module provides GlobalRoutePlanner implementation.

- **<font color="#f8805a">Classes</font>**

`GlobalRoutePlanner(dao)`
:   This class provides a very high level route plan.
    Instantiate the class by passing a reference to
    A GlobalRoutePlannerDAO object.
    
    Constructor

    ### Methods

    `abstract_route_plan(self, origin, destination)`
    :   The following function generates the route plan based on
        origin      : carla.Location object of the route's start position
        destination : carla.Location object of the route's end position
        return      : list of turn by turn navigation decisions as
        agents.navigation.local_planner.RoadOption elements
        Possible values are STRAIGHT, LEFT, RIGHT, LANEFOLLOW, VOID
        CHANGELANELEFT, CHANGELANERIGHT

    `setup(self)`
    :   Performs initial server data lookup for detailed topology
        and builds graph representation of the world map.

    `trace_route(self, origin, destination)`
    :   This method returns list of (carla.Waypoint, RoadOption)
        from origin to destination

#### opencda.core.plan.global_route_planner_dao
This module provides implementation for GlobalRoutePlannerDAO.

- **<font color="#f8805a">Classes</font>**

`GlobalRoutePlannerDAO(wmap, sampling_resolution)`
:   This class is the data access layer for fetching data
    from the carla server instance for GlobalRoutePlanner
    
    Constructor method.
    
        :param wmap: carla.world object
        :param sampling_resolution: sampling distance between waypoints

    ### Methods

    `get_resolution(self)`
    :   Accessor for self._sampling_resolution

    `get_topology(self)`
    :   Accessor for topology.
        This function retrieves topology from the server as a list of
        road segments as pairs of waypoint objects, and processes the
        topology into a list of dictionary objects.
        
            :return topology: list of dictionary objects with the following attributes
                entry   -   waypoint of entry point of road segment
                entryxyz-   (x,y,z) of entry point of road segment
                exit    -   waypoint of exit point of road segment
                exitxyz -   (x,y,z) of exit point of road segment
                path    -   list of waypoints separated by 1m from entry
                            to exit

    `get_waypoint(self, location)`
    :   The method returns waypoint at given location
        
            :param location: vehicle location
            :return waypoint: generated waypoint close to location

#### opencda.core.plan.local_planner_behavior
This module contains a local planner to perform low-level waypoint following based on PID controllers.

- **<font color="#f8805a">Classes</font>**

`LocalPlanner(agent, carla_map, config_yaml)`
:   LocalPlanner implements the basic behavior of following a trajectory
    of waypoints that is generated on-the-fly.
    The low-level motion of the vehicle is computed by using two PID controllers,
    one is used for the lateral control
    and the other for the longitudinal control (cruise speed).
    
    When multiple paths are available (intersections)
    this local planner makes a random choice.
    
    :param agent: agent that regulates the vehicle
    :param config_yaml: local planner configuration file

    ### Methods

    `generate_path(self)`
    :   Generate the smooth path using cubic spline
        :return: rx, ry, ryaw, rk: list of planned path points' x,y coordinates, yaw angle and curvature

    `generate_trajectory(self, rx, ry, rk)`
    :   Sampling the generated path and assign speed to each point
        :param rx: x coordinates of planning path
        :param ry: y coordinates of planning path
        :param rk: curvature of planning path
        :param debug: whether to draw the whole plan path
        :return:

    `get_trajetory(self)`
    :   Get the trajetory
        :return:

    `pop_buffer(self, vehicle_transform)`
    :   Remove waypoints achieved
        :return:

    `run_step(self, rx, ry, rk, target_speed=None, trajectory=None, following=False)`
    :   Execute one step of local planning which involves
        running the longitudinal and lateral PID controllers to
        follow the smooth waypoints trajectory.
        
            :param rx: generated path x coordinates
            :param ry: generated path y coordinates
            :param rk: generated path curvatures
            :param following: whether the vehicle is under following status
            :param trajectory: pre-generated trajectory only for following vehicles in the platooning
            :param target_speed: desired speed
            :return: next trajectory point's target speed and waypoint

    `set_global_plan(self, current_plan, clean=False)`
    :   Sets new global plan.
        
            :param clean:
            :param current_plan: list of waypoints in the actual plan

    `update_information(self, ego_pos, ego_speed)`
    :   Update the ego position and speed for trajectory planner.
        Args:
            ego_pos (carla.Transform): Ego position from localization module.
            ego_speed (float): Ego speed(km/h) from localization module.
        
        Returns:

`RoadOption(value, names=None, *, module=None, qualname=None, type=None, start=1)`
:   RoadOption represents the possible topological configurations
    when moving from a segment of lane to other.

    ### Ancestors (in MRO)

    * enum.Enum

    ### Class variables

    `CHANGELANELEFT`
    :

    `CHANGELANERIGHT`
    :

    `LANEFOLLOW`
    :

    `LEFT`
    :

    `RIGHT`
    :

    `STRAIGHT`
    :

    `VOID`
    :

#### opencda.core.plan.planer_debug_helper
Analysis + Visualization functions for planning.

- **<font color="#f8805a">Classes</font>**

`PlanDebugHelper(actor_id)`
:   This class aims to save statistics for planner behaviour
    Attributes:
        speed_list (list): The list containing speed info(m/s) of all time-steps
        acc_list(list): The list containing acceleration info(m^2/s) of all time-steps
        ttc_list(list): The list containing ttc info(s) for all time-steps
        count(int): Used to count how many simulation steps have been executed.

    ### Descendants

    * opencda.core.application.platooning.platoon_debug_helper.PlatoonDebugHelper

    ### Methods

    `evaluate(self)`
    :

    `update(self, ego_speed, ttc)`
    :   Update the speed info.
        Args:
            ego_speed(km/h): Ego speed.
            ttc(s): time to collision.
        Returns:

#### opencda.core.plan.spline
Cubic spline planner.

- **<font color="#f8805a">Functions</font>**

`calc_spline_course(x, y, ds=0.1)`
:   Calculate the spline based on x,y location and delta time ds.

- **<font color="#f8805a">Classes</font>**

`Spline(x, y)`
:   Cubic Spline class

    ### Methods

    `calc(self, t)`
    :   Calc position
        
        if t is outside of the input x, return None

    `calcd(self, t)`
    :   Calc first derivative
        
        if t is outside of the input x, return None

    `calcdd(self, t)`
    :   Calc second derivative

`Spline2D(x, y)`
:   2D Cubic Spline class

    ### Methods

    `calc_curvature(self, s)`
    :   calc curvature

    `calc_position(self, s)`
    :   calc position

    `calc_yaw(self, s)`
    :   calc yaw


### opencda.core.sensing 
This is the sensing stack module of the OpenCDA.

#### opencda.core.sensing.localization.
The localization functions are implemented in this class. 

##### opencda.core.senesing.localization.coordinate_transform
Functions to transfer coordinates under different coordinate system.

- **<font color="#f8805a">Functions</font>**

`geo_to_transform(lat, lon, alt, lat_0, lon_0, alt_0)`
:   Convert WG84 to ENU. The origin of the ENU should pass the geo reference.
    Note this function is a writen by reversing the official API transform_to_geo.
    :param lat: current latitude
    :param lon: current longitude
    :param alt: current altitude
    :param lat_0: geo_ref latitude
    :param lon_0: geo_ref longitude
    :param alt_0: geo_ref altitude
    :return:

##### opencda.core.sensing.localization.extented_kalman_filter
Use Extended Kalman Filter on GPS + IMU for better localization.

- **<font color="#f8805a">Classes</font>**

`ExtentedKalmanFilter(dt)`
:   Kalman Filter implementation for gps + imu
    
    Construct class
    Args:
        dt(float): unit time step for simulation.

    ### Methods

    `jacob_f(self, x, u)`
    :   Jacobian of Motion Model motion model
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}
        so
        dx/dyaw = -v*dt*sin(yaw)
        dx/dv = dt*cos(yaw)
        dy/dyaw = v*dt*cos(yaw)
        dy/dv = dt*sin(yaw)

    `motion_model(self, x, u)`
    :   Predict current position and yaw based on previous result.
        X = F * X_prev + B * u
        Args:
            x (np.array): [x_prev, y_prev, yaw_prev, v_prev], shape: (4, 1).
            u (np.array): [v_current, imu_yaw_rate], shape:(2, 1).
        
        Returns:
          np.array: predicted state.

    `observation_model(self, x)`
    :   Project the state matrix to sensor measurement matrix.
        Args:
            x (np.array): [x, y, yaw, v], shape: (4. 1).
        
        Returns:
            np.array: predicted measurement.

    `run_step(self, x, y, heading, velocity, yaw_rate_imu)`
    :   Apply EKF on current measurement and previous prediction
        :param x: x(esu) coordinate from gnss sensor at current timestamp
        :param y: y(esu) coordinate from gnss sensor at current timestamp
        :param heading: heading direction at current timestamp
        :param velocity: current speed
        :param yaw_rate_imu: yaw rate rad/s from IMU sensor
        :return: corrected x, y, heading, velocity

    `run_step_init(self, x, y, heading, velocity)`
    :   Initalization for states
        :param x:
        :param y:
        :param heading:
        :param velocity:
        :return:

##### opencda.core.sensing.localization.kalman_filter
Use Kalman Filter on GPS + IMU for better localization. Reference: https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

- **<font color="#f8805a">Classes</font>**

`KalmanFilter(dt)`
:   Kalman Filter implementation for gps + imu
    
    Construct class
    Args:
        dt(float): unit time step for simulation.

    ### Methods

    `motion_model(self, x, u)`
    :   Predict current position and yaw based on previous result.
        X = F * X_prev + B * u
        Args:
            x (np.array): [x_prev, y_prev, yaw_prev, v_prev], shape: (4, 1).
            u (np.array): [v_current, imu_yaw_rate], shape:(2, 1).
        
        Returns:
          np.array: predicted state.

    `observation_model(self, x)`
    :   Project the state matrix to sensor measurement matrix.
        Args:
            x (np.array): [x, y, yaw, v], shape: (4. 1).
        
        Returns:
            np.array: predicted measurement.

    `run_step(self, x, y, heading, velocity, yaw_rate_imu)`
    :   Apply KF on current measurement and previous prediction
        :param x: x(esu) coordinate from gnss sensor at current timestamp
        :param y: y(esu) coordinate from gnss sensor at current timestamp
        :param heading: heading direction at current timestamp
        :param velocity: current speed
        :param yaw_rate_imu: yaw rate rad/s from IMU sensor
        :return: corrected x, y, heading

    `run_step_init(self, x, y, heading, velocity)`
    :   Initial state filling.
        Args:
            x ():
            y ():
            heading ():
            velocity ():
        
        Returns:

##### opencda.core.sensing.localization.localization_debug_helper
Visualization tools for localization results.

- **<font color="#f8805a">Classes</font>**

`LocDebugHelper(config_yaml, actor_id)`
:   This class aims to help users debugging their localization algorithms.
    
    Users can apply this class to draw the x, y coordinate trajectory, yaw angle
     and vehicle speed from GNSS raw measurements, Kalman filter(or any other filter),
     and the groundtruth measurements. Error plotting is also enabled.
    
    Attributes:
        show_animation (bool):
        x_scale(float):
        y_scale(float):
    
    Args:
        config_yaml (dict):
        actor_id(int):

    ### Methods

    `evaluate(self)`
    :   Plot the localization related data points.
        Args:
        
        Returns:

    `run_step(self, gnss_x, gnss_y, gnss_yaw, gnss_spd, filter_x, filter_y, filter_yaw, filter_spd, gt_x, gt_y, gt_yaw, gt_spd)`
    :   Run a single step for DebugHelper to save and animate(optional) the localization data.
        Args:
            gnss_x (float):
            gnss_y (float):
            gnss_yaw (float):
            gnss_spd (float):
            filter_x (float):
            filter_y (float):
            filter_yaw (float):
            filter_spd (float):
            gt_x (float):
            gt_y (float):
            gt_yaw (float):
            gt_spd ()float:
        
        Returns:

##### opencda.core.sensing.localization.localization_manager
Manager class for localization module.

- **<font color="#f8805a">Classes</font>**

`GnssSensor(vehicle, config)`
:   Class for gnss sensors
    
    Construct class
    :param vehicle: carla actor
    :param config: gnss configuration

`ImuSensor(vehicle)`
:   IMU Sensor
    
    Construct class
    :param vehicle: Carla Actor

`LocalizationManager(vehicle, config_yaml, carla_map)`
:   The core class that manages localization estimation.
    
    Construction class
    :param vehicle: carla actor
    :param config_yaml: configuration related to localization

    ### Methods

    `add_heading_direction_noise(self, heading_direction)`
    :   Add synthetic noise to heading direction.
        :param heading_direction: groundtruth heading_direction obtained from the server.
        :return: heading direction with noise.

    `add_speed_noise(self, speed)`
    :   Add gaussian white noise to the current speed.
        Args:
            speed (float): m/s, current speed.
        
        Returns:
            float: the speed with noise added.

    `destroy(self)`
    :   Destroy the sensors
        :return:

    `get_ego_pos(self)`
    :   Retrieve ego vehicle position
        :return: vehicle position

    `get_ego_spd(self)`
    :   Retrieve ego vehicle speed
        :return:

    `localize(self)`
    :   Currently implemented in a naive way.
        :return:

#### opencda.core.sensing.perception
The perception functions are implemented in this class. 

##### opencda.core.senesing.perception.o3d_lidar_libs
Utility functions for 3d lidar visualization and processing by utilizing open3d.

- **<font color="#f8805a">Functions</font>**

`o3d_camera_lidar_fusion(objects, yolo_bbx, lidar_3d, projected_lidar, lidar_sensor)`
:   Utilize the 3D lidar points to extend the 2D bounding box from camera to 3D bounding box under world coordinates.
    Args:
        objects (dict): The dictionary contains all object detection result.
        yolo_bbx (torch.Tensor): Object detection bounding box at current photo from yolov5,
                                 shape:(n, [x1, y1, x2, y2, label]).
        lidar_3d (np.ndarray): Raw 3D lidar points in lidar coordinate system.
        projected_lidar (np.ndarray): 3D lidar points projected to the camera space.
        lidar_sensor (carla.Sensor): The lidar sensor.
    
    Returns:
        objects: dict
            The update object dictionary that contains 3d bounding boxes.

    
`o3d_pointcloud_encode(raw_data, point_cloud)`
:   Encode the raw point cloud to Open3d PointCloud object.
    Args:
        raw_data (np.ndarray): Raw lidar points (N, (x, y, z, i)) obtained from lidar sensor.
        point_cloud (o3d.PointCloud):  Open3d PointCloud.
    
    Returns:
        (o3d.PointCloud): PointCloud with added points.

    
`o3d_visualizer_init(actor_id)`
:   Initialize the visualizer.
    Args:
        actor_id (int): Vehicle's id.
    Returns:
        (o3d.visualizer): Initialize Open3d visualizer.

    
`o3d_visualizer_show(vis, count, point_cloud, objects)`
:   Visualize the point cloud at runtime.
    Args:
        vis (o3d.Visualizer): Visualization interface.
        count (int): current step since simulation started.
        point_cloud (o3d.PointCLoud): Open3d point clouds.
        objects (dict): The dictionary containing objects.
    Returns:

##### opencda.core.senesing.perception.obstacle_vehicle
Obstacle vehicle class to save object detection results.

- **<font color="#f8805a">Functions</font>**

`is_vehicle_cococlass(label)`
:   Check whether the label belongs to the vehicle class according to coco dataset.
    Args:
        label(int):
    
    Returns:
        is_vehicle: bool
            whether this label belongs to the vehicle class

- **<font color="#f8805a">Classes</font>**

`BoundingBox(corners)`
:   Bounding box class for obstacle vehicle.
    
    Construct class.
    Args:
        corners (nd.nparray): Eight corners of the bounding box. shape:(8, 3)

`ObstacleVehicle(corners, o3d_bbx, vehicle=None, lidar=None)`
:   A class for obstacle vehicle. The attributes are designed to match with carla.Vehicle class
    
    Construct class.
    Args:
        corners (nd.nparray): Eight corners of the bounding box. shape:(8, 3).
        o3d_bbx (open3d.AlignedBoundingBox): The bounding box object in Open3d. This is mainly used for
        visualization.
        vehicle(carla.Vehicle): carla.Vehicle object.
        lidar(carla.sensor.lidar): lidar sensor.

    ### Methods

    `get_location(self)`
    :

    `get_transform(self)`
    :

    `get_velocity(self)`
    :

    `set_vehicle(self, vehicle, lidar)`
    :   Assign the attributes from carla.Vehicle to ObstacleVehicle
        Args:
            vehicle(carla.Vehicle): carla.Vehicle object.
            lidar(carla.sensor.lidar): lidar sensor, used to project world coordinates to sensor coordinates.
        Returns:

    `set_velocity(self, velocity)`
    :   Set the velocity of the vehicle.
        Args:
            velocity(carla.Vector3D): velocity in 3d vector format.
        
        Returns:

`StaticObstacle(corner, o3d_bbx)`
:   Currently, we regard all static obstacles such as stop signs and traffic light as the same class.
    
    Construct class.
    Args:
        corner (nd.nparray): Eight corners of the bounding box. shape:(8, 3)
        o3d_bbx (open3d.AlignedBoundingBox): The bounding box object in Open3d. This is mainly used for
        visualization.

##### opencda.core.senesing.perception.perception_manager
Manager class for the perception module.

- **<font color="#f8805a">Classes</font>**


`CameraSensor(vehicle, position='front')`
:   Class for rgb camera.
    
    Construct class.
    Args:
        vehicle (carla.Vehicle): Carla actor.
        position (string): the camera mounted position, only front, left and right supported.

`LidarSensor(vehicle, config_yaml)`
:   Lidar sensor manager.
    
    Construct class.
    Args:
        vehicle (carla.Vehicle): The attached vehicle.
        config_yaml (dict): Configuration for lidar.

`PerceptionManager(vehicle, config_yaml, ml_manager)`
:   Perception manager mainly for object detection
    
    Construct class.
    Args:
        vehicle (carla.Actor): The carla vehicle.
        config_yaml (dict):  The configuration yaml dictionary.
        ml_manager(MlManager): Machine learning manager from CAV World.

    ### Methods

    `activate_mode(self, objects)`
    :   Use Yolov5 + Lidar fusion to detect objects.
        Args:
            objects(dict): object dictionary
        
        Returns:
            objects: dict
                The updated object dictionary.

    `deactivate_mode(self, objects)`
    :   Obstacle detection under perception deactivation mode.
        Args:
            objects(dict): object dictionary
        Returns:

    `destroy(self)`
    :   Destroy sensors.
        Returns:

    `detect(self, ego_pos)`
    :   Detect surrounding objects. Currently only vehicle detection supported.
        Args:
            ego_pos (carla.Transform): Vehicle ego position
        
        Returns:
            List of carla.Vehicle or ObstacleVehicle

    `dist(self, v)`
    :   A fast method to retrieve the obstable distance the ego vehicle from the server directly.
        Args:
            v (carla.vehicle):
        
        Returns:
            float: distance

    `speed_retrieve(self, objects)`
    :   We don't implement any obstacle speed calculation algorithm. The speed will be retrieved from
        the server directly.
        Args:
            objects(dict): The dictionary contains the objects.
        
        Returns:

    `visualize_3d_bbx_front_camera(self, objects, rgb_image)`
    :   Visualize the 3d bounding box on frontal camera image.
        Args:
            objects (dict): a dictionary containing all detected objects.
            rgb_image (np.ndarray):camera image.
        
        Returns:

##### opencda.core.senesing.perception.sensor_transformation
Class that contains the transformations between world and different sensors.

- **<font color="#f8805a">Variables</font>**

`VID_RANGE`
:   Part 1: Camera Related Transformation

- **<font color="#f8805a">Functions</font>**

`bbx_to_world(cords, vehicle)`
:   Convert bounding box coordinate at vehicle reference to world reference.
    Args:
        cords (np.ndarray): Bounding box coordinates with 8 vertices.
        vehicle (carla.vehicle or ObstacleVehicle): vehicle object.
    
    Returns:
        bb_world_cords: np.ndarray
            Bounding box coordinates under word reference.

    
`create_bb_points(vehicle)`
:   Extract the eight vertices of the bounding box from the vehicle.
    Args:
        vehicle (carla.Vehicle or ObstacleVehicle):
    
    Returns:
        (np.ndarray): 3d bounding box.

    
`get_2d_bb(vehicle, sensor, senosr_transform)`
:   Summarize 2D bounding box creation
    Args:
         vehicle (carla.vehicle or ObstacleVehicle): vehicle object.
         sensor (carla.sensor.camera.rgb): The CARLA sensor object.
         senosr_transform (carla.Transform): sensor position in the world
    
    Returns:
        (np.ndarray): 2d bounding box in camera image

    
`get_bounding_box(vehicle, sensor, sensor_transform)`
:   Get vehicle bounding box and project to sensor image
    Args:
         vehicle (carla.vehicle or ObstacleVehicle): vehicle object.
         sensor (carla.sensor.camera.rgb): The CARLA sensor object.
         sensor_transform (carla.Transform): sensor position in the world
    
    Returns:
         (np.ndarray): Bounding box coordinates in sensor image.

    
`get_camera_intrinsic(sensor)`
:   Retrieve the camera intrinsic matrix
    Args:
        sensor (carla.sensor.camera.rgb): The CARLA sensor object.
    
    Returns:
        np.ndarray: 2D intrinsic matrix

    
`p3d_to_p2d_bb(p3d_bb)`
:   Draw 2D bounding box (4 vertices) from 3D bounding box (8 vertices) in image.
    2D bounding box is represented by two corner points
    Args:
        p3d_bb ():
    
    Returns:

    
`project_lidar_to_camera(lidar, camera, point_cloud, rgb_image)`
:   Project lidar to camera space.
    Args:
        lidar (carla.Sensor): Lidar sensor.
        camera (carla.Sensor): Camera seonsor.
        point_cloud (np.ndarray): cloud points, (x, y, z, intensity).
        rgb_image (np.ndarray): rgb image from camera.
    
    Returns:
        (np.ndarray): new rgb image with lidar points projected.
        (np.ndarray): point clouds projected to camera space.

    
`sensor_to_world(cords, sensor_transform)`
:   Project 
    Args:
        cords (np.ndarray): Coordinates under sensor reference.
        sensor_transform (carla.Transform): sensor position in the world
    Returns:
        world_cords: np.ndarray
            Coordinates projected to world space.

    
`vehicle_to_sensor(cords, vehicle, sensor_transform)`
:   Transform coordinates from vehicle reference to sensor reference
    Args:
        cords (np.ndarray): Coordinates under vehicle reference, shape (4, n)
        vehicle (carla.vehicle or ObstacleVehicle): vehicle object.
        sensor_transform (carla.Transform): sensor position in the world, shape(3, 1)
    
    Returns:
        (np.ndarray): Coordinates in sensor reference.

    
`world_to_sensor(cords, sensor_transform)`
:   Transform coordinate from world reference to sensor reference.
    Args:
        cords (np.ndarray): Coordinates under world reference, shape:(4, n).
        sensor_transform (carla.Transform): sensor position in the world, shape:(3, 1).
    
    Returns:
        sensor_cords: np.ndarray
            Coordinates in sensor reference.

    
`x_to_world_transformation(transform)`
:   Get the transformation matrix from x(it can be vehicle or sensor) coordinates to world coordinate.
    Args:
        transform (carla.Transform): The transform that contains location and rotation.
    
    Returns:
        matrix: np.ndarray
            The transformation matrix

## OpenCDA Module: customize
This is the module that handles customized content (i.e., controllers, learning methods, etc.).

### opencda.customize.ml_libs
The manager class for machine learning libraries.

#### opencda.customize.ml_libs.ml_manager
Since multiple CAV normally use the same ML/DL model, here we have this class to enable different
CAVs share the same model to avoid duplicate memory consumption.

- **<font color="#f8805a">Variables</font>**

`MLManager()`
:   A class that should contain all the ML models you want to initialize.
    
    Construction class.

    ### Methods

    `draw_2d_box(self, result, rgb_image, index)`
    :   Draw 2d bounding box based on the yolo detection.
        Args:
            result (yolo.Result):Detection result from yolo 5.
            rgb_image (np.ndarray): Camera rgb image.
            index(int): Indicate the index
        
        Returns:
            (np.ndarray): camera image with bbx drawn.

## OpenCDA Module: scenario_testing
This is the module that regulates all the scenario related python scripts, utility functions and configuration files. 
Scenario defination scripts locate at this directory. 

### config_yaml 

- **<font color="#f8805a">Directory</font>**

This is the directory that contains configuration files for each scenario. 

### opencda.scenario_testing.evaluations
Class for scenario evalution functions.

#### opencda.scenario_testing.evaluations.evaluate_manager
The manager class for scenario evaluation.

- **<font color="#f8805a">Class</font>**

`EvaluationManager(cav_world)`
:   Evaluation manager to manage the analysis of the results for different modules.
        
    
    Construct class
    Args:
        cav_world (opencda.CavWorld): The CavWorld object that contains all CAVs' information

    ### Methods

    `evaluate(self)`
    :   Evaluate performance of all modules by plotting and writing the statistics into the log file.
        Returns:

    `kinematics_eval(self, log_file)`
    :   vehicle kinematics related evaluation.
        Args:
            log_file (File): The log file to write the data.
        
        Returns:

    `localization_eval(self, log_file)`
    :   Localization module evaluation.
        Args:
            log_file (File): The log file to write the data.
        
        Returns:

    `platooning_eval(self, log_file)`
    :   Platooning evaluation.
        Args:
            log_file (File): The log file to write the data.
        
        Returns:

#### opencda.scenario_testing.evaluations.evaluate_manager
Utility functions for evaluation.

- **<font color="#f8805a">Functions</font>**

`lprint(logfile, *argv)`
:   Save string to log file.
    Args:
        logfile (File): The log file path.
        *argv (string or number): the string that needs to be saved into the log file.
    
    Returns:

### opencda.scenario_testing.utils
Module for scenario related utility functions.

#### opencda.scenario_testing.utils.customized_map_api
Loading world from customized map

- **<font color="#f8805a">Functions</font>**

`load_customized_world(xodr_path, client)`
:   Load .xodr file and return the carla world object
    :param xodr_path: path to the xodr file
    :param client: created client
    :return:

    
`spawn_helper_2lanefree(carla_map, coefficient)`
:   A helper function to locate the valid spawn point on the merge lane.
    :param carla_map: the 2lanefreeway map
    :param coefficient: a single scalar indicating where is the spawn point, eg. 0.5 represents the spawn position
    is in the middle of the merge lane
    :return: carla transform

    
`spawn_helper_2lanefree_complete(carla_map, coefficient)`
:   A helper function to locate the valid spawn point on the merge lane.
    :param carla_map: the 2lanefreeway map
    :param coefficient: a single scalar indicating where is the spawn point, eg. 0.5 represents the spawn position
    is in the middle of the merge lane
    :return: carla transform

- **<font color="#f8805a">Classes</font>**
`bcolors()`
:   

    ### Class variables

    `BOLD`
    :

    `ENDC`
    :

    `FAIL`
    :

    `HEADER`
    :

    `OKBLUE`
    :

    `OKCYAN`
    :

    `OKGREEN`
    :

    `UNDERLINE`
    :

    `WARNING`
    :

#### opencda.scenario_testing.utils.sim_api
Simulation API for create simulation world, vehicle manager and so on.

- **<font color="#f8805a">Functions</font>**

`car_blueprint_filter(blueprints)`
:   Filter out the uncommon vehicles
    :return:

    
`createPlatoonManagers(world, carla_map, scenario_params, apply_ml, map_helper=None)`
:   Create Platooning Managers based on given params.
    Args:
    world (carla.World): World from CARLA simulator.
    carla_map (carla.Map): Map obtained from CARLA server.
    scenario_params (dict): Platoon paramters.
    apply_ml (bool): whether ml/dl model is included. Pytorch/sklearn required to install if set to true.
    map_helper (function): Specific function to convert certain parameters to spawn position in certain map.
    
    Returns:
        platoon_list list (list)
        cav_world (carla.World)

    
`createSimulationWorld(simulation_config, xodr_path=None, town=None)`
:   Create client and simulation world
    :param simulation_config: configuration dictionary for simulation
    :param xodr_path: optional, used only when customized map needed
    :param town: default town name if not using customized map, eg. 'Town06'
    :return: client, simulation world, origin setting

    
`createTrafficManager(client, world, traffic_config)`
:   Create background traffic
    :param client:
    :param world:
    :param traffic_config:
    :return:

    
`createVehicleManager(world, scenario_params, application, cav_world, carla_map, map_helper=None)`
:   Create single CAV manager
    :param world: simulation world
    :param scenario_params: scenario configuration
    :param application: the application purpose, a list, eg. ['single']
    :param cav_world: object containing all cav info
    :param carla_map: carla HD Map
    :param map_helper: A function used for conveniently set the spawn position depending on different maps
    :return: a list of vehicle managers

    
`destroyActors(world)`
:   Destroy all actors in the world
    :param world:
    :return:

#### opencda.scenario_testing.utils.yaml_utils
Used to load and write yaml files.

- **<font color="#f8805a">Functions</font>**

`load_yaml(file)`
:   load yaml file and return a dictionary
    :param file: yaml file path
    :return: a dictionary that contains defined parameters

---




**Note:**
The current OpenCDA Module is developed specificlly for OpenCDA version 1.0, please refer to the corresponding software version.
