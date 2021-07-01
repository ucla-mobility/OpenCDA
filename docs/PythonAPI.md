# opencda.opencda_carla
Description for this module: Used to reduce the dependency on CARLA api by mocking them in the same structure.

## Location(x: float = 0, y: float = 0, z: float = 0)
Stores a 3D location, and provides useful helper methods.

**Arguments:**

### Attributes:
### Ancestors 
opencda.opencda_carla.Vector3D

- <font color="#7fb800">from_simulator_location</font> (location)
 
Creates a pylot Location from a simulator location.

**Arguments:**

**Returns**

## Rotation(pitch: float = 0, yaw: float = 0, roll: float = 0)
Used to represent the rotation of an actor or obstacle.

**Arguments:**

### Attributes:
- <font color="#7fb800">from_simulator_rotation</font> (rotation)
 
Creates a pylot Rotation from a simulator rotation.

**Arguments:**

*rotation:*  An instance of a simulator rotation.

**Returns**

## Transform(location: opencda.opencda_carla.Location = None, rotation: opencda.opencda_carla.Rotation = None)
A class that stores the location and rotation of an obstacle.

**Arguments:**

### Attributes:
- <font color="#7fb800">from_simulator_transform</font> (transform)
 
Creates a pylot transform from a simulator transform.

**Arguments:**

*transform:*  A simulator transform.

**Returns**

## Vector3D(x: float = 0, y: float = 0, z: float = 0)
Represents a 3D vector and provides useful helper functions.

**Arguments:**

### Attributes:
- <font color="#7fb800">from_simulator_vector</font> (vector)
 
Creates a pylot Vector3D from a simulator 3D vector.

**Arguments:**

**Returns**

# opencda.core.plan.drive_profile_plotting
Description for this module: Tools to plot velocity, acceleration, and curvation.

## draw_acceleration_profile_single_plot(acceleration)
Draw velocity profiles in a single plot.

**Arguments:**

## draw_dist_gap_profile_singel_plot(gap_list)
Draw distance gap profiles in a single plot.

**Arguments:**

## draw_sub_plot(velocity_list, acceleration_list, time_gap_list, distance_gap_list, ttc_list)
This is a specific function that draws 4 in 1 images for trajectory following task. 

**Arguments:**

## draw_time_gap_profile_singel_plot(gap_list)
Draw inter gap profiles in a single plot.

**Arguments:**

## draw_ttc_profile_single_plot(ttc_list)
Draw ttc.

**Arguments:**

## draw_velocity_profile_single_plot(velocity_list)
Draw velocity profiles in a single plot.

**Arguments:**

# opencda.core.plan.collision_check
Description for this module: This module is used to check collision possibility

## CollisionChecker(time_ahead=1.2, circle_radius=1.3, circle_offsets=None)
The default collision checker module.

### Parameters
- <font color="#f8805a">time_ahead</font> (float)
 
how many seconds we look ahead in advance for collision check.


- <font color="#f8805a">circle_radius</font> (float)
 
The radius of the collision checking circle.


- <font color="#f8805a">circle_offsets</font> (float)
 
The offset between collision checking circle and the trajectory point.


### Methods 
- <font color="#7fb800">adjacent_lane_collision_check</font> (self, ego_loc, target_wpt, overtake, world)
 
Generate a straight line in the adjacent lane for collision detection during

**Arguments:**

*ego_loc (carla.Location):*  Ego Location.

*target_wpt (carla.Waypoint):*  the check point in the adjacent at a far distance.

*overtake (bool):*  indicate whether this is an overtake or normal lane change behavior.

*world (carla.World):*  CARLA Simulation world, used to draw debug lines.

**Returns**

*rx (list):*  the x coordinates of the collision check line in the adjacent lane

*ry (list):*  the y coordinates of the collision check line in the adjacent lane

*ryaw (list):*  the yaw angle of the the collision check line in the adjacent lane

- <font color="#7fb800">collision_circle_check</font> (self, path_x, path_y, path_yaw, obstacle_vehicle, speed, adjacent_check=False)
 
Use circled collision check to see whether potential hazard on the forwarding path.

**Arguments:**

*adjacent_check (boolean):*  Indicator of whether do adjacent check. Note: always give full path for adjacent lane check.

*speed (float):*  ego vehicle speed in m/s.

*path_yaw (float):*  a list of yaw angles

*path_x (list):*  a list of x coordinates

*path_y (list):*  a list of y coordinates

*obstacle_vehicle (carla.vehicle):*  potention hazard vehicle on the way

**Returns**

*collision_free (boolean):*  Flag indicate whether the current range is collision free.

- <font color="#7fb800">is_in_range</font> (self, ego_pos, target_vehicle, candidate_vehicle, carla_map)
 
Check whether there is a obstacle vehicle between target_vehicle and ego_vehicle during back_joining.

**Arguments:**

*carla_map (carla.map):*  carla map  of the current simulation world.

*ego_pos (carla.transform):*  Ego vehicle position.

*target_vehicle (carla.vehicle):*  The vehicle that is suppose to be catched.

*candidate_vehicle (carla.vehicle):*  The possible obstacle vehicle blocking the ego vehicle and target vehicle.

**Returns**

*detection result (boolean):*  Indicator of whther the target vehicle is in range

# opencda.core.plan.spline
Description for this module: Cubic spline planner

## calc_spline_course(x, y, ds=0.1)
Caculate 2D splice course.

**Arguments:**

**Returns**

## main()
Main function to calculate spline and visulize the results.

## Spline(x, y)
Cubic Spline class for calculte curvature (Author: Atsushi Sakai(@Atsushi_twi)).

### Parameters
- <font color="#f8805a">x</font> (float)
 
The x coordinate.


- <font color="#f8805a">y</font> (float)
 
The y coordinate.


### Attributes
- <font color="#f8805a">b</font> (float)
 
The spline coefficient b.


- <font color="#f8805a">c</font> (float)
 
The spline coefficient c.


- <font color="#f8805a">d</font> (float)
 
The spline coefficient d.


- <font color="#f8805a">w</font> (float)
 
The spline coefficient w.


- <font color="#f8805a">nx</font> (float)
 
The dimension of x.


- <font color="#f8805a">h</font> (float )
 
The n-th discrete difference along the x-axis.


### Methods 
- <font color="#7fb800">calc</font> (self, t)
 
Calc position

**Arguments:**

* t (float):*  if t is outside of the input x, return None

**Returns**

* result (float):*  The calcualtion result of position. If t is outside the range of x, return None.

- <font color="#7fb800">calcd</font> (self, t)
 
Calc first derivative. If t is outside of the input x, return None.

- <font color="#7fb800">calcdd</font> (self, t)
 
Calc second derivative, If t is outside of the input x, return None.

## Spline2D(x, y)
2D Cubic Spline class for calculte curvature (Author: Atsushi Sakai(@Atsushi_twi)).

### Parameters
- <font color="#f8805a">x</font> (float)
 
The x coordinate.


- <font color="#f8805a">y</font> (float)
 
The y coordinate.


### Attributes
- <font color="#f8805a">b</font> (float)
 
The spline coefficient b.


- <font color="#f8805a">c</font> (float)
 
The spline coefficient c.


- <font color="#f8805a">d</font> (float)
 
The spline coefficient d.


- <font color="#f8805a">w</font> (float)
 
The spline coefficient w.


- <font color="#f8805a">nx</font> (float)
 
The dimension of x.


- <font color="#f8805a">h</font> (float )
 
The n-th discrete difference along the x-axis.


### Methods 
- <font color="#7fb800">calc_curvature</font> (self, s)
 
Calculate curvature.

- <font color="#7fb800">calc_position</font> (self, s)
 
Calculate position.

- <font color="#7fb800">calc_yaw</font> (self, s)
 
Calculate yaw.
# opencda.core.plan.behavior_agent
Description for this module: This module implements an agent that roams around a track following random

## BehaviorAgent(vehicle, carla_map, config_yaml)
A modulized version of carla BehaviorAgent.

### Parameters
- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla.Vehicle. We need this class to spawn our gnss and imu sensor.


- <font color="#f8805a">carla_map</font> (carla.map)
 
The carla HD map for simulation world.


- <font color="#f8805a">config</font> (dict)
 
The configuration dictionary of the localization module.


### Attributes
- <font color="#f8805a">_ego_pos</font> (carla.position)
 
Posiion of the ego vehicle. 


- <font color="#f8805a">_ego_speed</font> (float )
 
Speed of the ego vehicle. 


- <font color="#f8805a">_map</font> (carla.map)
 
The HD map of the current simulation world.


- <font color="#f8805a">max_speed</font> (float)
 
The current speed limit of the ego vehicles.


- <font color="#f8805a">break_distance</font> (float)
 
The current distance needed for ego vehicle to reach a steady stop.


- <font color="#f8805a">_collision_check</font> (collisionchecker)
 
A collision check class to estimate the collision with front obstacle.


- <font color="#f8805a">ignore_traffic_light</font> (boolean)
 
Boolean indicator of whether to ignore traffic light.


- <font color="#f8805a">overtake_allowed</font> (boolean)
 
Boolean indicator of whether to allow overtake.


- <font color="#f8805a">_local_planner</font> (LocalPlanner)
 
A carla local planner class for behavior planning.


- <font color="#f8805a">lane_change_allowed</font> (boolean)
 
Boolean indicator of whether the lane change is allowed.


- <font color="#f8805a">white_list</font> (list)
 
The white list contains all position of target platoon member for joining.


- <font color="#f8805a">debug_helper</font> (PlanDebugHelper)
 
The helper class that help with the debug functions.


### Methods 
- <font color="#7fb800">add_white_list</font> (self, vm)
 
Add vehicle manager to white list.

- <font color="#7fb800">car_following_manager</font> (self, vehicle, distance, target_speed=None)
 
Module in charge of car-following behaviors when there's

#  Module in charge of car-following behaviors when there's
**Arguments:**

*target_speed (float):*  The target car following speed.

*vehicle (carla.vehicle):*  Leading vehicle to follow.

*distance (float):*  distance from leading vehicle.

*control (carla.VehicleControl):*  Vehicle control of the next step.

**Returns**

*target_speed (float):*  The target speed for the next step.

*target_loc (carla.location):*  The target location for the next step.

- <font color="#7fb800">collision_manager</font> (self, rx, ry, ryaw, waypoint, adjacent_check=False)
 
This module is in charge of warning in case of a collision.

**Arguments:**

**Returns**

- <font color="#7fb800">get_local_planner</font> (self)
 
return the local planner

- <font color="#7fb800">lane_change_management</font> (self)
 
Identify whether a potential hazard exits if operating lane change.

**Returns**

- <font color="#7fb800">overtake_management</font> (self, obstacle_vehicle)
 
Overtake behavior.

**Arguments:**

- <font color="#7fb800">reroute</font> (self, spawn_points)
 
This method implements re-routing for vehicles approaching its destination.

**Arguments:**

- <font color="#7fb800">run_step</font> (self, target_speed=None, collision_detector_enabled=True, lane_change_allowed=True)
 
Execute one step of navigation

**Arguments:**

**Returns**

- <font color="#7fb800">set_destination</font> (self, start_location, end_location, clean=False, end_reset=True, clean_history=False)
 
This method creates a list of waypoints from agent's position to destination location

**Arguments:**

*end_reset (boolean):*  Flag to reset the waypoint queue.

*start_location (carla.location):*  initial position.

*end_location (carla.location):*  final position.

*clean (boolean):*  Flag to clean the waypoint queue.

*clean_history (boolean):*  Flag to clean the waypoint history.

- <font color="#7fb800">traffic_light_manager</font> (self, waypoint)
 
This method is in charge of behaviors for red lights and stops.

**Arguments:**

*waypoint (carla.waypoint):*  current waypoint of the agent.

- <font color="#7fb800">update_information</font> (self, ego_pos, ego_speed, objects)
 
Update the perception and localization information to the behavior agent.

**Arguments:**

*ego_pos (carla.Transform):*  ego position from localization module.

*ego_speed (float):*  km/h, ego speed.

*objects (dictionary):*  Objects detection results from perception module.

- <font color="#7fb800">white_list_match</font> (self, obstacles)
 
Match the detected obstacles with the white list. Remove the obstacles that are in white list.

**Arguments:**

*obstacles (list):*   a list of carla.Vehicle or ObstacleVehicle

**Returns**

*new_obstacle_list (list):*  the new list of obstacles

# opencda.core.plan.global_route_planner
Description for this module: This module provides GlobalRoutePlanner implementation.

## GlobalRoutePlanner(dao)
This class provides a very high level route plan.

### Parameters
- <font color="#f8805a">dao</font> (carla.dao)
 
A global plan that contains routes from start to end.


### Attributes
- <font color="#f8805a">_topology</font> (carla.topology)
 
The topology graph of the current routes.


- <font color="#f8805a">_graph</font> (nx.DiGraph )
 
The node-edge graph of the current routes.


- <font color="#f8805a">_id_map</font> (dict)
 
A map constructed with road segment IDs.


- <font color="#f8805a">_road_id_to_edge</font> (list)
 
A mapping that reference road it to edge in the graph.  


- <font color="#f8805a">_intersection_end_node</font> (int)
 
The node ID of at the end of the intersection.    


- <font color="#f8805a">_previous_decision</font> (carla.RoadOption)
 
The previous behavioral option of the ego vehicle.


### Methods 
- <font color="#7fb800">abstract_route_plan</font> (self, origin, destination)
 
The function that generates the route plan based on origin and destination.

**Arguments:**

*origin (carla.Location):*  object of the route's start position.

*destination (carla.Location):*   object of the route's end position.

**Returns**

* plan (list):*  List of turn by turn navigation decisions as agents.navigation.local_planner.RoadOption.

- <font color="#7fb800">setup</font> (self)
 
Performs initial server data lookup for detailed topology

- <font color="#7fb800">trace_route</font> (self, origin, destination)
 
This method returns list of (carla.Waypoint, RoadOption)

# opencda.core.plan.global_route_planner_dao
Description for this module: This module provides implementation for GlobalRoutePlannerDAO

## GlobalRoutePlannerDAO(wmap, sampling_resolution)
This class is the data access layer for fetching data from the carla server instance for GlobalRoutePlanner.

### Parameters
- <font color="#f8805a">wmap</font> (carla.world)
 
The current carla simulation world.


- <font color="#f8805a">sampling_resolution</font> (float)
 
sampling distance between waypoints.


### Methods 
- <font color="#7fb800">get_resolution</font> (self)
 
Return the sampling resolution.

- <font color="#7fb800">get_topology</font> (self)
 
Accessor for topology.

- <font color="#7fb800">get_waypoint</font> (self, location)
 
The method returns waypoint at given location.

**Arguments:**

*location (carla.lcoation):*  Vehicle location.

**Returns**

*waypoint (carla.waypoint):*  Newly generated waypoint close to location

# opencda.core.plan.local_planner_behavior
Description for this module: This module contains a local planner to perform

## LocalPlanner(agent, carla_map, config_yaml)
LocalPlanner implements the basic behavior of following a trajectory of waypoints that is generated on-the-fly.

### Parameters
- <font color="#f8805a">agent</font> (carla.agent)
 
The carla.agent that applying vehicle contorl.


- <font color="#f8805a">carla_map</font> (carla.map)
 
The HD map of the current simulation world.


- <font color="#f8805a">config</font> (dict)
 
The configuration dictionary of the trajectory planning module.


### Attributes
- <font color="#f8805a">_vehicle</font> (carla.vehicle)
 
The caral vehicle objcet.


- <font color="#f8805a">_ego_pos</font> (carla.position )
 
The current position of the ego vehicle.


- <font color="#f8805a">_ego_speed</font> (float)
 
The current speed of the ego vehicle.


- <font color="#f8805a">waypoints_queue</font> (deque)
 
The waypoint deque of the current plan.


- <font color="#f8805a">_waypoint_buffer</font> (deque)
 
A buffer deque to store waypoints of the next steps.


- <font color="#f8805a">_long_plan_debug</font> (list)
 
A list that stores the waypoints of global plan for debug purposes.


- <font color="#f8805a">_trajectory_buffer</font> (deque)
 
A deque buffer that stores the current trajectory.


- <font color="#f8805a">_history_buffer</font> (deque)
 
A deque buffer that stores the trajectory history of the ego vehicle.


- <font color="#f8805a">lane_change</font> (boolean)
 
A indicator used to identify whether lane change is operated


- <font color="#f8805a">lane_id_change</font> (boolean)
 
In some corner cases, the id is not changed but we regard it as lane change due to large lateral diff.


### Methods 
- <font color="#7fb800">generate_path</font> (self)
 
Generate the smooth path using cubic spline.

**Returns**

*rx (list):*  List of planned path points' x coordinates.

*ry (list):*  List of planned path points' y coordinates.

*ryaw (list):*  List of planned path points' yaw angles.

*rk (list):*  List of planned path points' curvatures.

- <font color="#7fb800">generate_trajectory</font> (self, rx, ry, rk)
 
Sampling the generated path and assign speed to each point.

**Arguments:**

*rx (list):*  List of planned path points' x coordinates.

*ry (list):*  List of planned path points' y coordinates.

*rk (list):*  List of planned path points' curvatures.

*debug (boolean):*  whether to draw the whole plan path

- <font color="#7fb800">get_trajetory</font> (self)
 
Get the trajetory.

- <font color="#7fb800">pop_buffer</font> (self, vehicle_transform)
 
Remove waypoints the ego vehicle has achieved.

- <font color="#7fb800">run_step</font> (self, rx, ry, rk, target_speed=None, trajectory=None, following=False)
 
Execute one step of local planning which involves

**Arguments:**

*rx (list):*  List of planned path points' x coordinates.

*ry (list):*  List of planned path points' y coordinates.

*ryaw (list):*  List of planned path points' yaw angles.

*rk (list):*  List of planned path points' curvatures.

*following (boolean):*  Indicator of whether the vehicle is under following status.

*trajectory (list):*  Pre-generated car-following trajectory only for platoon members.

*target_speed (float):*  The ego vehicle's desired speed.

**Returns**

*speed (float):*  Next trajectory point's target speed

*waypoint (carla.waypoint):*  Next trajectory point's waypoint.

- <font color="#7fb800">set_global_plan</font> (self, current_plan, clean=False)
 
Sets new global plan.

**Arguments:**

*clean (boolean):*  Indicator of whether to clear the global plan.

*current_plan (list):*  list of waypoints in the actual plan.

- <font color="#7fb800">update_information</font> (self, ego_pos, ego_speed)
 
Update the ego position and speed for trajectory planner.

**Arguments:**

*ego_pos (carla.Transform):*  Ego position from localization module.

*ego_speed (float):*  Ego speed(km/h) from localization module.

## RoadOption(value, names=None, *, module=None, qualname=None, type=None, start=1)
RoadOption represents the possible topological configurations when moving from a segment of lane to other.

### Ancestors 
enum.Enum

### Class variables 
*CHANGELANELEFT*

*CHANGELANERIGHT*

*LANEFOLLOW*

*LEFT*

*RIGHT*

*STRAIGHT*

*VOID*

# opencda.core.plan.planer_debug_helper
Description for this module: Analysis + Visualization functions for planning

## PlanDebugHelper(actor_id)
This class aims to save statistics for planner behaviour.

### Parameters:
- <font color="#f8805a">actor_id</font> (int)
 
The actor ID of the target vehicle for bebuging. 


### Attributes
- <font color="#f8805a">speed_list</font> (list )
 
The list containing speed info(m/s) of all time-steps.


- <font color="#f8805a">acc_list</font> (list)
 
The list containing acceleration info(m^2/s) of all time-steps.


- <font color="#f8805a">ttc_list</font> (list)
 
The list containing ttc info(s) for all time-steps.


- <font color="#f8805a">count</font> (int )
 
Used to count how many simulation steps have been executed.


### Methods 
- <font color="#7fb800">evaluate</font> (self)
 
Evaluate the target vehicle and visulize the plot.

**Returns**

*figure (matplotlib.pyplot.figure):*  The target vehicle's planning profile (velocity, acceleration, and ttc).

*perform_txt (txt file):*  The target vehicle's planning profile as text files.

- <font color="#7fb800">update</font> (self, ego_speed, ttc)
 
Update the speed info.

**Arguments:**

*ego_speed (float):*  Ego speed in km/h.

*ttc (flot):*  Time to collision in seconds

# opencda.core.sensing.localization.kalman_filter
Description for this module: Use Kalman Filter on GPS + IMU for better localization.

## KalmanFilter(dt)
Kalman Filter implementation for gps and imu. 

### Parameters
- <font color="#f8805a">dt</font> (float)
 
The step time for kalman filter calculation.


### Attributes
- <font color="#f8805a">Q</font> (numpy.array)
 
predict state covariance.


- <font color="#f8805a">R</font> (numpy.array)
 
Observation x,y position covariance.


- <font color="#f8805a">time_step</font> (float)
 
The step time for kalman filter calculation.


- <font color="#f8805a">xEst</font> (numpy.array)
 
Estimated x values.


- <font color="#f8805a">PEst</font> (numpy.array)
 
The estimated P values.


**Arguments:**

### Methods 
- <font color="#7fb800">motion_model</font> (self, x, u)
 
Predict current position and yaw based on previous result (X = F * X_prev + B * u).

**Arguments:**

*x (np.array) [x_prev, y_prev, yaw_prev, v_prev], shape:*  (4, 1).

*u (np.array):*  [v_current, imu_yaw_rate], shape:(2, 1).

**Returns**

*x (np.array):*  predicted state.

- <font color="#7fb800">observation_model</font> (self, x)
 
Project the state matrix to sensor measurement matrix.

**Arguments:**

*x (np.array):*  [x, y, yaw, v], shape: (4. 1).

**Returns**

*z (np.array):*  predicted measurement.

- <font color="#7fb800">run_step</font> (self, x, y, heading, velocity, yaw_rate_imu)
 
Apply KF on current measurement and previous prediction.

**Arguments:**

*x (float):*  x(esu) coordinate from gnss sensor at current timestamp.

*y (float):*  y(esu) coordinate from gnss sensor at current timestamp.

*heading (float):*  heading direction at current timestamp.

*velocity (float):*  current speed.

*yaw_rate_imu (float):*  yaw rate rad/s from IMU sensor.

**Returns**

*Xest (np.array):*  The corrected x, y, heading, and velocity information.

- <font color="#7fb800">run_step_init</font> (self, x, y, heading, velocity)
 
Initial state filling.

**Arguments:**

*x (float):*  The x coordinate.

*y (float):*  The y coordinate.

*heading (float):*  The heading direction.

*velocity (flaot):*  The velocity speed

# opencda.core.sensing.localization.localization_debug_helper
Description for this module: Visualization tools for localization

## LocDebugHelper(config_yaml, actor_id)
This class aims to help users debugging their localization algorithms.

### Attributes
### Methods 
- <font color="#7fb800">evaluate</font> (self)
 
Plot the localization related data points.

**Returns**

*figures(matplotlib.pyplot.plot):*  The plot of localization related figures.

*perform_txt(txt file):*  The localization related datas saved as text file.

- <font color="#7fb800">run_step</font> (self, gnss_x, gnss_y, gnss_yaw, gnss_spd, filter_x, filter_y, filter_yaw, filter_spd, gt_x, gt_y, gt_yaw, gt_spd)
 
Run a single step for DebugHelper to save and animate(optional) the localization data.

**Arguments:**

*gnss_x (float):*  GNSS detected x coordinate. 

*gnss_y (float):*  GNSS detected y coordinate. 

*gnss_yaw (float):*  GNSS detected yaw angle. 

*gnss_spd (float):*  GNSS detected speed value. 

*filter_x (float):*  Filtered x coordinates. 

*filter_y (float):*  Filtered y coordinates. 

*filter_yaw (float):*  Filtered yaw angle. 

*filter_spd (float):*  Filtered speed value. 

*gt_x (float):*  The ground truth x coordinate.

*gt_y (float):*  The ground truth y coordinate.

*gt_yaw (float):*  The ground truth yaw angle.

*gt_spd (float):*  The ground truth speed value

# opencda.core.sensing.localization.coordinate_transform
Description for this module: Functions to transfer coordinates under different coordinate system

## geo_to_transform(lat, lon, alt, lat_0, lon_0, alt_0)
Convert WG84 to ENU. The origin of the ENU should pass the geo reference.

**Arguments:**

**Returns**

# opencda.core.sensing.localization.localization_manager
Description for this module: Localization module

## GnssSensor(vehicle, config)
The default GNSS sensor module.

### Parameters
- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla.Vehicle. We need this class to spawn our gnss and imu sensor.


- <font color="#f8805a">config</font> (dict)
 
The configuration dictionary of the localization module.


### Attributes
- <font color="#f8805a">world</font> (carla.world)
 
The caral world of the current vehicle.


- <font color="#f8805a">blueprint</font> (carla.blueprint )
 
The current blueprint of the sensor actor.


- <font color="#f8805a">weak_self</font> (opencda Object)
 
A weak reference point to avoid circular reference.


- <font color="#f8805a">sensor</font> (CARLA actor)
 
The current sensor actors that will be attach to the vehicles.


## ImuSensor(vehicle)
Default ImuSensor module.

### Parameters
- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla.Vehicle. We need this class to spawn our gnss and imu sensor.


### Attributes
- <font color="#f8805a">world</font> (carla.world)
 
The caral world of the current vehicle.


- <font color="#f8805a">blueprint</font> (carla.blueprint )
 
The current blueprint of the sensor actor.


- <font color="#f8805a">weak_self</font> (opencda Object)
 
A weak reference point to avoid circular reference.


- <font color="#f8805a">sensor</font> (CARLA actor)
 
The current sensor actors that will be attach to the vehicles.


## LocalizationManager(vehicle, config_yaml, carla_map)
Default localization module.

### Parameters
- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla.Vehicle. We need this class to spawn our gnss and imu sensor.


- <font color="#f8805a">config_yaml</font> (dict)
 
The configuration dictionary of the localization module.


- <font color="#f8805a">carla_map</font> (carla.Map)
 
The carla HDMap. We need this to find the map origin to


### Attributes
- <font color="#f8805a">gnss</font> (opencda object)
 
GNSS sensor manager for spawning gnss sensor and listen to the data


- <font color="#f8805a">ImuSensor</font> (opencda object)
 
Imu sensor manager for spawning gnss sensor and listen to the data


- <font color="#f8805a">kf</font> (opencda object)
 
The filter used to fuse different sensors.


- <font color="#f8805a">debug_helper</font> (opencda object)
 
The debug helper is used to visualize the accuracy of


### Methods 
- <font color="#7fb800">add_heading_direction_noise</font> (self, heading_direction)
 
Add synthetic noise to heading direction.

**Arguments:**

*heading_direction (float):*  groundtruth heading_direction obtained from the server.

**Returns**

*heading_direction (float):*  heading direction with noise.

- <font color="#7fb800">add_speed_noise</font> (self, speed)
 
Add gaussian white noise to the current speed.

**Arguments:**

*speed (float):*  m/s, current speed.

**Returns**

*speed (float):*  the speed with noise.

- <font color="#7fb800">destroy</font> (self)
 
Destroy the sensors

- <font color="#7fb800">get_ego_pos</font> (self)
 
Retrieve ego vehicle position

- <font color="#7fb800">get_ego_spd</font> (self)
 
Retrieve ego vehicle speed

- <font color="#7fb800">localize</font> (self)
 
Currently implemented in a naive way.
# opencda.core.sensing.perception.obstacle_vehicle
Description for this module: Obstacle vehicle class to save object detection.

## is_vehicle_cococlass(label)
Check whether the label belongs to the vehicle class according to coco dataset.

**Arguments:**

**Returns**

## BoundingBox(corners)
Bounding box class for obstacle vehicle.

- <font color="#f8805a">corners</font> (nd.nparray)
 
Eight corners of the bounding box. (shape:(8, 3))


### Attributes:
- <font color="#f8805a">location</font> (carla.location)
 
The location of the object.


- <font color="#f8805a">extent</font> (carla.vector3D)
 
The extent of  the object.


## ObstacleVehicle(corners, o3d_bbx, vehicle=None, lidar=None)
A class for obstacle vehicle. The attributes are designed to match with carla.Vehicle class.

### Parameters: 
- <font color="#f8805a">corners</font> (nd.nparray)
 
Eight corners of the bounding box. (shape:(8, 3))


- <font color="#f8805a">o3d_bbx</font> (pen3d.AlignedBoundingBox)
 
The bounding box object in Open3d. This is mainly used forvisualization.


- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla.Vehicle object.


- <font color="#f8805a">lidar</font> (carla.sensor.lidar)
 
The lidar sensor.


### Attributes:
- <font color="#f8805a">bounding_box</font> (BoundingBox)
 
Bounding box of the osbject vehicle. 


- <font color="#f8805a">location</font> (carla.location)
 
The location of the object.


- <font color="#f8805a">velocty</font> (carla.Vector3D vehicle.)
 
Velocity of the object vehicle.


### Methods 
- <font color="#7fb800">get_location</font> (self)
 
Return the location of the object vehicle.

- <font color="#7fb800">get_transform</font> (self)
 
Return the transform of the object vehicle.

- <font color="#7fb800">get_velocity</font> (self)
 
Return the velocity of the object vehicle.

- <font color="#7fb800">set_vehicle</font> (self, vehicle, lidar)
 
Assign the attributes from carla.Vehicle to ObstacleVehicle.

**Arguments:**

*vehicle(carla.Vehicle):*  The carla.Vehicle object.

*lidar(carla.sensor.lidar):*  The lidar sensor, used to project world coordinates to sensor coordinates.

- <font color="#7fb800">set_velocity</font> (self, velocity)
 
Set the velocity of the vehicle.

**Arguments:**

*velocity(carla.Vector3D):*  The target velocity in 3d vector format.

## StaticObstacle(corner, o3d_bbx)
The general class for obstacles. Currently, we regard all static obstacles such as stop signs and traffic light as the same class.

### Parameters 
- <font color="#f8805a">corner</font> (nd.nparray)
 
Eight corners of the bounding box (shape:(8, 3)).


- <font color="#f8805a">o3d_bbx</font> (open3d.AlignedBoundingBox)
 
The bounding box object in Open3d. This is mainly used for visualization.


### Attributes
- <font color="#f8805a">bounding_box</font> (BoundingBox)
 
Bounding box of the osbject vehicle. 


# opencda.core.sensing.perception.o3d_lidar_libs
Description for this module: Utility functions for 3d lidar visualization and processing by utilizing open3d.

## o3d_camera_lidar_fusion(objects, yolo_bbx, lidar_3d, projected_lidar, lidar_sensor)
Utilize the 3D lidar points to extend the 2D bounding box from camera to 3D bounding box under world coordinates.

**Arguments:**

**Returns**

## o3d_pointcloud_encode(raw_data, point_cloud)
Encode the raw point cloud to Open3d PointCloud object.

**Arguments:**

## o3d_visualizer_init(actor_id)
Initialize the visualizer.

**Arguments:**

**Returns**

## o3d_visualizer_show(vis, count, point_cloud, objects)
Visualize the point cloud at runtime.

**Arguments:**

# opencda.core.sensing.perception.perception_manager
Description for this module: Perception module

## CameraSensor(vehicle, position='front')
Class for rgb camera.

### Parameters
- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla.Vehicle. We need this class to spawn sensors.


- <font color="#f8805a">position</font> (string)
 
Indicates the sensor is a front or rear camera. option: front, left, right.


### Attributes
- <font color="#f8805a">image</font> (np.ndarray)
 
Current received image.


- <font color="#f8805a">sensor</font> (CARLA actor)
 
The current sensor actors that will be attach to the vehicles.


## LidarSensor(vehicle, config_yaml)
Lidar sensor manager.

### Parameters
- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla.Vehicle. We need this class to spawn sensors.


- <font color="#f8805a">config_yaml</font> (dict)
 
Configuration for lidar sensor.


### Attributes
- <font color="#f8805a">o3d_pointcloud</font> (o3d.PointCloud)
 
Recieved point cloud saved in o3d.PointCloud format.


- <font color="#f8805a">sensor</font> (CARLA actor)
 
The current sensor actors that will be attach to the vehicles.


**Arguments:**

## PerceptionManager(vehicle, config_yaml, ml_manager)
Default perception module. Currenly only used to detect vehicles.

### Parameters
- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla.Vehicle. We need this class to spawn sensors.


- <font color="#f8805a">config_yaml</font> (dict)
 
Configuration for lidar sensor.


- <font color="#f8805a"> ml_manage</font> (opencda object)
 
Ml manager includes all loaded trained perception models.


### Attributes
- <font color="#f8805a">ml_manager</font> (opencda object)
 
weak reference of the ML Manager.


- <font color="#f8805a">activate</font> (bool)
 
Whether perception algorithms are activated. If not, load object


- <font color="#f8805a">lida</font> (opencda object)
 
Lidar sensor manager.


- <font color="#f8805a">rgb_camer</font> (opencda object)
 
Camera manager.


**Arguments:**

### Methods 
- <font color="#7fb800">activate_mode</font> (self, objects)
 
Use Yolov5 + Lidar fusion to detect objects.

**Arguments:**

**Returns**

- <font color="#7fb800">deactivate_mode</font> (self, objects)
 
Obstacle detection under perception deactivation mode.

**Arguments:**

**Returns**

- <font color="#7fb800">destroy</font> (self)
 
Destroy sensors.

**Returns**

- <font color="#7fb800">detect</font> (self, ego_pos)
 
Detect surrounding objects. Currently only vehicle detection supported.

**Arguments:**

**Returns**

- <font color="#7fb800">dist</font> (self, v)
 
A fast method to retrieve the obstable distance the ego vehicle from the server directly.

**Arguments:**

**Returns**

- <font color="#7fb800">speed_retrieve</font> (self, objects)
 
We don't implement any obstacle speed calculation algorithm. The speed will be retrieved from

**Arguments:**

**Returns**

- <font color="#7fb800">visualize_3d_bbx_front_camera</font> (self, objects, rgb_image)
 
Visualize the 3d bounding box on frontal camera image.

**Arguments:**

**Returns**

# opencda.core.sensing.perception.sensor_transformation
Description for this module: This script contains the transformations between world and different sensors.

## bbx_to_world(cords, vehicle)
Convert bounding box coordinate at vehicle reference to world reference.

**Arguments:**

**Returns**

## create_bb_points(vehicle)
Extract the eight vertices of the bounding box from the vehicle.

**Arguments:**

**Returns**

## get_2d_bb(vehicle, sensor, senosr_transform)
Summarize 2D bounding box creation

**Arguments:**

**Returns**

## get_bounding_box(vehicle, camera, sensor_transform)
Get vehicle bounding box and project to sensor image.

**Arguments:**

**Returns**

## get_camera_intrinsic(sensor)
Retrieve the camera intrinsic matrix

**Arguments:**

**Returns**

## p3d_to_p2d_bb(p3d_bb)
Draw 2D bounding box (4 vertices) from 3D bounding box (8 vertices) in image.

**Arguments:**

**Returns**

## project_lidar_to_camera(lidar, camera, point_cloud, rgb_image)
Project lidar to camera space.

**Arguments:**

**Returns**

## sensor_to_world(cords, sensor_transform)
Project

**Arguments:**

**Returns**

## vehicle_to_sensor(cords, vehicle, sensor_transform)
Transform coordinates from vehicle reference to sensor reference

**Arguments:**

**Returns**

## world_to_sensor(cords, sensor_transform)
Transform coordinate from world reference to sensor reference.

**Arguments:**

**Returns**

## x_to_world_transformation(transform)
Get the transformation matrix from x(it can be vehicle or sensor) coordinates to world coordinate.

**Arguments:**

**Returns**

# opencda.core.application.platooning.fsm
Description for this module: Finite State Machine

## FSM(value, names=None, *, module=None, qualname=None, type=None, start=1)
The finite state machine class for platooning. These classes are used to indicate

### Attributes
- <font color="#f8805a">SEARCHING</font> (int)
 
The vehicle is not in any platoon and currently searching one to join.


- <font color="#f8805a">OPEN_GAP</font> (int)
 
The platoon member is increasing the gap for other vehicle to merge.


- <font color="#f8805a">MOVE_TO_POINT</font> (int)
 
The merging vehicle is moving to the meeting points for joining.


- <font color="#f8805a">JOINING</font> (int)
 
The merging vehicle is operating the joining maneuver(lane change).


- <font color="#f8805a">MAINTINING</font> (int)
 
The platoon member is following the leader and maintain the time gap.


- <font color="#f8805a">BACK_JOINING</font> (int)
 
The merging vehicle is in back-join state.


- <font color="#f8805a">CUT_IN_TO_BACK</font> (int)
 
The merging vehicle abandons cut-in-join and switch to back join.


- <font color="#f8805a">JOINING_FINISHED</font> (int)
 
Indicate the joining finished and the vehicle will switch to maintaining state.


- <font color="#f8805a">LEADING_MODE</font> (int)
 
The vehicle is the platoon leader.


- <font color="#f8805a">ABONDO</font> ()
 
Current joining is abandoned.


- <font color="#f8805a">DISABL</font> ()
 
V2X is not available and thus won't join any platoon.


### Ancestors 
enum.Enum

### Class variables 
*ABONDON*

*BACK_JOINING*

*CUT_IN_TO_BACK*

*DISABLE*

*FRONT_JOINING*

*JOINING*

*JOINING_FINISHED*

*LEADING_MODE*

*MAINTINING*

*MOVE_TO_POINT*

*OPEN_GAP*

*SEARCHING*

# opencda.core.application.platooning.platooning_plugin
Description for this module: Platooning plugin for communication and track FSM

## PlatooningPlugin(search_range, cda_enabled)
Platooning plugin inside the V2X manager.

### Parameters
- <font color="#f8805a">search_range</font> (float)
 
The search range of the communication equipment.


- <font color="#f8805a">cda_enabled</font> (boolean)
 
Whether connectivity is supported.


### Attributes
- <font color="#f8805a">leader</font> (boolean)
 
Boolean indicator of the platoon leader status.


- <font color="#f8805a">platooning_object</font> (opencda object )
 
The current platoon object.


- <font color="#f8805a">platooning_id</font> (int)
 
The current platoon ID.


- <font color="#f8805a">in_id</font> (int)
 
The position in the platoon.


- <font color="#f8805a">status</font> (enum)
 
The current platooning status.


- <font color="#f8805a">ego_pos</font> (carla.transformation)
 
The current position (i.e., location and rotation) of the ego vehicle.


- <font color="#f8805a">ego_spd</font> (float)
 
The current speed(km/h) of the ego vehicle.


- <font color="#f8805a">platooning_blacklist</font> (list)
 
The platoon in the black list won't be considered again.


- <font color="#f8805a">front_vehicle</font> (opencda object)
 
The front vehicle manager of the ego vehicle.


- <font color="#f8805a">rear_vechile</font> (opencda object)
 
The rear vehicle manager of the ego vehicle.


### Methods 
- <font color="#7fb800">match_platoon</font> (self, cav_world)
 
A naive way to find the best position to join a platoon

**Arguments:**

*cav_world (carla.world):*  Current simulation world.

**Returns**

*(boolean):*  The boolean indicator of matching result. 

*min_index (int):*  The minimum index inside the selected platoon.

*platoon_vehicle_list (list):*  The list of platoon vehicle memebers.

- <font color="#7fb800">reset</font> (self)
 
Reset to the origin status.

- <font color="#7fb800">search_platoon</font> (self, ego_pos, cav_world)
 
Search platoon candidate in the range

**Arguments:**

*ego_pos (carla.transformation):*  Current position of the ego vehicle.

*cav_world (carla.world):*  Current simulation world.

**Returns**

*pmid (int):*  Platoon manager ID.

*pm (opencda object):*  Platoon manager ID.

- <font color="#7fb800">set_platoon</font> (self, in_id, platooning_object=None, platooning_id=None, leader=False)
 
Set platooning status

**Arguments:**

*in_id (int):*  Inner platoon ID of the vehicle.

*platooning_object (opencda object):*  The current platoon object.

*platooning_id (int):*  The current platoon ID.

*leader (boolean):*  Boolean indicator of the platoon leader status.

- <font color="#7fb800">set_status</font> (self, status)
 
Set FSM status

**Arguments:**

*status (string):*  The current platooning status.

- <font color="#7fb800">update_info</font> (self, ego_pos, ego_spd)
 
Update the ego position and speed

**Arguments:**

*heading_direction:*  groundtruth heading_direction obtained from the server.

*dummy_variable:*  dummy variable to test multiple return/args.

*dummy_variable:*  dummy variable to test multiple return/args.

*dummy_variable:*  dummy variable to test multiple return/args.

**Returns**

*heading_direction:*  heading direction with noise.

*dummy_variable:*  dummy variable to test multiple return/args

# opencda.core.application.platooning.platoon_debug_helper
Description for this module: Analysis + visualization functions for platooning

## PlatoonDebugHelper(actor_id)
This class aims to save statistics for platoon behaviour

### Parameters
- <font color="#f8805a">actor_id</font> (int)
 
The actor ID of the selected vehcile.


### Attributes
- <font color="#f8805a">time_gap_list</font> (list )
 
The list containing intra-time-gap(s) of all time-steps.


- <font color="#f8805a">dist_gap_list</font> (list )
 
The list containing distance gap(s) of all time-steps.


### Ancestors 
opencda.core.plan.planer_debug_helper.PlanDebugHelper

### Methods 
- <font color="#7fb800">update</font> (self, ego_speed, ttc, time_gap=None, dist_gap=None)
 
Update the platoon related vehicle information.

**Arguments:**

*ego_speed (float):*  Ego vehcile speed.

*ttc (flaot):*  Ego vehicle time-to-collision.

*time_gap (float):*  Ego vehicle time gap with the front vehicle.

*dist_gap (float):*  Ego vehicle distance gap with front vehicle

# opencda.core.application.platooning.platoon_behavior_agent
Description for this module: Behavior manager for platooning specifically

## PlatooningBehaviorAgent(vehicle, vehicle_manager, v2x_manager, behavior_yaml, platoon_yaml, carla_map)
Platoon behavior agent that inherits the single vehicle behavior agent.

### Parameters
- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla vehicle.


- <font color="#f8805a">vehicle_manager</font> (opencda object)
 
The vehicle manager, used when joining platoon finished.


- <font color="#f8805a">v2x_manager</font> (opencda object)
 
Used to received and deliver information.


- <font color="#f8805a">behavior_yaml</font> (dict)
 
The configuration dictionary for BehaviorAgent.


- <font color="#f8805a">platoon_yaml</font> (dict.)
 
The configuration dictionary for platoon behavior.


- <font color="#f8805a">carla_map</font> (carla.Map)
 
The HD Map used in the simulation.


### Attributes
- <font color="#f8805a">vehicle_manager</font> (opencda object)
 
The vehicle manager, used when joining platoon finished.


- <font color="#f8805a">v2x_manager</font> (opencda object)
 
Used to received and deliver information.


- <font color="#f8805a">debug_helper</font> (opencda Object)
 
A debug helper used to record the driving performance during platooning.


- <font color="#f8805a">inter_gap</font> (float)
 
The desired time gap between each platoon member.


### Ancestors 
opencda.core.plan.behavior_agent.BehaviorAgent

### Methods 
- <font color="#7fb800">calculate_gap</font> (self, distance)
 
Calculate the current vehicle and frontal vehicle's time/distance gap.

**Arguments:**

*distance (float):*   distance between the ego vehicle and frontal vehicle

- <font color="#7fb800">joining_finish_manager</font> (self, insert_vehicle='front')
 
Called when a joining is finish to update the platoon manager list.

**Arguments:**

*insert_vehicle (string):*  indicate use the front or rear vehicle index to update the platoon manager list.

- <font color="#7fb800">platooning_following_manager</font> (self, inter_gap)
 
Car following behavior in platooning with gap regulation.

**Arguments:**

* inter_gap (float):*  the gap designed for platooning

- <font color="#7fb800">platooning_merge_management</font> (self, frontal_vehicle_vm)
 
Merge the vehicle into the platooning.

**Arguments:**

*frontal_vehicle_vm (opencda object):*  The vehivlel manager of the front vehicle.

**Returns**

*target_speed (float):*  The target speed for ego vehicle.

*target_waypoint (carla.waypoint):*  The target waaypoint for ego vehcile.

- <font color="#7fb800">run_step</font> (self, target_speed=None, collision_detector_enabled=True, lane_change_allowed=True)
 
Run a single step for navigation under platooning agent. Finite state machine is used to switch between

**Arguments:**

*target_speed (float):*  Target speed in km/h

*collision_detector_enabled (bool):*  Whether collision detection enabled.

*ane_change_allowed (bool):*  Whether lane change is allowed.

- <font color="#7fb800">run_step_back_joining</font> (self)
 
Back-joining Algorithm.

**    Back-joining Algorithm.

**Returns**

*control command (opencda object):*  control command for bacj joining.

*back join status (enum):*  FSM back joining status.

- <font color="#7fb800">run_step_cut_in_joining</font> (self)
 
Check if the vehicle has been joined successfully.

- <font color="#7fb800">run_step_cut_in_move2point</font> (self)
 
The vehicle is trying to get to the move in point.

**Arguments:**

*target_speed (float):*  The target speed for ego vehile.

*target_waypoint (carla.waypoint):*  The waypoint for ego vehile.

*next FSM state (enum):*  The next finite state machine state.

- <font color="#7fb800">run_step_front_joining</font> (self)
 
Front-joining algorithm.

**Returns**

*control command (opencda object):*  control command for bacj joining.

*back join status (enum):*  FSM back joining status.

- <font color="#7fb800">run_step_maintaining</font> (self)
 
Next step behavior planning for speed maintaining.

- <font color="#7fb800">run_step_open_gap</font> (self)
 
Open gap for cut-in vehicle.

- <font color="#7fb800">update_information</font> (self, ego_pos, ego_speed, objects)
 
Update the perception and localization information to the behavior agent.

**Arguments:**

*ego_pos (carla.Transform):*  ego position from localization module.

*ego_speed (float):*  km/h, ego speed.

*objects (dictionary):*  Objects detection results from perception module

# opencda.core.application.platooning.platooning_manager
Description for this module: Platooning Manager

## PlatooningManager(config_yaml, cav_world)
Platoon manager. Used to manage all vehicle managers inside the platoon.

### Parameters
- <font color="#f8805a">config_yaml</font> (dict)
 
The configuration dictionary for platoon.


- <font color="#f8805a">cav_world</font> (opencda object)
 
CAV world that stores all CAV information.


### Attributes
- <font color="#f8805a">pmid</font> (int)
 
The  platooning manager ID.


- <font color="#f8805a">vehicle_manager_list</font> (list)
 
A list of all vehciel managers within the platoon.


- <font color="#f8805a">destination</font> (carla.location)
 
The destiantion of the current plan.


- <font color="#f8805a">center_loc</font> (carla.location)
 
The center location of the platoon.


- <font color="#f8805a">leader_target_speed</font> (float)
 
The speed of the leader vehicle.


- <font color="#f8805a">origin_leader_target_speed</font> (float)
 
The original planned target speed of the platoon leader.


- <font color="#f8805a">recover_speed_counter</font> (int)
 
The counter that record the number of speed recovery attempts.


### Methods 
- <font color="#7fb800">add_member</font> (self, vehicle_manager, leader=False)
 
Add memeber to the current platooning

**Arguments:**

*leader (boolean):*  Indicator of whether this cav is a leader.

*vehicle_manager (opencda object):*  The vehicle manager class.

- <font color="#7fb800">cal_center_loc</font> (self)
 
Calculate and update center location of the platoon.

- <font color="#7fb800">destroy</font> (self)
 
Destroy platoon vehicles actors inside simulation world.

- <font color="#7fb800">evaluate</font> (self)
 
Used to save all members' statistics.

- <font color="#7fb800">reset_speed</font> (self)
 
After joining request accepted for certain steps, the platoon will return to the origin speed.

- <font color="#7fb800">response_joining_request</font> (self, request_loc)
 
Identify whether to accept the joining request based on capacity.

**Arguments:**

*request_loc (carla.Location):*  request vehicle location.

**Returns**

*response (boolean):*  Indicator of whether the joining request is accepted.

- <font color="#7fb800">run_step</font> (self)
 
Run one control step for each vehicles.

- <font color="#7fb800">set_destination</font> (self, destination)
 
Set desination of the vehicle managers in the platoon.

- <font color="#7fb800">set_lead</font> (self, vehicle_manager)
 
Set the leader of the platooning

**Arguments:**

*vehicle_manager (opencda object):*  The vehicle manager class.

- <font color="#7fb800">set_member</font> (self, vehicle_manager, index, lead=False)
 
Set member at specific index

**Arguments:**

*lead (boolean):*  Indicator of whether this cav is a leader.

*vehicle_manager (opencda object):*  The vehicle manager class.

*index (int):*  The platoon index of the current vehicle.

- <font color="#7fb800">update_information</font> (self)
 
Update CAV world information for every member in the list.

- <font color="#7fb800">update_member_order</font> (self)
 
Update the members' front and rear vehicle.

# opencda.core.actuation.pid_controller
Description for this module: PID Control Class

## Controller(args)
PID Controller implementation.

### Parameters
- <font color="#f8805a">args</font> (dict)
 
The configuration dictionary parsed from yaml file.


### Attributes
- <font color="#f8805a">_lon_ebuffer</font> (deque)
 
A deque buffer that stores longitudinal control errors.


- <font color="#f8805a">_lat_ebuffer</font> (deque)
 
A deque buffer that stores latitudinal control errors.


- <font color="#f8805a">current_transform</font> (carla.transform)
 
Current ego vehicle transformation in CARLA world(i.e., location and rotation).


- <font color="#f8805a">current_speed</font> (float)
 
Current ego vehicle speed .


- <font color="#f8805a">past_steering</font> (float)
 
Sterring angle from previous control step.


### Methods 
- <font color="#7fb800">dynamic_pid</font> (self)
 
Compute kp, kd, ki based on current speed

- <font color="#7fb800">lat_run_step</font> (self, target_location)
 
Generate the throttle command based on current speed and target speed

**Arguments:**

*target_location (carla.loaction):*  Target location of the ego vehicle.

**Returns**

*current_steering (float):*  Desired steering angle value for the current step to achieve target location.

- <font color="#7fb800">lon_run_step</font> (self, target_speed)
 
Generate the throttle command based on current speed and target speed

**Arguments:**

*target_speed (float):*  Target speed of the ego vehicle.

**Returns**

*acceleration (float):*  Desired acceleration value for the current step to achieve target speed.

- <font color="#7fb800">run_step</font> (self, target_speed, waypoint)
 
Execute one step of control invoking both lateral and longitudinal

**Arguments:**

*target_speed (float):*  Target speed of the ego vehicle.

*target_location (carla.loaction):*  Target location of the ego vehicle.

**Returns**

*control (carla.VehicleControl):*  Desired vehicle control command for the current step.

- <font color="#7fb800">update_info</font> (self, ego_pos, ego_spd)
 
Update ego position and speed to controller.

**Arguments:**

*ego_pos (carla.location):*  Position of the ego vehicle.

*ego_spd (float):*  Speed of the ego vehicle

# opencda.core.actuation.control_manager
Description for this module: Controller interface

## ControlManager(control_config)
Controller manager that is used to choose and call different controller's functions.

### Parameters
- <font color="#f8805a">control_config</font> (dict)
 
The configuration dictionary of the control manager module.


### Attributes
- <font color="#f8805a">controller</font> (opencda object.)
 
The controller object of the OpenCDA framwork.


### Methods 
- <font color="#7fb800">run_step</font> (self, target_speed, waypoint)
 
Execute current controller step.

- <font color="#7fb800">update_info</font> (self, ego_pos, ego_speed)
 
Update ego vehicle information for controller.
# opencda.core.common.vehicle_manager
Description for this module: Basic class of CAV

## VehicleManager(vehicle, config_yaml, application, carla_map, cav_world)
A class manager to embed different modules with vehicle together.

### Parameters
- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla.Vehicle. We need this class to spawn our gnss and imu sensor.


- <font color="#f8805a">config</font> (dict)
 
The configuration dictionary of the localization module.


- <font color="#f8805a">application</font> (list)
 
The application category, currently support:['single','platoon'].


- <font color="#f8805a">carla_map</font> (carla.Map)
 
The CARLA simulation map.


- <font color="#f8805a">cav_world</font> (opencda object)
 
CAV World.


### Attributes
- <font color="#f8805a">v2x_manager</font> (opencda object)
 
The current V2X manageer. 


- <font color="#f8805a">localizer</font> (opencda object)
 
The current localization manageer. 


- <font color="#f8805a">perception_manager</font> (opencda object)
 
The current V2X perception manageer. 


- <font color="#f8805a">agent</font> (opencda object)
 
The current carla agent that handles the basic control of ego vehicle.


- <font color="#f8805a">controller</font> (opencda object)
 
The current control manager.


### Methods 
- <font color="#7fb800">destroy</font> (self)
 
Destroy the actor vehicle

- <font color="#7fb800">run_step</font> (self, target_speed=None)
 
Execute one step of navigation.

- <font color="#7fb800">set_destination</font> (self, start_location, end_location, clean=False, end_reset=True)
 
Wrapper function to set global route

**Arguments:**

*start_location (carla.location):*  The start location of the current task.

*end_location (carla.location):*  The destination location of the current task.

*clean (boolean):*  Indicator of whether clean waypoint queue.

*end_reset (boolean):*  Indicator of whether reset the end location.

- <font color="#7fb800">update_info</font> (self)
 
Call perception and localization module to retrieve surrounding info an ego position.
# opencda.core.common.misc
Description for this module: Module with auxiliary functions.

# with auxiliary functions.
## cal_distance_angle(target_location, current_location, orientation)
Calculate the vehicle current relative distance to target location.

**Arguments:**

**Returns**

## compute_distance(location_1, location_2)
Euclidean distance between 3D points.

**Arguments:**

## compute_magnitude_angle(target_location, current_location, orientation)
Compute relative angle and distance between a target_location and a current_location.

**Arguments:**

**Returns**

## distance_vehicle(waypoint, vehicle_transform)
Returns the 2D distance from a waypoint to a vehicle

**Arguments:**

## draw_trajetory_points(world, waypoints, z=0.25, color=<carla.libcarla.Color object>, lt=5, size=0.1, arrow_size=0.1)
Draw a list of trajetory points

**Arguments:**

## draw_waypoints(world, waypoints, z=0.5)
Draw a list of waypoints at a certain height given in z.

**Arguments:**

## get_acc(vehicle, meters=False)
Compute acceleration of a vehicle.

**Arguments:**

**Returns**

## get_speed(vehicle, meters=False)
Compute speed of a vehicle in Km/h.

**Arguments:**

**Returns**

## is_within_distance(target_location, current_location, orientation, max_distance, d_angle_th_up, d_angle_th_low=0)
Check if a target object is within a certain distance from a reference object.

**Arguments:**

**Returns**

## is_within_distance_ahead(target_transform, current_transform, max_distance)
Check if a target object is within a certain distance in front of a reference object.

**Arguments:**

**Returns**

## positive(num)
Return the given number if positive, else 0

## vector(location_1, location_2)
Returns the unit vector from location_1 to location_2.

**Arguments:**

# opencda.core.common.cav_world
Description for this module: 

## CavWorld(apply_ml=False)
A customized world object to save all CDA vehicle information and shared ML models.

### Parameters
- <font color="#f8805a">apply_ml</font> (boolean)
 
whether apply ml/dl models in this simulation, please make sure 


### Attributes
- <font color="#f8805a">vehicle_id_set</font> (set)
 
A set that stores vehicle IDs.


- <font color="#f8805a">_vehicle_manager_dict</font> (dict)
 
A dictionary that stores vehicle managers.


- <font color="#f8805a">_platooning_dict</font> (dict)
 
A dictionary that stores platooning managers. 


- <font color="#f8805a">ml_manager</font> (opencda object.)
 
The machine learning manager class.


### Methods 
- <font color="#7fb800">get_platoon_dict</font> (self)
 
Return existing platoons.

- <font color="#7fb800">get_vehicle_managers</font> (self)
 
Return vehicle manager dictionary.

- <font color="#7fb800">locate_vehicle_manager</font> (self, loc)
 
Locate the vehicle manager based on the given location.

**Arguments:**

*loc (carla.Location):*  vehicle location.

**Returns**

*target_vm (vehicle_manager):*  The vehicle manager at the give location.

- <font color="#7fb800">update_platooning</font> (self, platooning_manger)
 
Add created platooning.

**Arguments:**

*platooning_manger (opencda object):*  The platooning manager class.

- <font color="#7fb800">update_vehicle_manager</font> (self, vehicle_manager)
 
Update created CAV manager to the world.

**Arguments:**

*vehicle_manager (opencda object):*  The vehicle manager class

# opencda.core.common.v2x_manager
Description for this module: Communication manager for cooperation

## V2XManager(cav_world, config_yaml)
V2X Manager for platooning, cooperative perception and so on.

### Parameters
- <font color="#f8805a">cav_world</font> (opencda object)
 
CAV world.


- <font color="#f8805a">config_yaml</font> (dict)
 
The configuration dictionary of the v2x module.


### Attributes
- <font color="#f8805a">_recieved_buffer</font> (dict)
 
A buffer for receive data.


- <font color="#f8805a">platooning_plugin</font> (opencda object)
 
The platooning plugin for communication during platooning.


### Methods 
- <font color="#7fb800">add_platoon_blacklist</font> (self, pmid)
 
Add an existing platoon to current blacklist.

**Arguments:**

* pmid (int):* The target platoon manager ID.

- <font color="#7fb800">get_platoon_front_rear</font> (self)
 
Get the ego vehicle's front and rear cav in the platoon

- <font color="#7fb800">get_platoon_manager</font> (self)
 
Retrieve the platoon manager the cav belongs to and the corresponding id

- <font color="#7fb800">get_platoon_status</font> (self)
 
Retrive the FSM status for platooning application

- <font color="#7fb800">in_platoon</font> (self)
 
Check whether the vehicle is inside the platoon.

**Arguments:**

* detection result (bool):*  Flag indication whether in a platoon.

- <font color="#7fb800">match_platoon</font> (self)
 
A naive way to find the best position to join a platoon.

- <font color="#7fb800">set_platoon</font> (self, in_id, platooning_object=None, platooning_id=None, leader=False)
 
Set platooning status

**Arguments:**

- <font color="#7fb800">set_platoon_front</font> (self, vm)
 
Set the frontal vehicle to another vehicle

**Arguments:**

- <font color="#7fb800">set_platoon_rear</font> (self, vm)
 
Set the rear vehicle to another vehicle

**Arguments:**

* vm (vehicle manager):* The target vehicle manager.

- <font color="#7fb800">set_platoon_status</font> (self, status)
 
Set the cav to a different fsm status.

**Arguments:**

*status (string):*  fsm status.

- <font color="#7fb800">update_info</font> (self, ego_pos, ego_spd)
 
Update all communication plugins with current localization info.
# opencda.customize.core.sensing.localization.extented_kalman_filter
Description for this module: Use Extended Kalman Filter on GPS + IMU for better localization.

## ExtentedKalmanFilter(dt)
Extended Kalman Filter implementation for gps and imu.

### Parameters
- <font color="#f8805a">dt</font> (float)
 
The step time for kalman filter calculation.


### Attributes
- <font color="#f8805a">Q</font> (numpy.array)
 
predict state covariance.


- <font color="#f8805a">R</font> (numpy.array)
 
Observation x,y position covariance.


- <font color="#f8805a">time_step</font> (float)
 
The step time for kalman filter calculation.


- <font color="#f8805a">xEst</font> (numpy.array)
 
Estimated x values.


- <font color="#f8805a">PEst</font> (numpy.array)
 
The estimated P values.


### Methods 
- <font color="#7fb800">jacob_f</font> (self, x, u)
 
Jacobian of Motion Model motion model

**Arguments:**

*x (np.array):*  Input X array.

**Returns**

*jF (np.array):*   Jacobian of Motion Model motion model.

- <font color="#7fb800">motion_model</font> (self, x, u)
 
Predict current position and yaw based on previous result (X = F * X_prev + B * u).

**Arguments:**

*x (np.array):*  [x_prev, y_prev, yaw_prev, v_prev], shape: (4, 1).

*u (np.array):*  [v_current, imu_yaw_rate], shape:(2, 1).

**Returns**

- <font color="#7fb800">observation_model</font> (self, x)
 
Project the state.array to sensor measurement.array.

**Arguments:**

*x (np.array):*  [x, y, yaw, v], shape: (4. 1).

**Returns**

*z (np.array):*  predicted measurement.

- <font color="#7fb800">run_step</font> (self, x, y, heading, velocity, yaw_rate_imu)
 
Apply EKF on current measurement and previous prediction.

**Arguments:**

*x (float):*  x(esu) coordinate from gnss sensor at current timestamp.

*y (float):*  y(esu) coordinate from gnss sensor at current timestamp.

*heading (float):*  heading direction at current timestamp.

*velocity (float):*  current speed.

*yaw_rate_imu (float):*  yaw rate rad/s from IMU sensor.

**Returns**

* xEST (np.array):*  The corrected x, y, heading, and velocity information.

- <font color="#7fb800">run_step_init</font> (self, x, y, heading, velocity)
 
Initalization for states.

**Arguments:**

*x (float):*  The X coordinate.

*y (float):*  Tehe y coordinate.

*heading (float):*  The heading direction. 

*velocity (float):*  The velocity

# opencda.customize.core.sensing.localization.localization_manager
Description for this module: Customized Localization Module.

# zed Localization Module.
## CustomizedLocalizationManager(vehicle, config_yaml, carla_map)
Customized Localization module to replace the default module.

### Parameters
- <font color="#f8805a">vehicle</font> (carla.Vehicle)
 
The carla.Vehicle. We need this class to spawn our gnss and imu sensor.


- <font color="#f8805a">config_yam</font> (dict)
 
The configuration dictionary of the localization module.


- <font color="#f8805a">carla_ma</font> (carla.Map)
 
The carla HDMap. We need this to find the map origin to convert wg84 to enu coordinate system.


### Attributes
- <font color="#f8805a">kf</font> (opencda object)
 
The filter used to fuse different sensors.


### Ancestors 
opencda.core.sensing.localization.localization_manager.LocalizationManager
# opencda.customize.ml_libs.ml_manager
Description for this module: Since multiple CAV normally use the same ML/DL model, here we have this class to enable different

## is_vehicle_cococlass(label)
Check whether the label belongs to the vehicle class according to coco dataset.

**Arguments:**

**Returns**

## MLManager()
A class that should contain all the ML models you want to initialize.

### Attributes
- <font color="#f8805a">object_detector</font> (torch_detector)
 
The YoloV5 detector load from pytorch.


### Methods 
- <font color="#7fb800">draw_2d_box</font> (self, result, rgb_image, index)
 
Draw 2d bounding box based on the yolo detection.

**Arguments:**

*result (yolo.Result):* Detection result from yolo 5.

*rgb_image (np.ndarray):*  Camera rgb image.

*index(int):*  Indicate the index.

**Returns**

*rgb_image (np.ndarray):*  camera image with bbx drawn

# opencda.scenario_testing.platoon_joining_2lanefree_carla
Description for this module: Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway simplified map sorely with carla

## run_scenario(opt, config_yaml)
# opencda.scenario_testing.platoon_joining_2lanefreecomplete_carla
Description for this module: Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway sorely with carla

## run_scenario(opt, config_yaml)
# opencda.scenario_testing.platoon_joining_town06_carla
Description for this module: Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway sorely with carla

## run_scenario(opt, config_yaml)
# opencda.scenario_testing.single_town06_carla
Description for this module: Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway simplified map sorely with carla

## run_scenario(opt, config_yaml)
# opencda.scenario_testing.single_2lanefree_carla
Description for this module: Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway simplified map sorely with carla

## run_scenario(opt, config_yaml)
# opencda.scenario_testing.platoon_stability_2lanefree_carla
Description for this module: Scenario testing: merging vehicle joining a platoon in the customized 2-lane freeway simplified map sorely with carla

## run_scenario(opt, config_yaml)
# opencda.scenario_testing.evaluations.utils
Description for this module: Utility functions for evaluation.

## lprint(logfile, *argv)
Save string to log file.

**Arguments:**

# opencda.scenario_testing.evaluations.evaluate_manager
Description for this module: Evaluation manager.

## EvaluationManager(cav_world)
Evaluation manager to manage the analysis of the results for different modules.

**Arguments:**

- <font color="#f8805a">cav_world</font> (opencda.CavWorld))
 
The CavWorld object that contains all CAVs' information.


### Methods 
- <font color="#7fb800">evaluate</font> (self)
 
Evaluate performance of all modules by plotting and writing the statistics into the log file.

- <font color="#7fb800">kinematics_eval</font> (self, log_file)
 
vehicle kinematics related evaluation.

**Arguments:**

*log_file (File):*  The log file to write the data.

- <font color="#7fb800">localization_eval</font> (self, log_file)
 
Localization module evaluation.

**Arguments:**

*log_file (File):*  The log file to write the data.

- <font color="#7fb800">platooning_eval</font> (self, log_file)
 
Platooning evaluation.

**Arguments:**

*log_file (File):*  The log file to write the data

# opencda.scenario_testing.utils.sim_api
Description for this module: Simulation API for create simulation world, vehicle manager and so on

## car_blueprint_filter(blueprints)
Exclude the uncommon vehicles from the default CARLA blueprint library (i.e., isetta, carlacola, cybertruck, t2).

## createPlatoonManagers(world, carla_map, scenario_params, apply_ml, map_helper=None)
Create Platooning Managers based on given params.

**Arguments:**

**Returns**

## createSimulationWorld(simulation_config, xodr_path=None, town=None)
Create client and simulation world.

**Arguments:**

## createTrafficManager(client, world, traffic_config)
Create background traffic.

**Arguments:**

**Returns**

## createVehicleManager(world, scenario_params, application, cav_world, carla_map, map_helper=None)
Create single CAV manager.

**Arguments:**

**Returns**

## destroyActors(world)
Destroy all actors in the world.
# opencda.scenario_testing.utils.yaml_utils
Description for this module: Used to load and write yaml files

## load_yaml(file)
Load yaml file and return a dictionary.

**Arguments:**

**Returns**

# opencda.scenario_testing.utils.customized_map_api
Description for this module: Loading world from customized map

## load_customized_world(xodr_path, client)
Load .xodr file and return the carla world object

**Arguments:**

## spawn_helper_2lanefree(carla_map, coefficient)
A helper function to locate the valid spawn point on the merge lane.

**Arguments:**

**Returns**

## spawn_helper_2lanefree_complete(carla_map, coefficient)
A helper function to locate the valid spawn point on the merge lane.

**Arguments:**

**Returns**

## bcolors()


### Class variables 
*BOLD*

*ENDC*

*FAIL*

*HEADER*

*OKBLUE*

*OKCYAN*

*OKGREEN*

*UNDERLINE*

*WARNING*

# opencda.co_simulation.sumo_integration.constants
Description for this module: This module defines constants used for the sumo-carla co-simulation.
# opencda.co_simulation.sumo_integration.bridge_helper
Description for this module: This module provides a helper for the co-simulation between sumo and carla .

## BridgeHelper()
BridgeHelper provides methos to ease the co-simulation between sumo and carla.

### Class variables 
*blueprint_library*

*data_json*

*dir_path*

*f*

*offset*

- <font color="#7fb800">get_carla_blueprint</font> (sumo_actor, sync_color=False)
 
Returns an appropriate blueprint based on the received sumo actor.

**Returns**

- <font color="#7fb800">get_carla_lights_state</font> (current_carla_lights, sumo_lights)
 
Returns carla vehicle light state based on sumo signals.

**Returns**

- <font color="#7fb800">get_carla_traffic_light_state</font> (sumo_tl_state)
 
Returns carla traffic light state based on sumo traffic light state.

**Returns**

- <font color="#7fb800">get_carla_transform</font> (in_sumo_transform, extent)
 
Returns carla transform based on sumo transform.

**Returns**

- <font color="#7fb800">get_sumo_lights_state</font> (current_sumo_lights, carla_lights)
 
Returns sumo signals based on carla vehicle light state.

**Returns**

- <font color="#7fb800">get_sumo_traffic_light_state</font> (carla_tl_state)
 
Returns sumo traffic light state based on carla traffic light state.

**Returns**

- <font color="#7fb800">get_sumo_transform</font> (in_carla_transform, extent)
 
Returns sumo transform based on carla transform.

**Returns**

- <font color="#7fb800">get_sumo_vtype</font> (carla_actor)
 
Returns an appropriate vtype based on the type id and attributes.
**Returns**

# opencda.co_simulation.sumo_integration.carla_simulation
Description for this module: This module is responsible for the management of the carla simulation.

## CarlaSimulation(host, port, step_length, xdor_path)
CarlaSimulation is responsible for the management of the carla simulation.

*traffic_light_ids*

### Methods 
- <font color="#7fb800">close</font> (self)
 
Closes carla client.

- <font color="#7fb800">destroy_actor</font> (self, actor_id)
 
Destroys the given actor.

- <font color="#7fb800">get_actor</font> (self, actor_id)
 
Accessor for carla actor.

- <font color="#7fb800">get_actor_light_state</font> (self, actor_id)
 
Accessor for carla actor light state.

- <font color="#7fb800">get_traffic_light_state</font> (self, landmark_id)
 
Accessor for traffic light state.

- <font color="#7fb800">spawn_actor</font> (self, blueprint, transform)
 
Spawns a new actor.

- <font color="#7fb800">switch_off_traffic_lights</font> (self)
 
Switch off all traffic lights.

- <font color="#7fb800">synchronize_traffic_light</font> (self, landmark_id, state)
 
Updates traffic light state.

- <font color="#7fb800">synchronize_vehicle</font> (self, vehicle_id, transform, lights=None)
 
Updates vehicle state.

- <font color="#7fb800">tick</font> (self)
 
Tick to carla simulation.
# opencda.co_simulation.sumo_integration.sumo_simulation
Description for this module: This module is responsible for the management of the sumo simulation.

## SumoActor(type_id, vclass, transform, signals, extent, color)
SumoActor(type_id, vclass, transform, signals, extent, color)

### Ancestors 
builtins.tuple

*color*

*extent*

*signals*

*transform*

*type_id*

*vclass*

## SumoActorClass(value, names=None, *, module=None, qualname=None, type=None, start=1)
SumoActorClass enumerates the different sumo actor classes.

### Ancestors 
builtins.tuple

### Class variables 
*ARMY*

*AUTHORITY*

*BICYCLE*

*BUS*

*COACH*

*CUSTOM1*

*CUSTOM2*

*DELIVERY*

*EMERGENCY*

*EVEHICLE*

*HOV*

*IGNORING*

*MOPED*

*MOTORCYCLE*

*PASSENGER*

*PEDESTRIAN*

*PRIVATE*

*RAIL*

*RAIL_ELECTRIC*

*RAIL_FAST*

*RAIL_URBAN*

*SHIP*

*TAXI*

*TRAILER*

*TRAM*

*TRUCK*

*VIP*

## SumoSignalState()
SumoSignalState contains the different traffic light states.

### Class variables 
*GREEN*

*GREEN_RIGHT_TURN*

*GREEN_WITHOUT_PRIORITY*

*OFF*

*OFF_BLINKING*

*RED*

*RED_YELLOW*

*YELLOW*

## SumoSimulation(cfg_file, step_length, host=None, port=None, sumo_gui=False, client_order=1)
SumoSimulation is responsible for the management of the sumo simulation.

- <font color="#7fb800">close</font> ()
 
Closes traci client.

- <font color="#7fb800">destroy_actor</font> (actor_id)
 
Destroys the given actor.

- <font color="#7fb800">get_actor</font> (actor_id)
 
Accessor for sumo actor.

- <font color="#7fb800">subscribe</font> (actor_id)
 
Subscribe the given actor to the following variables:

- <font color="#7fb800">unsubscribe</font> (actor_id)
 
Unsubscribe the given actor from receiving updated information each step.

*traffic_light_ids*

### Methods 
- <font color="#7fb800">get_net_offset</font> (self)
 
Accessor for sumo net offset.

- <font color="#7fb800">get_traffic_light_state</font> (self, landmark_id)
 
Accessor for traffic light state.

- <font color="#7fb800">spawn_actor</font> (self, type_id, color=None)
 
Spawns a new actor.

- <font color="#7fb800">switch_off_traffic_lights</font> (self)
 
Switch off all traffic lights.

- <font color="#7fb800">synchronize_traffic_light</font> (self, landmark_id, state)
 
Updates traffic light state.

- <font color="#7fb800">synchronize_vehicle</font> (self, vehicle_id, transform, signals=None)
 
Updates vehicle state.

- <font color="#7fb800">tick</font> (self)
 
Tick to sumo simulation.

## SumoTLLogic(tlid, states, parameters)
SumoTLLogic holds the data relative to a traffic light in sumo.

### Methods 
- <font color="#7fb800">get_all_landmarks</font> (self)
 
Returns all the landmarks associated with this traffic light.

**Returns**

- <font color="#7fb800">get_all_signals</font> (self)
 
Returns all the signals of the traffic light.

**Returns**

- <font color="#7fb800">get_associated_signals</font> (self, landmark_id)
 
Returns all the signals associated with the given landmark.

**Returns**

- <font color="#7fb800">get_number_signals</font> (self)
 
Returns number of internal signals of the traffic light.

**Returns**

## SumoTLManager()
SumoTLManager is responsible for the management of the sumo traffic lights (i.e., keeps control

- <font color="#7fb800">subscribe</font> (tlid)
 
Subscribe the given traffic ligth to the following variables:

- <font color="#7fb800">unsubscribe</font> (tlid)
 
Unsubscribe the given traffic ligth from receiving updated information each step.

### Methods 
- <font color="#7fb800">get_all_associated_signals</font> (self, landmark_id)
 
Returns all the signals associated with the given landmark.

**Returns**

- <font color="#7fb800">get_all_landmarks</font> (self)
 
Returns all the landmarks associated with this traffic light.

**Returns**

- <font color="#7fb800">get_all_signals</font> (self)
 
Returns all the signals of the traffic light.

**Returns**

- <font color="#7fb800">get_state</font> (self, landmark_id)
 
Returns the traffic light state of the signals associated with the given landmark.

**Returns**

- <font color="#7fb800">set_state</font> (self, landmark_id, state)
 
Updates the state of all the signals associated with the given landmark.

- <font color="#7fb800">switch_off</font> (self)
 
Switch off all traffic lights.

- <font color="#7fb800">tick</font> (self)
 
Tick to sumo simulation.

## SumoVehSignal()
SumoVehSignal contains the different sumo vehicle signals.

### Class variables 
*BACKDRIVE*

*BLINKER_EMERGENCY*

*BLINKER_LEFT*

*BLINKER_RIGHT*

*BRAKELIGHT*

*DOOR_OPEN_LEFT*

*DOOR_OPEN_RIGHT*

*EMERGENCY_BLUE*

*EMERGENCY_RED*

*EMERGENCY_YELLOW*

*FOGLIGHT*

*FRONTLIGHT*

*HIGHBEAM*

*WIPER*

# opencda.co_simulation.sumo_src.intersectionController
Description for this module: 

## IntersectionController(intersection, zip_flag=True)


### Methods 
- <font color="#7fb800">addPlatoon</font> (self, platoon)
 
Adds a platoon to this intersection controller

- <font color="#7fb800">calculateNewReservedTime</font> (self, pv, reservedTime)
 
Calculates the time that is needed to be reserved for a given platoon or vehicle (pv)

- <font color="#7fb800">findAndAddReleventPlatoons</font> (self, platoons)
 
Finds platoons in the given list that can be managed by this controller, then

- <font color="#7fb800">getNewSpeed</font> (self, pv, reservedTime)
 
Gets the speed the platoon or vehicle should adhere to in order to pass through the intersection safely

- <font color="#7fb800">getVehicleZipOrderThroughJunc</font> (self)
 
Gets the order that a platoon should [pass through the junction if zipping is enabled

- <font color="#7fb800">removeIrreleventPlatoons</font> (self)
 
Function to remove any platoons from the intersection that have either left the sphere of influence or left the map

- <font color="#7fb800">removePlatoon</font> (self, platoon)
 
Removes a platoon from this controller and then resets its behaviour to default

- <font color="#7fb800">update</font> (self)
 
Performs various functions to update the junction's state.

# opencda.co_simulation.sumo_src.platoon
Description for this module: Created on Tue Dec 1, 2020

## Platoon(startingVehicles)
Create a platoon, setting default values for all variables

### Methods 
- <font color="#7fb800">addControlledLanes</font> (self, lanes)
 

- <font color="#7fb800">addVehicle</font> (self, vehicle, index)
 
Adds a single vehicle to this platoon at the index provided

- <font color="#7fb800">canMerge</font> (self)
 
Returns True if this platoon can currently merge with another

**Returns**

- <font color="#7fb800">checkVehiclePathsConverge</font> (self, vehicles)
 

- <font color="#7fb800">disband</font> (self)
 
Marks a platoon as dead and returns vehicles to normal

- <font color="#7fb800">getAcceleration</font> (self)
 

- <font color="#7fb800">getAllVehicles</font> (self)
 
Retrieve the list of all the vehicles in this platoon

- <font color="#7fb800">getAllVehiclesByName</font> (self)
 
Retrieve the list of all the vehicles in this platoon by name

- <font color="#7fb800">getID</font> (self)
 
Generates and returns a unique ID for this platoon

- <font color="#7fb800">getLane</font> (self)
 

- <font color="#7fb800">getLanePositionFromFront</font> (self, lane=None)
 

- <font color="#7fb800">getLanesOfAllVehicles</font> (self)
 

- <font color="#7fb800">getLeadVehicle</font> (self)
 

- <font color="#7fb800">getLength</font> (self)
 
Gets the total length of the platoon

- <font color="#7fb800">getLengthOfSingleVehicle</font> (self)
 

- <font color="#7fb800">getMaxSpeed</font> (self)
 
Gets the maximum speed of the platoon

- <font color="#7fb800">getMergePosition</font> (self, relVeh, vehicle, direction)
 
Get the merge position and relevant vehicle(s)

- <font color="#7fb800">getNumberOfVehicles</font> (self)
 

- <font color="#7fb800">getSpeed</font> (self)
 

- <font color="#7fb800">getTargetSpeed</font> (self)
 

- <font color="#7fb800">isActive</font> (self)
 
Is the platoon currently active within the scenario

- <font color="#7fb800">mergePlatoon</font> (self, platoon)
 
Merges the given platoon into the current platoon

- <font color="#7fb800">removeControlledLanes</font> (self, lanes)
 
Removes the lanes from the platoon that were previously being controlled by an

- <font color="#7fb800">removeTargetSpeed</font> (self)
 
Removes the target speed from this platoon

- <font color="#7fb800">removeVehicle</font> (self, v)
 

- <font color="#7fb800">setEligibleForMerging</font> (self, canMerge)
 

- <font color="#7fb800">setGap</font> (self, gap)
 
Set the gap between vehicles in the platoon

- <font color="#7fb800">setSpeedMode</font> (self, speedMode)
 
Set the speed mode for every vehicle in the platoon.

- <font color="#7fb800">setTargetSpeed</font> (self, speed)
 
Sets a manual target speed for this platoon (normally determined by the lead

- <font color="#7fb800">startBehaviour</font> (self, vehicles)
 
A function to start platooning a specific set of vehicles

- <font color="#7fb800">stopBehaviour</font> (self)
 
Stops vehicles exhibiting platoon behaviour, if they are

- <font color="#7fb800">update</font> (self)
 
Performs updates to maintain the platoon

- <font color="#7fb800">updateIsActive</font> (self)
 
Is Active Update, if not disband
# opencda.co_simulation.sumo_src.utils
Description for this module: 

## add_vehicle(vid, position, lane, speed, cacc_spacing, real_engine=False)
Adds a vehicle to the simulation

## change_lane(vid, lane)
Let a vehicle change lane without respecting any safety distance

## communicate(topology)
Performs data transfer between vehicles, i.e., fetching data from

## get_distance(v1, v2)
Returns the distance between two vehicles, removing the length

## get_par(vid, par)
Shorthand for the getParameter method

## get_par_new(vid)
@Author: Yi

## get_pos(vid)
@Author: Yi

## running(demo_mode, step, max_step)
Returns whether the demo should continue to run or not. If demo_mode is

## set_par(vid, par, value)
Shorthand for the setParameter method

## start_sumo(config_file, already_running)
Starts or restarts sumo with the given configuration file

# opencda.co_simulation.sumo_src.vehicle
Description for this module: Created on Tue Dec 1, 2020

## Vehicle(vehicle)


### Methods 
- <font color="#7fb800">changeLane</font> (self, laneID, mode_num)
 

- <font color="#7fb800">getAcceleration</font> (self)
 

- <font color="#7fb800">getClosestDistances</font> (self, veh)
 

- <font color="#7fb800">getContext</font> (self)
 

- <font color="#7fb800">getDistance</font> (self, vehicle)
 

- <font color="#7fb800">getEdge</font> (self)
 

- <font color="#7fb800">getFollower</font> (self, range=20)
 

- <font color="#7fb800">getLane</font> (self)
 

- <font color="#7fb800">getLaneIndex</font> (self)
 

- <font color="#7fb800">getLanePosition</font> (self)
 

- <font color="#7fb800">getLanePositionFromFront</font> (self)
 

- <font color="#7fb800">getLeader</font> (self, range=20)
 

- <font color="#7fb800">getLeftFollower</font> (self)
 

- <font color="#7fb800">getLeftLeader</font> (self)
 

- <font color="#7fb800">getLength</font> (self)
 

- <font color="#7fb800">getMaxSpeed</font> (self)
 

- <font color="#7fb800">getName</font> (self)
 

- <font color="#7fb800">getPosition</font> (self)
 

- <font color="#7fb800">getRemainingRoute</font> (self)
 

- <font color="#7fb800">getRightFollower</font> (self)
 

- <font color="#7fb800">getRightLeader</font> (self)
 

- <font color="#7fb800">getRoute</font> (self)
 

- <font color="#7fb800">getSpeed</font> (self)
 

- <font color="#7fb800">getState</font> (self)
 

- <font color="#7fb800">getTargetLane</font> (self)
 

- <font color="#7fb800">getTau</font> (self)
 

- <font color="#7fb800">isActive</font> (self)
 

- <font color="#7fb800">setColor</font> (self, color)
 

- <font color="#7fb800">setImperfection</font> (self, imperfection)
 

- <font color="#7fb800">setInActive</font> (self)
 

- <font color="#7fb800">setMinGap</font> (self, minGap)
 

- <font color="#7fb800">setSpeed</font> (self, speed)
 

- <font color="#7fb800">setSpeedFactor</font> (self, speedFactor)
 

- <font color="#7fb800">setSpeedMode</font> (self, speedMode)
 

- <font color="#7fb800">setState</font> (self, state)
 

- <font color="#7fb800">setTargetLane</font> (self, targetLane)
 

- <font color="#7fb800">setTau</font> (self, tau)
 

# opencda.co_simulation.sumo_src.ccparams
Description for this module: 

## pack(*args)


## unpack(string)
# opencda.co_simulation.sumo_src.simlib
Description for this module: 

## flatten(l)


## setUpSimulation(configFile, trafficScale=1)
# opencda.co_simulation.sumo_src.simulationmanager
Description for this module: Created on Tue Dec 1, 2020

## SimulationManager(pCreation=True, iCoordination=True, iZipping=True)


### Methods 
- <font color="#7fb800">addNewVehicle</font> (self, vehicleID, vehicle)
 

- <font color="#7fb800">add_vehicles</font> (self, n, real_engine=False)
 
Adds a platoon of n vehicles to the simulation, plus an additional one

- <font color="#7fb800">checkPlatoonsNearbyAndUpdate</font> (self, platoon)
 

- <font color="#7fb800">createPlatoon</font> (self, vehicles)
 

- <font color="#7fb800">getActivePlatoons</font> (self)
 

- <font color="#7fb800">getAllVehicleNames</font> (self)
 

- <font color="#7fb800">getAllVehicles</font> (self)
 

- <font color="#7fb800">getAllVehiclesInPlatoons</font> (self)
 

- <font color="#7fb800">getAverageLengthOfAllPlatoons</font> (self)
 

- <font color="#7fb800">getPlatoonByLane</font> (self, lane)
 

- <font color="#7fb800">getPlatoonByVehicle</font> (self, v)
 

- <font color="#7fb800">getReleventPlatoon</font> (self, vehicle)
 

- <font color="#7fb800">getVehicleByID</font> (self, vehicleID)
 

- <font color="#7fb800">getVehicleType</font> (self, vehicleID)
 

- <font color="#7fb800">handleSimulationStep</font> (self, time)
 
Handle sumo simulation for each step

