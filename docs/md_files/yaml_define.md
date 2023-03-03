## Yaml Rule

To create a scenario test in OpenCDA, you need to start by writing a yml file to define the simulation parameters. This can be a bit tricky, so we've provided a helpful starting point in the form of a yaml file called default.yaml.

The default.yaml file provides default parameters for a scenario, which you can modify as needed to create your own scenario. Instead of starting from scratch, you can use this file as a template and only change the parts that are different from the default parameters.

If you're not sure where to start, we've included example yaml files for various scenarios in the opencda/scenario_testing/config_yaml directory. You can use these as a guide or a starting point for your own scenario.

Below show an concrete example:
```yaml
# default.yaml
description: |-
  Copyright 2021 <UCLA Mobility Lab>
  Author: Runsheng Xu <rxx3386@ucla.edu>
  Content: This is the template scenario testing configuration file that other scenarios could directly refer

# define carla simulation setting
world:
  sync_mode: true
  client_port: 2000
  fixed_delta_seconds: 0.05
  seed: 11 # seed for numpy and random
  weather:
    sun_altitude_angle: 15 # 90 is the midday and -90 is the midnight
    cloudiness: 0 # 0 is the clean sky and 100 is the thickest cloud
    precipitation: 0 # rain, 100 is the heaviest rain
    precipitation_deposits: 0 # Determines the creation of puddles. Values range from 0 to 100, being 0 none at all and 100 a road completely capped with water.
    wind_intensity: 0 # it will influence the rain
    fog_density: 0 # fog thickness, 100 is the largest
    fog_distance: 0  # Fog start distance. Values range from 0 to infinite.
    fog_falloff: 0 # Density of the fog (as in specific mass) from 0 to infinity. The bigger the value, the more dense and heavy it will be, and the fog will reach smaller heights
    wetness: 0


# Define the basic parameters of the rsu
rsu_base:
  sensing:
    perception:
      activate: false # when not activated, objects positions will be retrieved from server directly
      camera:
        visualize: 4 # how many camera images need to be visualized. 0 means no visualization for camera
        num: 4 # how many cameras are mounted on the vehicle. Maximum 3(frontal, left and right cameras)
        # relative positions (x,y,z,yaw) of the camera. len(positions) should be equal to camera num
        positions:
          - [2.5, 0, 1.0, 0]
          - [0.0, 0.3, 1.8, 100]
          - [0.0, -0.3, 1.8, -100]
          - [-2.0, 0.0, 1.5, 180]
      lidar: # lidar sensor configuration, check CARLA sensor reference for more details
        visualize: true
        channels: 32
        range: 120
        points_per_second: 1000000
        rotation_frequency: 20 # the simulation is 20 fps
        upper_fov: 2
        lower_fov: -25
        dropoff_general_rate: 0.3
        dropoff_intensity_limit: 0.7
        dropoff_zero_intensity: 0.4
        noise_stddev: 0.02
    localization:
      activate: true # when not activated, ego position will be retrieved from server directly
      dt: ${world.fixed_delta_seconds} # used for kalman filter
      gnss: # gnss sensor configuration
        noise_alt_stddev: 0.05
        noise_lat_stddev: 3e-6
        noise_lon_stddev: 3e-6

# Basic parameters of the vehicles
vehicle_base:
  sensing: # include perception and localization
    perception:
      activate: false # when not activated, objects positions will be retrieved from server directly
      camera:
        visualize: 1 # how many camera images need to be visualized. 0 means no visualization for camera
        num: 1 # how many cameras are mounted on the vehicle.
        positions:  # relative positions (x,y,z,yaw) of the camera. len(positions) should be equal to camera num
          - [2.5, 0, 1.0, 0]
      lidar: # lidar sensor configuration, check CARLA sensor reference for more details
        visualize: true
        channels: 32
        range: 50
        points_per_second: 100000
        rotation_frequency: 20 # the simulation is 20 fps
        upper_fov: 10.0
        lower_fov: -30.0
        dropoff_general_rate: 0.0
        dropoff_intensity_limit: 1.0
        dropoff_zero_intensity: 0.0
        noise_stddev: 0.0

    localization:
      activate: false # when not activated, ego position will be retrieved from server directly
      dt: ${world.fixed_delta_seconds} # used for kalman filter
      gnss: # gnss sensor configuration
        noise_alt_stddev: 0.001
        noise_lat_stddev: 1.0e-6
        noise_lon_stddev: 1.0e-6
        heading_direction_stddev: 0.1 # degree
        speed_stddev: 0.2
      debug_helper:
        show_animation: false # whether to show real-time trajectory plotting
        x_scale: 1.0 # used to multiply with the x coordinate to make the error on x axis clearer
        y_scale: 100.0 # used to multiply with the y coordinate to make the error on y axis clearer

  map_manager:
    pixels_per_meter: 2 # rasterization map resolution
    raster_size: [224, 224] # the rasterize map size (pixel)
    lane_sample_resolution: 0.1 # for every 0.1m, we draw a point of lane
    visualize: true # whether to visualize the rasteraization map
    activate: true # whether activate the map manager

  safety_manager: # used to watch the safety status of the cav
    print_message: true # whether to print the message if hazard happens
    collision_sensor:
      history_size: 30
      col_thresh: 1
    stuck_dector:
      len_thresh: 500
      speed_thresh: 0.5
    offroad_dector: [ ]
    traffic_light_detector: # whether the vehicle violate the traffic light
      light_dist_thresh: 20

  behavior:
    max_speed: 111 # maximum speed, km/h
    tailgate_speed: 121 # when a vehicles needs to be close to another vehicle asap
    speed_lim_dist: 3 # max_speed - speed_lim_dist = target speed
    speed_decrease: 15 # used in car following mode to decrease speed for distance keeping
    safety_time: 4 # ttc safety thresholding for decreasing speed
    emergency_param: 0.4 # used to identify whether a emergency stop needed
    ignore_traffic_light: true # whether to ignore traffic light
    overtake_allowed: true # whether overtake allowed, typically false for platoon leader
    collision_time_ahead: 1.5 # used for collision checking
    overtake_counter_recover: 35 # the vehicle can not do another overtake during next certain steps
    sample_resolution: 4.5 # the unit distance between two adjacent waypoints in meter
    local_planner:  # trajectory planning related
      buffer_size: 12 # waypoint buffer size
      trajectory_update_freq: 15 # used to control trajectory points updating frequency
      waypoint_update_freq: 9 # used to control waypoint updating frequency
      min_dist: 3 # used to pop out the waypoints too close to current location
      trajectory_dt: 0.20 # for every dt seconds, we sample a trajectory point from the trajectory path as next goal state
      debug: false # whether to draw future/history waypoints
      debug_trajectory: false # whether to draw the trajectory points and path

  controller:
    type: pid_controller # this has to be exactly the same name as the controller py file
    args:
      lat:
        k_p: 0.75
        k_d: 0.02
        k_i: 0.4
      lon:
        k_p: 0.37
        k_d: 0.024
        k_i: 0.032
      dynamic: false # whether use dynamic pid setting
      dt: ${world.fixed_delta_seconds} # this should be equal to your simulation time-step
      max_brake: 1.0
      max_throttle: 1.0
      max_steering: 0.3
  v2x:  # communication related
    enabled: true
    communication_range: 35


# define the background traffic control by carla
carla_traffic_manager:
  sync_mode: true # has to be same as the world setting
  global_distance: 5 # the minimum distance in meters that vehicles have to keep with the rest
  # Sets the difference the vehicle's intended speed and its current speed limit.
  #  Carla default speed is 30 km/h, so -100 represents 60 km/h,
  # and 20 represents 24 km/h
  global_speed_perc: -100
  set_osm_mode: true # Enables or disables the OSM mode.
  auto_lane_change: false
  ignore_lights_percentage: 0 # whether set the traffic ignore traffic lights
  random: false # whether to random select vehicles' color and model
  vehicle_list: []  # define in each scenario. If set to ~, then the vehicles be spawned in a certain range
  # Used only when vehicle_list is ~
  # x_min, x_max, y_min, y_max, x_step, y_step, vehicle_num
  range: []

# define the platoon basic characteristics
platoon_base:
  max_capacity: 10
  inter_gap: 0.6 # desired time gap
  open_gap: 1.2 # open gap
  warm_up_speed: 55 # required speed before cooperative merging
  change_leader_speed: true # whether to assign leader multiple speed to follow
  leader_speeds_profile: [ 85, 95 ] # different speed for leader to follow
  stage_duration: 10 # how long should the leader keeps in the current velocity stage

# define tne scenario in each specific scenario
scenario:
  single_cav_list: []
  platoon_list: []
```
The above yaml file is the `default.yaml`. If the users wants to create a platoon joining scenario in highway,
here is how we create `platoon_joining_2lanefree_carla.yaml`:

```yaml
# platoon_joining_2lanefree_carla.yaml
vehicle_base:
  sensing:
    perception:
      camera:
        visualize: 0 # how many camera images need to be visualized. 0 means no visualization for camera
        num: 0 # how many cameras are mounted on the vehicle. Maximum 3(frontal, left and right cameras)
        # relative positions (x,y,z,yaw) of the camera. len(positions) should be equal to camera num
        positions: []
      lidar:
        visualize: false
  map_manager:
    visualize: false
    activate: false
  behavior:
    max_speed: 95 # maximum speed, km/h
    tailgate_speed: 105 # when a vehicles needs to be close to another vehicle asap
    overtake_allowed: false # whether overtake allowed, typically false for platoon leader
    collision_time_ahead: 1.3 # used for collision checking
    overtake_counter_recover: 35 # the vehicle can not do another overtake during next certain steps
    local_planner:
      trajectory_dt: 0.25 # for every dt seconds, we sample a trajectory point from the trajectory path as next goal state

# define the platoon basic characteristics
platoon_base:
  max_capacity: 10
  inter_gap: 0.6 # desired time gap
  open_gap: 1.5 # open gap
  warm_up_speed: 55 # required speed before cooperative merging

# define the background traffic control by carla
carla_traffic_manager:
  global_distance: 4.0 # the minimum distance in meters that vehicles have to keep with the rest
  # Sets the difference the vehicle's intended speed and its current speed limit.
  #  Carla default speed is 30 km/h, so -100 represents 60 km/h,
  # and 20 represents 24 km/h
  global_speed_perc: -300
  vehicle_list:
    - spawn_position: [-285, 8.3, 0.3, 0, 0, 0]
    - spawn_position: [-310, 8.3, 0.3, 0, 0, 0]
    - spawn_position: [-390, 8.3, 0.3, 0, 0, 0]
    - spawn_position: [-320, 4.8, 0.3, 0, 0, 0]
      vehicle_speed_perc: -200
    - spawn_position: [-335, 4.8, 0.3, 0, 0, 0]
    - spawn_position: [-360, 4.8, 0.3, 0, 0, 0]
    - spawn_position: [-400, 4.8, 0.3, 0, 0, 0]
    - spawn_position: [-410, 4.8, 0.3, 0, 0, 0]

# define scenario. In this scenario, a 4-vehicle platoon already exists.
scenario:
  platoon_list:
    - name: platoon1
      destination: [1000.372955, 8.3, 0.3]
      members: # the first one is regarded as leader by default
        - name: cav1
          spawn_position: [-350, 8.3, 0.3, 0, 0, 0] # x, y, z, roll, yaw, pitch
          perception:
            camera:
              visualize: 1 # how many camera images need to be visualized. 0 means no visualization for camera
              num: 1 # how many cameras are mounted on the vehicle. Maximum 3(frontal, left and right cameras)
              # relative positions (x,y,z,yaw) of the camera. len(positions) should be equal to camera num
              positions:
                - [2.5, 0, 1.0, 0]
            lidar:
              visualize: true
          behavior:
            local_planner:
              debug_trajectory: true
              debug: false
        - name: cav2
          spawn_position: [-360, 8.3, 0.3, 0, 0, 0]
        - name: cav3
          spawn_position: [-370, 8.3, 0.3, 0, 0, 0]
        - name: cav4
          spawn_position: [-380, 8.3, 0.3, 0, 0, 0]
  single_cav_list: # this is for merging vehicle or single cav without v2x
    - name: single_cav
      spawn_position: [-380, 4.8, 0.3, 0, 0, 0]
      # when this is defined, the above parameter will be ignored, and a special map function will
      # be used to define the spawn position based on the argument
      spawn_special: [0.625]
      destination: [300, 12.0, 0]
      sensing:
        perception:
          camera:
            visualize: 1 # how many camera images need to be visualized. 0 means no visualization for camera
            num: 1 # how many cameras are mounted on the vehicle. Maximum 3(frontal, left and right cameras)
            # relative positions (x,y,z,yaw) of the camera. len(positions) should be equal to camera num
            positions:
              - [2.5, 0, 1.0, 0]
          lidar:
            visualize: true
      v2x:
        communication_range: 35
      behavior:
        overtake_allowed: true
        local_planner:
          debug_trajectory: true
          debug: false
```
As you can see, the `platoon_joining_2lanefree_carla.yaml` only contains the part that `default.yaml` does not have or has different
parameters. 

### Detailed Explanation

---
#### world
The parameter `world` in the yaml defines the CARLA server setting.
* `sync` : boolean type, if true, the simulation will be in sync mode, otherwise async mode. Check 
the [CARLA Sync documentation](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/)  to know more.
* `client_port` : the python client port connected to the CARLA server.
* `fixed_delta_seconds` : The elapsed time remains constant between simulation steps.
 If it is set to 0.05 seconds, there will be 20 frames per simulated second.

 #### vehicle_base
 `vehicle_base` defines the default parameters for each CAV including perception, localization, planning, control,
 and v2x modules. The ampersand `&` character before `vehicle_base` is ued to to create a `named anchor`, 
 which be referenced later on with an asterisk `*`. 

 * `sensing` :  Define perception and localization related parameters.
    * `perception`:  Module related to object detection.
        * `activate` : bool type, if false, the CAV will retrieve object positions from the server directly. 
        Otherwise, yolov5 will be used for object detection.
        * `camera_visualize` : int type, indicates how many camera rgb images should be visualized during
        simulation. 0 means no rgb image visualization.
        * `camera_num` : int type, indicates how many cameras are mounted in the CAV, e.g. 3 represents a frontal,
        a left, and a right camera will be mounted on the top of the vehicle to cover a 270 degree of FOV.
        * `lidar_visualize` : bool type, indicates whether to visualize 3d lidar points during simulaiton.
        * `lidar` : set the configuration of the lidar sensor.
     * `localization` : Module related to self-localization.
        * `activate` : bool type, if true, the CAV will use gnss+imu+kf to get ego vehicle position. Otherwise, 
        the CAV will load the ego position from server directly.
        * `gnss` : related to the parameters of the gnss sensor.
        * `debug_helper` : parameters related to localization debugging and real-time trajectory plotting.
 * `map_manager` : Define HDMap manager parameters
   * `pixels_per_meter` : The rasterization map precision.
   * `raster_size` : The rasterization map's H and W in pixels.
   * `lane_sample_resolution` : Waypoint sampling resolution for lane drawing.
   * `visualize` : Whether to show the rasterization map during running in real-time.
   * `activate` : Whether activate the map_manager module.
 * `behavior` : Define behavior planning parameters
     * `max_speed` : int type, the maximum speed (km/h) that the CAV is allowed to reach.
     * `tailgate_speed` : int type, the target speed (km/h) for CAV when it tries to catch up with a platoon, it is usually larger
     than `max_speed`
     * `speed_lim_dist` : int type, during normal driving mode, `target_speed` = `max_speed` - `speed_lim_dist`
     * `speed_decrease` : int type, when the CAV is in car following mode and it gets too close to the
     front vehicle, `target_speed` = `front_vehicle_speed` - `speed_decrease`
     * `safety_time` : float type, ttc thresholding to identify whether the ego vehicle is too close to 
     the front vehicle.
     * `emergency_param` : float type, `emergency_stop_distance` = `current_speed` * `emergency_param`
     * `ignore_traffic_light` : bool type, if set to true, the CAV will ignore the traffic light.
     * `overtake_allowed` :  bool type, if set to false, overtaking is not allowed during driving.
     * `collision_time_ahead` : float type, collision detection range
     * `sample_resolution` : float type, the unit distance (m) between two adjacent waypoints
     * `local_planner` : Define trajectory planning parameters.
         * `buffer_size` : dequeue type, waypoint buffer size.
         * `trajectory_update_freq` : int type, the update frequency for trajectory, when the length of trajectory buffer 
         is below the frequency number, the ego vehicle will re-generate the trajectory.
         * `waypoint_update_freq` : int type, the update frequency for waypoint buffer, when the length of the 
         waypoint buffer is below the frequency, the waypoint buffer will load waypoints from `waypoint_queue`.
         * `min_dist` : float type, used to pop out the waypoints that are too close to the current location
         * `trajectory_dt` : float type, trajectory points sampling resolution.
         * `debug` : bool type, if true, waypoint will be visualized.
         * `debug_trajectory` : bool type, if true, trajectory will be visualized.
 * `controller` : Define controller parameters.
     * `type` : string type, the type of controller the ego vehicle uses.
     * `args` : the arguments related to the selected controller.

 * `v2x` : Defome vehicle communication parameters.
     * `enabled` : bool type, indicate whether v2x is enabled.
     * `communication_range` : float type, the searching range of the CAV
     * `loc_noise` : float type, the deviation of the noise added to the received ego position 
     during communication.
     * `yaw_noise` : float type, the deviation of the noise added to the received yaw angle
     during communication.
     * `speed_noise` : float type, the deviation of the noise added to the received ego speed 
     during communication.
     * `lag` : int type, the lagging during the communication. E.g., 2 means the CAV
      receives the packages of other CAVs at most 2 time steps ago. 

#### platoon_base
`platoon_base` define the default platooning parameters.
* `max_capacity` : int type, the maximum number of members that the platoon can include.
* `inter_gap` : float type, desired time gap.
* `open_gap` : float type, time gap during cut-in-join.
* `warm_up_speed` : float type, the speed that the merging vehicle needs to reach before do any
kind of joining.

#### carla_traffic_manager
`carla_traffic_manager` defines the traffic flow controlled by <strong>CARLA</strong> traffic manager. <strong>
Users do not need to define this parameter if co-simulation is conducted as Sumo will control the traffic.</strong>

There are two ways to define the positions of the background vehicles. 
* Set the parameter `vehicle_list` under `carla_traffic_manager` as a list. An example is demonstrated
below. In this example, two vehicles are spawned as background vehicle. The first one is spawned at position `x=100, y=100, z=0.3`,
and the initial rotation angle is `roll=0, yaw=20 (degree), pitch=0`. The second one is spawned at position 
`x=122, y=666, z=0.3`, and the angle is `roll=0, yaw=0, pitch=0`.
```yaml
carla_traffic_manager:
  vehicle_list: 
    - spawn_position: [100, 100, 0.3, 0 , 20, 0]
    - spawn_position: [122, 666, 0.3, 0 , 0, 0]
```
* Set the parameter `vehicle_list` under `carla_traffic_manager` as `~`. The CARLA server will then spawn
the vehicles randomly in a certain rectangle range given by the additional parameter `range`. 
```yaml
carla_traffic_manager:
  vehicle_list: ~  # a number or a list
  # Used only when vehicle_list is a number.
  # x_min, x_max, y_min, y_max, x_step, y_step, veh_num
  range:
    - [ 2, 10, 0, 200, 3.5, 25, 30]

```
Other important parameters:
* `sync_mode` : bool type, it should be consistent with server's sync setting.
* `global_speed_perc` : float type, sets the difference the vehicle's intended speed and its current speed limit. 
Speed limits can be exceeded by setting the number to a negative value. Default is 30 km/h. Exceeding a speed limit can be done using negative percentages.
For example, -300 will assign a speed of 90, 50 will assign a speed of 15.
* `auto_lane_change` : bool type, whether the vehicles are allowed to do lane change.
* `random` : bool type, if set true, the background traffic will randomly select car model and color. Otherwise,
all vehicles will be in lincoln mkz model and green color.

#### scenario
`scenario` defines each CAV's spawn position and vehicle parameters if different from the default setting `vehicle_base`.
```yaml
scenario:
  platoon_list:
    - <<: *platoon_base
      destination: [1000.372955, 8.3, 0.3]
      members: # the first one is regarded as leader by default
        - <<: *vehicle_base
          spawn_position: [-350, 8.3, 0.3, 0, 0, 0] # x, y, z, roll, yaw, pitch
          behavior:
            <<: *base_behavior
            overtake_allowed: false
          platoon: # we need to add platoon specific params
            <<: *platoon_base
        - <<: *vehicle_base
          spawn_position: [-360, 8.3, 0.3, 0, 0, 0]
          platoon: # we need to add platoon specific params
            <<: *platoon_base
   single_cav_list: 
    - <<: *vehicle_base
      spawn_position: [-380, 4.8, 0.3, 0, 0, 0]
      destination: [300, 12.0, 0]
      sensing:
        <<: *base_sensing
        perception:
          <<: *base_perception
          activate: true
```
In the above example, a platoon containing two members and a single CAV that is out of any platoon will be spawn.
The `destination` in the platoon sets the destination of the platoon. The first member of platoon is regarded
as the leader by default, and the second member will be the following car. All members in the same platoon should
be spawn in the same lane with close distance.  The `<<: *vehicle_base` will load the whole default setting of CAV 
into the leader. However, since overtake is not allowed for a platoon leader and the default setting allows so, this
needs to be changed by using the following part:
```yaml
behavior:
  <<: *base_behavior
  overtake_allowed: false   
```
In this way, the default attribute  `overtake_allowed` will be overwritten to false while keeping other
attributes unchanged. Similarly, the default CAV setting does not have the `platoon` attribute, thus we also 
add `platoon: <<: *platoon_base` to each member to assign the `platoon` attribute.

For the single CAV, the meaning of the parameters are quite similar with the platoon members we just described.

#### sumo (optional)
`sumo` needs to be set only when co-simulation is required.

```yaml
sumo:
  port: ~
  host: ~
  gui: true
  client_order: 1
  step_length: *delta
```
* `port` : int type, TCP port to listen to (default: 8813).
* `host` : str type, IP of the sumo host server (default: 127.0.0.1).
* `gui` : bool type, when set to true, Sumo gui will be show.
* `client_order` : int type, client order number for the co-simulation TraCI connection (default: 1).
* `step_length` : The elapsed time remains constant between simulation steps. It should be the same
as the CARLA server.