## Yaml Rule

To construct a scenario testing in OpenCDA, users have to first write a yml file to define the simulation parameters. To help reuse the parameters across different modules within the system, we adopt the `name anchor`. To know more details about the `named anchor` feature in yaml file, [read this blog](https://anil.io/blog/symfony/yaml/using-variables-in-yaml-files/). 

Below we demonstrate an example in `platoon_2lanefree_carla` scenario. Our example yaml files for various scenes are stored in the path`opencda/scenario_testing/config_yaml`. 

```yaml
# platoon_2lanefree_carla.yaml
world:  # define the CARLA server setting
  sync_mode: true # whether to use sync mode
  client_port: 2000 # client port to connect to the server
  fixed_delta_seconds: &delta 0.05 # fixed time step 
  weather: # set weather parameters
    sun_altitude_angle: 15 # 90 is the midday and -90 is the midnight
    cloudiness: 0 # 0 is the clean sky and 100 is the thickest cloud
    precipitation: 0 # rain, 100 is the heaviest rain
    precipitation_deposits: 0 # Determines the creation of puddles. Values range from 0 to 100, being 0 none at all and 100 a road completely capped with water.
    wind_intensity: 0 # it will influence the rain
    fog_density: 0 # fog thickness, 100 is the largest
    fog_distance: 0  # Fog start distance. Values range from 0 to infinite.
    fog_falloff: 0 # Density of the fog (as in specific mass) from 0 to infinity. The bigger the value, the more dense and heavy it will be, and the fog will reach smaller heights
    wetness: 0


vehicle_base: &vehicle_base # define cav default parameters
  sensing: &base_sensing # define sensing parameters
    perception: &base_perception # define perception related settings
    localization: &base_localize # define localization related settings
  map_manager: &base_map_manager # define HDMap manager
  behavior: &base_behavior # define planning related parameters
  controller: &base_controller # define controller
    type: pid_controller # define the type of controller
    args: ...  # define the detailed parmaters of the controller
  v2x: &base_v2x # define v2x configuration

carla_traffic_manager: # define the background traffic controled by carla.TrafficManager
  global_speed_perc: -100 # define the default speed of traffic flow
  auto_lane_change: false # whether lane change is allowed in the traffic flow
  ...: # some other parameters
  random: true # whether the car models of traffic flow are random selected
  vehicle_list: # define all human drive vehicles' position and individual speed
    - spawn_position: [x,y,z,pitch,yaw,roll]
      vehicle_speed_perc: -200 # this speed will overwrite the default traffic flow speed
  # this is another we to define carla traffic flow by giving the number of
  # spawn vehicles and the spawn range.
  # vehicle_list : 10 
  # range: [x1, y1, x2, y2]
    
platoon_base: &platoon_base # define the platoon default characteristics
  max_capacity: 10
  inter_gap: 0.6 # desired time gap
  open_gap: 1.5 # desired open gap during cut-in join
  warm_up_speed: 55 # required minimum speed before cooperative merging


scenario: # define each cav's spawn position, driving task, and parameters
  platoon_list: # define the platoons
    - <<: *platoon_base # the first platoon will take the default platoon parameters
      destination: [x, y, z] # platoon destination
      members: # define the first platoon's members
       - <<: *vehicle_base # the platoon leader(a cav) will take the default cav parameters
        spawn_position: ...
        behavior:
          <<: *base_behavior
          max_speed: 100 # overwrite the default target speed defined in &vehicle_base
        platoon: *platoon_base # add a new category 'platoon' to the origin vehicle parameters
        - <<: *vehicle_base # the second platoon mameber(a cav)
          ...
    single_cav_list: # define the cavs that are not in any platoon and aim to search and join one.
       - <<: *vehicle_base
         spawn_position: [x,y,z,pitch,yaw,roll]
         destination: [x, y, z]
         sensing: &base_sensing # point to default sensing setting
           ...: ... # overwrite the default sensing parameters for this cav
  
```
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
* Set the parameter `vehicle_list` under `carla_traffic_manager` as an integer. The CARLA server will then spawn
the same number of vehicles. If `vehicle_list` is an  integer, an additional parameter `range` needs to be set to
give the server the spawn area. In the example shown below, 5 vehicles will be randomly spawn in the restricted
rectangle area  `0<x<100, 22<y<334`.
```yaml
carla_traffic_manager:
    vehicle_list : 5
    range: [0, 100, 22, 334]
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