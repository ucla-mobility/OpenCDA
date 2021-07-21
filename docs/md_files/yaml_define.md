## OpenCDA Yaml Rule

To construct a scenario testing in OpenCDA, users have to first write a yml file to 
define the simulation parameters. Below we demonstrate an example in `platoon_2lanefree_carla` scenario.
```yaml
# platoon_2lanefree_carla.yaml
world:  # define the CARLA server setting
  sync_mode: true # whether to use sync mode
  client_port: 2000 # client port to connect to the server
  fixed_delta_seconds: &delta 0.05 # fixed time step 

vehicle_base: &vehicle_base # define cav default parameters
  sensing: &base_sensing # define sensing parameters
    perception: &base_perception # define perception related settings
    localization: &base_localize # define localization related settings
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
* `sync` : bool type, if true, the simulation will be in sync mode, otherwise async mode. Check 
the [CARLA Sync documentation](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/)  to know more.
* `client_port` : the python client port connected to the CARLA server.
* `fixed_delta_seconds` : The elapsed time remains constant between simulation steps.
 If it is set to 0.05 seconds, there will be 20 frames per simulated second.
 
 #### vehicle_base
 `vehicle_base` defines the default parameters for each CAV including perception, localization, planning, control,
 and v2x modules. The ampersand `&` character before `vehicle_base` is ued to to create a `named anchor`, 
 which we can then reference later on with an asterisk `*`. To know more details about the `named anchor` feature
 in yaml file, [read this blog](https://anil.io/blog/symfony/yaml/using-variables-in-yaml-files/) .
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
* `behavior` : Define behavior planning parameters
        
 