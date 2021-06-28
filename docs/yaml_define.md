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
  vehicle_list: # define all human drive vehicles' position and individual speed
    - spawn_position: [x,y,z,pitch,yaw,roll]
      vehicle_speed_perc: -200 # this speed will overwrite the default traffic flow speed
    
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
