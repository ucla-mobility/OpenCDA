## Developer Tutorial

In this section, we will take a deeper look at the implementation details of several important classes in our OpenCDA framework. For beginners, we encourage you to go through our [OpenCDA tutorial](tutorial.md) first. This tutorial will emphasis on the design logics behind each class and try to give a clean description of the core algorithms of each module. For easy understanding, we have removed many assert and branch statements and evaluation/debug code to simplify the code flow. For complete working code, please refer to our [repo](https://github.com/ucla-mobility/OpenCDA). For details about our cooperative architecture, please refer to our [paper](https://arxiv.org/abs/2107.06260). 

### Workflow

The workflow of opencda can be summarized as follows. 

* Write yaml file to define various configurations of the simulation.
*  Load those configurations into a dictionary (`scenario_params`). For convience, we have provided `load_yaml` for loading the configurations. 
* Create `CavWorld` object to store information about registered vehicles and more importantly, this is the place to store the shared large models like neural networks. 
*  `ScenarioManager` will use those configurations to set `carla.TrafficManager` and load customized map if any.
* After the creation of `ScenarioManager`, users can use its `create_vehicle_manager` method to spawn Connected Automated Vehicles (CAVs) . Internally, each vehicle is managed by a `VehicleManager`, which will wrap the original `carla.Vehicle` with additional modules such as v2x manager, perception, localization, control and agent. Intuitively, `VehicleManager` is like the simulation characterization of the real vehicle with various sensors and the agent module will use the preceived information to reason and produce control actions, which will be passed to the control module. 
* Besides single CAVs, there are also platoons in the traffic. To spawn the platoons, users should use `create_platoon_manager` method. Similar as the previous signle CAV, `VehicleManager` can be used to wrap the vehicle. In addition,  `PlatooningManager` is used to maintain the information of all the vehicles in the same platoon. And after the creation of vehicles and platoons, it will return a list of `PlatooningManager`. 
* Similar as the process of spawing single CAV, `create_traffic_carla` method will spawn background traffic with `autopilot=True`. 
* Now we are ready to simulate various behaviors as the world ticks. Remember up till now, we have three type of traffic in our simulated world i.e., background traffic, single CAVs, platoons. For background traffic, since `autopilote=True`, we don't need special treatment. For platoon, we will update its information and use the gathered information to reason the correct controls for each vehicle within the platoon. For CAVs (`VehicleManager` object),, we need to first remove the vehicle that has been merged into the platoon. Afterwords, we will update its information to get the latest location and detection estimation and call `run_step` to reason the actual control command to reach our destination, considering the latest preceived world and neighboring dynamic road users.  And for each type, after getting the command, we will call controller to execute the command. And one iteration of simulation ends.

```python
from opencda.scenario_testing.utils.customized_map_api import customized_map_helper
def run_scenario(opt, config_yaml):
    scenario_params = load_yaml(config_yaml)
    xodr_path = "path/to/customized_map.xodr"
    # create CAV world
    cav_world = CavWorld(opt.apply_ml)
    # create scenario manager
    scenario_manager = \
    sim_api.ScenarioManager(scenario_params,opt.apply_ml,xodr_path=xodr_path,cav_world=cav_world)
    # create a list of single CAV
    single_cav_list = \
    scenario_manager.create_vehicle_manager(['platooning'],map_helper=customized_map_helper)
    # create platoon members
    platoon_list = scenario_manager.create_platoon_manager(data_dump=False)
    # create background traffic in carla
    traffic_manager, bg_veh_list = scenario_manager.create_traffic_carla()
    # run steps
    while True:
        scenario_manager.world.tick()
        # update vehicles within platoon
        for platoon in platoon_list:
            platoon.update_information()
            platoon.run_step()
        # update signle CAV
        for i, single_cav in enumerate(single_cav_list):
            # If the CAV is merged into one of the platoon, then we should let platoon manage it.
            # Thus we should remove them from single_cav_list
          	if single_cav.v2x_manager.in_platoon():
                single_cav_list.pop(i)
            # If the CAV is not contained in any platoon, then we should update the vehicle.
            else:
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)
```

### CavWorld

`CavWorld` will store CAVs/platooning information and more importantly it will store a shared Machine Learning model.  So we don't need to store a separate model for each CAV. <strong>if you plan to use a neural networks, this is a good place to load your model and call it in the customized algorithm class to utilize the model.</strong>

```python
class CavWorld(object):
    def __init__(self, apply_ml=False):
        # store the set of carla.Vehicle ids.
        self.vehicle_id_set = set()
        # store (VehicleManager.vid, VehicleManager) pairs
        self._vehicle_manager_dict = {}
        # store (PlatooningManger.pmid, PlatooningManger) pairs
        self._platooning_dict = {}
        self.ml_manager = None
        if apply_ml:
          	# add more ml models here
            self.ml_manager = function_to_load_ML_model()
```

### ScenarioManager

`ScenarioManager` sets simulation/traffic configurations and controls simulation construction, background traffic generation, and CAVs spawning. During the initialization stage, it will create the `client` and load the world with (customized) map into the variable `self.world`. Right now only synchronization mode is supported. In summary, in the initialization stage, it does the following job. 

```python
class ScenarioManager:
    def __init__(self, scenario_params,
                   apply_ml,
                   xodr_path=None,
                   town=None,
                   cav_world=None):
        # create the carla.Client
        self.client = carla.Client('localhost', simulation_config['client_port'])
        # load customized map if any
        self.world = load_customized_world(xodr_path, self.client)
        # change the world setting
        setting = self.world.get_settings()
        setting.synchronous_mode = True
        setting.fixed_delta_seconds = simulation_config['fixed_delta_seconds']
        self.world.apply_settings(new_settings)
        self.carla_map = self.world.get_map()
```

As we have seen in the workflow section, within this class, there are 3 important methods -- `create_vehicle_manager`, `create_platoon_manager` and `create_traffic_carla`. 

The `create_vehicle_manager` method will create a list of single CAVs according to the configurations of the yaml files (like the destination, desired speed/limits etc.).  The `create_platoon_manager` will create vehicles within platoons and create a platoon manager to group those vehicles together. The `create_traffic_carla` will create background traffic in carla and configure associated global TrafficManager behaviors. 

Now we will introduce each of them:

* `Create_vehicle_manager`

    The CAVs information is stored as a list in the yaml file. Each entry in the list corresponds to a CAV's configuration. We will create spawn positions specified in the yaml file and spawn the `carla.Vehicle` object. For each spawned vehicle, we will wrap it with the class `VehicleManager` , which essentially supports the localization, perception, platooning behavior etc. Details about this class can be found in the  `VehicleManager` section. 

```python
def create_vehicle_manager(self, application,
                               map_helper=None,
                               data_dump=False):
    # By default, we use lincoln as our cav model.
    cav_vehicle_bp = self.world.get_blueprint_library().find('vehicle.lincoln.mkz2017')
    single_cav_list = []
    # Each entry in the list corresponds to a CAV
    for i, cav_config in enumerate(self.scenario_params['scenario']['single_cav_list']):
        spawn_transform = function_to_load_spawn_position(cav_config)
        cav_vehicle_bp.set_attribute('color', '0, 0, 255')
        vehicle = self.world.spawn_actor(cav_vehicle_bp, spawn_transform)
        # create vehicle manager for each cav
        vehicle_manager = VehicleManager(vehicle, cav_config, application,...)
        self.world.tick()
        vehicle_manager.v2x_manager.set_platoon(None)
				# Set the destination of the vehicle according to the configuration
        destination = carla.Location(x=cav_config['destination'][0],
                                     y=cav_config['destination'][1],
                                     z=cav_config['destination'][2])
        # The `update_info` method will call the internal localization module and perception module
        # to update position and detected objects. 
        # Those information is then again passed to the v2x_manager/controller/BehaviorAgent module.
        vehicle_manager.update_info()
        vehicle_manager.set_destination(vehicle_manager.vehicle.get_location(),destination,clean=True)

        single_cav_list.append(vehicle_manager)

    return single_cav_list
```

* `create_platoon_manager`

    It will first loop over the predefined platoon list. For each platoon, we will create a `PlatooningManager` object to group all the vehicles within the platoon. In the current version, we assume the first vehicle in the platoon is the lead vehicle. After creating all of the vehicles of each platoon, it will return a list of `PlatooningManager`. As a result, we can control the behavior of each platoon via single API without worrying about details of how each vehicle will react. 

    ```python
    def create_platoon_manager(self, map_helper=None, data_dump=False):    
        platoon_list = []
        self.cav_world = CavWorld(self.apply_ml)
        # we use lincoln as default choice since our UCLA mobility lab use the
        # same car
        cav_vehicle_bp = \
        self.world.get_blueprint_library().find('vehicle.lincoln.mkz2017')
        # create platoons
        for i, platoon in enumerate(self.scenario_params['scenario']['platoon_list']):
          platoon_manager = PlatooningManager(platoon, self.cav_world)
          for j, cav in enumerate(platoon['members']):
              # Similar as spawning single CAV, we need to create its start location (spawn_transform)
              # and set its color etc. 
              ...
              vehicle = self.world.spawn_actor(cav_vehicle_bp,spawn_transform)
              # create vehicle manager for each cav
              vehicle_manager = VehicleManager(vehicle, cav, ['platooning'],
                        self.carla_map, self.cav_world,
                        current_time=self.scenario_params['current_time'],
                        data_dumping=data_dump)
              # add the vehicle manager to platoon
              if j == 0:
                  platoon_manager.set_lead(vehicle_manager)
              else:
                  platoon_manager.add_member(vehicle_manager, leader=False)
              self.world.tick()
              destination = carla.Location(x=platoon['destination'][0],
                                           y=platoon['destination'][1],
                                           z=platoon['destination'][2])
              platoon_manager.set_destination(destination)
              platoon_manager.update_member_order()
              platoon_list.append(platoon_manager)
              return platoon_list
    ```

* `create_traffic_carla`

    It will get the `carla.TrafficManager` and set associated parameters. Afterwords, it will spawn the background vehicles. For spawnning the vehicles, there are two options -- `spawn_vehicle_by_range` and `spawn_vehicles_by_list`. Depending on the way you configure them, the code will choose the associated one to do the task. Here for illustration, we use the `spawn_vehicles_by_list`. 

    ```python
    def create_traffic_carla(self):
        traffic_config = self.scenario_params['carla_traffic_manager']
        # get carla.TrafficManager
        tm = self.client.get_trafficmanager()
        tm.set_global_distance_to_leading_vehicle(
        traffic_config['global_distance'])
        tm.set_synchronous_mode(traffic_config['sync_mode'])
        tm.set_osm_mode(traffic_config['set_osm_mode'])
        tm.global_percentage_speed_difference(traffic_config['global_speed_perc'])
        bg_list = spawn_vehicles_by_list(tm, traffic_config, bg_list)
    
        return tm, bg_list
    ```

    The `spawn_vehicles_by_list` has similar structure as `create_vehicle_manager` with the support of randomness of the vehicles' apperance and colors. Notice that, different from CAVs, we will set autopilot to `True` for those background traffic and we will return a list of `carla.Vehicle` instead of the `VehicleManager`  used in the `Create_vehicle_manager`. 

### VehicleManager

This class will wrap the original `carla.Vehicle` object and associate the vehicles with various modules including localization, perception, control, agent and V2Xmanager. 

```python
class VehicleManager(object):
    def __init__(self, vehicle, config_yaml, application, 
                 carla_map, cav_world, current_time='',data_dumping=False):

        # an unique uuid for this vehicle
        self.vid = str(uuid.uuid1())
        self.vehicle = vehicle
        self.carla_map = carla_map

        # retrieve the configure for different modules
        sensing_config = config_yaml['sensing']
        behavior_config = config_yaml['behavior']
        control_config = config_yaml['controller']
        v2x_config = config_yaml['v2x']

        # v2x module
        self.v2x_manager = V2XManager(cav_world, v2x_config)
        # localization module
        self.localizer = LocalizationManager(vehicle, sensing_config['localization'], carla_map)
        # perception module
        self.perception_manager = PerceptionManager(
            vehicle, sensing_config['perception'], cav_world.ml_manager, data_dumping)
        # BehaviorAgent
        self.agent = BehaviorAgent(vehicle, carla_map, behavior_config)
        # Control module
        self.controller = ControlManager(config_yaml['controller'])
        cav_world.update_vehicle_manager(self)
```

The localization module will spawn the location-related sensor actors such as GNSS and IMU. And within the localization module, it will use Kalman filter to keep track of vehicle's locaiton and speed. 

The perception module will spawn perception-related sensors such as camera and LiDAR. If the ML model is applied (`self.active=True`), it will also detect the surrounding vehicles with the default Yolov5 detector. If the ML model is not applied (`self.active=False`), it will use server information directly and use the LiDAR to filter out vehicles out of the range. 

The agent module is a key component in our architecture. It will utilize the perception and localization information to reason the actual control command that should be passed into the downstream controller so that the desired destination can be reached.  There are two types of <strong>agents</strong> in our released codebase -- `BehaviorAgent` and `PlatooningBehaviorAgent`. `BehaviorAgent` is designed for the single vehicle while `PlatooningBehaviorAgent` has special methods to deal with the platooning behaviors. We will talk more about those classes in their sections. 

There are several commonly used methods within this class. Here we will brefiely talk about each of them. 

* `set_destination` 

    It will call the agent's `set_destimation` method to set the destination and prepare the global planner and local planner. Please refer to the localization module for details.

* `update_info`

    It will call localization module to get the latest position and velocity. Besides, it will call the perception module to get a list of obstacle vehicles. Afterwords, it will update the `agent` and `controller` as well to inform them about those changes. 

* `run_step`

    It will take in the `target_speed` and call `agent.run_step(target_speed)` to get the ideal speed and location that we want to reach in the next time frame. After that, it will pass the speed and location to the controller to generate the actual control command. And it will return the control command to the caller. 

* `destroy`

    It will destroy the vehicle and associated sensors. 

### PerceptionManager

`PerceptionManager` will spawn perception-related sensors including camera, LiDAR, and Semantic LiDAR. In this class, there are also many attributes for visualization. For the code simplicity, we have removed them from our sample code here. 

```python
class PerceptionManager:
    def __init__(self, vehicle, config_yaml, cav_world, data_dump=False):
        
				self.activate = config_yaml['activate']
        self.rgb_camera = []
        mount_position = ['front', 'right', 'left']
        for i in range(self.camera_num):
            self.rgb_camera.append(CameraSensor(vehicle, mount_position[i]))
        self.lidar = LidarSensor(vehicle, config_yaml['lidar'])
        if data_dump:
            self.semantic_lidar = SemanticLidarSensor(vehicle,
                                                      config_yaml['lidar'])
        # count how many steps have been passed
        self.count = 0
        # ego position
        self.ego_pos = None

        # the dictionary contains all objects
        self.objects = {}
```

For the current version, the main function we provide is detection. And there are two important methods in this class -- `detect` and `retrieve_traffic_lights`.

* `detect`

    It will detect surrounding vehicles by using specified model.  If `self.activate` flag is set, it will use the Yolov5 stored in the `CavWorld` to detect the obstacle vehicles. Otherwise, it will use the server information directly. 

    ```python
    def detect(self, ego_pos):
        self.ego_pos = ego_pos
        objects = {'vehicles': [],
        'traffic_lights': []}
        if not self.activate:
        		objects = self.deactivate_mode(objects)
        else:
        		objects = self.activate_mode(objects)
        self.count += 1
        return objects
    ```

* `retrieve_traffic_lights` 

    It will retrieve the traffic light states directly from the server. Thus in current version, we use ground truth to get the traffic light data. We may consider adding customized traffic light detection module in the next version. Researchers can also replace this method with their own traffic light detection algorithm by simply rewriting the following method. 

    ```python
    def retrieve_traffic_lights(self, objects):
        world = self.vehicle.get_world()
        tl_list = world.get_actors().filter('traffic.traffic_light*')
        objects.update({'traffic_lights': []})
        for tl in tl_list:
            distance = self.dist(tl)
            if distance < 50:
                objects['traffic_lights'].append(tl)
        return objects
    ```

### LocalizationManager

The `LocalizationManager` will spawn location-related sensors including GNSS and IMU. And it will use the Kalman filter to keep track of cars' location and speed. Though we read the speed directly from the server, to simulate the real world's uncertainty, noise is also added to the data retrieved from the server. And user can control the noise level by changing the parameters like `speed_stddev` in `yaml` file. 

```python
class LocalizationManager(object):
    def __init__(self, vehicle, config_yaml, carla_map):

        self.vehicle = vehicle
        self.activate = config_yaml['activate']
        self.map = carla_map
        self.geo_ref = self.map.transform_to_geolocation(
            carla.Location(x=0, y=0, z=0))

        # speed and transform and current timestamp
        self._ego_pos = None
        self._speed = 0

        # history track
        self._ego_pos_history = deque(maxlen=100)
        self._timestamp_history = deque(maxlen=100)

        self.gnss = GnssSensor(vehicle, config_yaml['gnss'])
        self.imu = ImuSensor(vehicle)

        # heading direction noise
        self.heading_noise_std = \
            config_yaml['gnss']['heading_direction_stddev']
        self.speed_noise_std = config_yaml['gnss']['speed_stddev']

        self.dt = config_yaml['dt']
        # Kalman Filter
        self.kf = KalmanFilter(self.dt)


```

### BehaviorAgent

The behavior agent will collect the information from the perception and localization module and use those information to reason the actual control command, which will be later passed to the controller.  There are two types of planner within the agent -- local planner and global planner. The global planner will generate the global path, considering the static map. The local planner will utilize the new preceived information to modify the global plan so that it can avoid dynamic obstacles and obey traffic rules. 

```python
class BehaviorAgent(object):
    def __init__(self, vehicle, carla_map, config_yaml):
      	# Load various parameters 
        ...
        self._sampling_resolution = config_yaml['sample_resolution']
        
        # collision checker
        self._collision_check = CollisionChecker(time_ahead=config_yaml['collision_time_ahead'])
       
        # used to indicate whether a vehicle is on the planned path
        self.hazard_flag = False

        # route planner related
        self._global_planner = None
        self.start_waypoint = None
        self.end_waypoint = None

        # intersection agent related
        self.light_state = "Red"
        self.light_id_to_ignore = -1

        # trajectory planner
        self._local_planner = LocalPlanner(self, carla_map, config_yaml['local_planner'])
        # special behavior rlated
        self.car_following_flag = False
        # lane change allowed flag
        self.lane_change_allowed = True
        # destination temp push flag
        self.destination_push_flag = False
        # white list of vehicle managers that the cav does not consider as
        # obstacles
        self.white_list = []
        self.obstacle_vehicles = []
```

* `update_information`

    The vehicleManager will call `agent.update_information` to update the position and speed as well as the detected objects. Afterterwards, the agent will update the local planner with the new speed/location information. For the detected objects, it may contain the vehicles that is about to join the platooning and those vehicles should be managed by the platooning manager. Thus we should remove those vehicles from the `objects` like shown below. Besides, we will also update the traffic light state here. 

    ```python
    def update_information(self, ego_pos, ego_speed, objects):
            # update localization information
            self._ego_speed = ego_speed
            self._ego_pos = ego_pos
            self.break_distance = self._ego_speed / 3.6 * self.emergency_param
            # update the localization info to trajectory planner
            self.get_local_planner().update_information(ego_pos, ego_speed)
            # The white list contains all position of target platoon member for joining. 
            # Those vehicles should be managed by platooning manager. Thus we should remove them.
            # Right now the white_list_match function will associate the vehicles 
            # based on their lane_id and location.
            obstacle_vehicles = objects['vehicles']
            self.obstacle_vehicles = self.white_list_match(obstacle_vehicles)
            # update the debug helper
            self.debug_helper.update(ego_speed, self.ttc)
            if self.ignore_traffic_light:
                self.light_state = "Green"
            else:
                # This method also includes stop signs and intersections.
                self.light_state = str(self.vehicle.get_traffic_light_state())
    ```

* `set_destination`

    Given the start location and end location, it will find the closest waypoints in the map to each of them. And we will use those 2 waypoints as the starting and end node. We will use the following code to make sure the starting node is always in fron of the vehicle.

    ```python
    _, angle = cal_distance_angle(self.start_waypoint.transform.location, cur_loc, cur_yaw)
    while angle > 90:
        self.start_waypoint = self.start_waypoint.next(1)[0]
        _, angle = cal_distance_angle(
        self.start_waypoint.transform.location, cur_loc, cur_yaw)
    ```

    And we will call `self._trace_route` to find the list of waypoints from the starting node to the end node. Then, we will store the route into the buffer `self.waypoints_queue`. 

* `_trace_route`

    This method will find the path from the `start_waypoint` to the `end_waypoint`. If the global plan has not been set before, it will load the `GlobalRoutePlanner` first. The algorithm for searching can be summarized as follows.

    * Given the `start_waypoint`, find the edge in the original graph containing this waypoint's location (achieved through `GlobalRoutePlanner._localize`.  And use the start node of the edge as the actual search origin. Through the same process, we can find the actual destination based on the provided `end_point`. 
    * Use A$^*$ algorithm to find the path from origin to destination.  The path will contain list of waypoints. The waypoint is either the start or end of a lane segment. Add the destination at the end of the path.
    * For each of the traversed edge in the path, we can find the corresponding turn decision (it is called RoadOption, e.g. lanefollow, left, right turn etc.). See `RoadOption` class for complete definition. Then, loop over the edges,
        * If the turn decision of the edge is lanefollow or void, then also add the `edge['path']` with associated RoadOption to the path.
        * Else, only add current_waypoint and a point  in the target lane with certain distance away. (see `GlobalRoutePlanner.trace_route` for details.)
    * Now we have a list of (waypoint, RoadOption) pairs. 
    * Add the pairs to the `waypoints_queue`. If the `clean` flag is on, also update the `_waypoint_buffer`. 

* `run_step` (<strong>details coming soon</strong>)

    This method contains the function of local planner and control command generation. In order to obey the traffic rules and consider the dynamic road elements, we have designed the following cases. Each case will have distinct driving behavior.

    * <strong>Destination arrived</strong>

        If the current location is near a radius of the destination ($10$ meters by default), the vehicle has arived at the destination and we will exit the agent. 

    * <strong>Red traffic light</strong>

        If the vehicle is in the junction and the traffic light is red, then return the break control command. Here we also consider the case when the car has moved to the center of the junction and the traffic light turns to green at current timestamp. For this case, it is very dangerious for the car to stop at the center of the junction. Thus we will use `light_id_to_ignore` to ignore this red light so that the car will continue moving. See [code](https://github.com/ucla-mobility/OpenCDA/blob/555aeab2bac7471d9400c51aea9c76741954b54b/opencda/core/plan/behavior_agent.py#L352) for details

    * <strong>Lane change behaviors</strong>
        * If the car is near intersection, for safety concern, the overtake and lane change are not allowed.
        * If the curvature of the road is high, the lane change is disabled. 
        * If lane change is allowed and the global plan indeed output a lane change path, then do collision check to see if the target lane is free. 

    * <strong>Car following</strong>

        If car following mode is on, follow the leading car.

    * <strong>Normal Mode</strong>

        For the normal mode, we will sample points along the path and return the target speed and target location. 

### LocalPlanner

LocalPlanner implements the basic behavior of following a trajectory of waypoints that is generated on-the-fly.The low-level motion of the vehicle is computed by using lateral and longitudinal PID controllers. When multiple paths are available (like in intersections) this local planner makes a random choice.

```python
class LocalPlanner(object):
    def __init__(self, agent, carla_map, config_yaml):
        self._vehicle = agent.vehicle
        self._map = carla_map

        self._ego_pos = None
        self._ego_speed = None

        # waypoint pop out thresholding
        self._min_distance = config_yaml['min_dist']
        self._buffer_size = config_yaml['buffer_size']

        # global route
        self.waypoints_queue = deque(maxlen=20000)
        # waypoint route
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        # trajectory buffer
        self._trajectory_buffer = deque(maxlen=30)
        self._history_buffer = deque(maxlen=3)
        self._long_plan_debug = []
        self.trajectory_update_freq = config_yaml['trajectory_update_freq']

        # trajectory sampling rate
        self.dt = config_yaml['trajectory_dt']

        # used to identify whether lane change is operated
        self.lane_change = False
        # In some corner cases, the id is not changed but we regard it as lane
        # change due to large lateral diff
        self.lane_id_change = False

        # debug option
        self.debug = config_yaml['debug']
        self.debug_trajectory = config_yaml['debug_trajectory']
```

* `update_information`

    Update the internal position and speed.

* `buffer_filter`

    If the vehicle needs to change the lane and the waypoint in the new lane is too close to the previous waypoint, the steering angle will be large in order to achieve this transformation. To avoid this case, we will remove waypoints that is too close when the lane change happens. 


### V2XManager

```python
class V2XManager(object):
    def __init__(self, cav_world, config_yaml, vid):
        # if disabled, no cooperation will be operated
        self.cda_enabled = config_yaml['enabled']
        self.communication_range = config_yaml['communication_range']

        # found CAVs nearby
        self.cav_nearby = {}

        # used for cooperative perception.
        self._recieved_buffer = {}

        # used for platooning communication
        self.platooning_plugin = PlatooningPlugin(
            self.communication_range, self.cda_enabled)

        self.cav_world = weakref.ref(cav_world)()

        self.ego_pos = None
        self.ego_spd = 0
        self.vid = vid
```

* `update_info`

    This method updates ego vehcle's speed and position and passes the updated information to `PlatooningPlugin`. Also this method searches all of the neighboring vehicles within range. To search the vehicles, we need to retreive a list of registered vehicles' information from `CavWorld`. For each vehicle in the list, we compute its distance with respect to the ego vehicle. If the distance is less than the threshold (`communication_range`), we consider it as a neighoring vehicle.  

