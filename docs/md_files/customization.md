## Algorithm Customization

Due to the high modularity of OpenCDA, you can conveniently replace any default module with your own
algorithms. The best way for customization is to <strong>put your customized module under `opencda/customize/...` , </strong> and 
<strong>use inheritance to overwrite the default algorithms</strong>. Afterwards, import your customized module in
`VehicleManager` class. The only thing you need to pay attention is to make the input and output format the same
as origin module if you only want to change a single module. Below we will show a detailed instruction 
of customizations for each module.

---
### Localization Customization
The class `LocalizationManager` takes charges of the localization task. During initialization, a gps and imu sensor will
be spawned to collect localization related information. 

The core function in it is `localize()`, which doesn't take any input, and aims to fill the correct value for
`self._ego_pos` and `_speed`.
The default algorithm to fuse the gps and imu data is the Kalman Filter. It takes the current gps and imu data as input,
and return the corrected `x, y, z` coordinates.

```python
from opencda.core.sensing.localization.kalman_filter import KalmanFilter

class LocalizationManager(object):
     def __init__(self, vehicle, config_yaml, carla_map):
        self.kf = KalmanFilter(self.dt)
     
     def localize(self):
        ...
        corrected_cords = self.kf(x, y, z, speed, yaw, imu_yaw_rate)
```
If a user wants to remain the whole structure of localization and just replace the filter (e.g. Extended Kalman Filter),
then he/she just needs to create a `localization_manager.py` under `opencda/customize/core/sensing/localization`
folder and initializes the `CustomizedLocalizationManager` with Extended Kalman Filter:

```python
from opencda.core.sensing.localization.localization_manager import LocalizationManager
from opencda.customize.core.sensing.localization.extented_kalman_filter import ExtentedKalmanFilter

class CustomizedLocalizationManager(LocalizationManager):
    def __init__(self, vehicle, config_yaml, carla_map):
        super(CustomizedLocalizationManager, self).__init__(vehicle, config_yaml, carla_map)
        self.kf = ExtentedKalmanFilter(self.dt)
```

Then go to `VehicleManager` class, import this customized module and set it as the localizer.
```python
from opencda.core.sensing.localization.localization_manager import LocalizationManager
from opencda.customize.core.sensing.localization.localization_manager import CustomizedLocalizationManager

class VehicleManager(object):
    def __init__(self, vehicle, config_yaml, application, carla_map, cav_world):
        # self.localizer = LocalizationManager(vehicle, sensing_config['localization'], carla_map)
        self.localizer = CustomizedLocalizationManager(vehicle, sensing_config['localization'], carla_map)
```
If the users want to modify more (e.g. change the data pre-processing), as long as they remember to fill `self._ego_pos`
and `self._speed` in the `localize()` function under `CustomizedLocalizationManager`, it will not cause any problem
for the downstream modules.

---
### Perception Customization
The class `PerceptionManager` is responsible for perception related task. Right now it supports vehicle detection and traffic light detection. The core function `detect(ego_pos)` takes the ego position from localization module as the input, and return a dictionary `objects` whose keys are the object categories and the values are each object's attributes (e.g. 3d poses, static or dynamic) under world coordinate system in this category.

To customize your own object detection algorithms, create a `perception_manager.py` under
`opencda/customize/core/sensing/perception/` folder. 

```python
import cv2
from opencda.core.sensing.perception.perception_manager import PerceptionManager
from opencda.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from opencda.core.sensing.perception.static_obstacle import TrafficLight

class CustomziedPeceptionManager(PerceptionManager):
     def __init__(self, vehicle, config_yaml, cav_world, data_dump=False):
        super(CustomizedLocalizationManager, self).__init__(vehicle, config_yaml, cav_world, data_dump)
     
     def detect(self, ego_pos):
        objects = {'vehicles': [],
                   'traffic_lights': [],
                   'other_objects_you_wanna_add' : []}


        # retrieve current rgb images from all cameras
        rgb_images = []
        for rgb_camera in self.rgb_camera:
            while rgb_camera.image is None:
                continue
            rgb_images.append(cv2.cvtColor(np.array(rgb_camera.image),
                    cv2.COLOR_BGR2RGB))
        
        # retrieve lidar data from the sensor
        lidar_data = self.lidar.data
        
        ########################################
        # this is where you put your algorithm #
        ########################################
        objects = your_algorithm(rgb_images, lidar_data)
        assert type(objects['vehicles']) == ObstacleVehicle
        assert type(objects['traffic_lights']) == TrafficLight
         
     return objects

```
---
### Behavior Planning Customization
During simulation runtime, `BehaviorAgent` first saves the ego vehicle position, speed
and surrounding objects information from `PerceptionManager` and `LocalizationManager` through
function `update_information`. Afterwards,  the `BehaviorAgent` will call function
`run_step()` to execute a single step and return the `target_speed` and `target_location`.

To customize your own behavior planning algorithms, create a `behavior_agent.py` under
`opencda/customize/core/plan/` folder.

```python
import carla.libcarla
from opencda.core.plan.behavior_agent import BehaviorAgent


class CustomizedBehaviorAgent(BehaviorAgent):
    def __init__(self, vehicle, carla_map, config_yaml):
        ...

    def update_information(self, ego_pos, ego_speed, objects):
        ########################################
        # this is where you put your algorithm #
        ########################################
        do_some_preprocessing(ego_pos, ego_speed, objects)

    def run_step(self):
        ########################################
        # this is where you put your algorithm #
        ########################################
        target_speed, target_loc = your_plan_algorithm()
        assert type(target_speed) == float
        assert type(target_loc) == carla.Location
        
        return target_speed, target_loc
```
---
### Control Customization
Similar with `BehaviorAgent`, `ControlManager` first saves the ego vehicle and position
through `update_info()`, and then takes the `target_speed` and `target_loc` generated from
behavior agent to produce the final control command through `run_step()`.

Different from other modules, `ControlManager` is more like a abstract class, which provides an
interface to call the corresponding controller (default pid controller).

```python
import importlib

class ControlManager(object):
    def __init__(self, control_config):
        controller_type = control_config['type']
        controller = getattr(
            importlib.import_module(
                "opencda.core.actuation.%s" %
                controller_type), 'Controller')
        self.controller = controller(control_config['args'])

    def update_info(self, ego_pos, ego_speed):
        """
        Update ego vehicle information for controller.
        """
        self.controller.update_info(ego_pos, ego_speed)

    def run_step(self, target_speed, target_loc):
        """
        Execute current controller step.
        """
        control_command = self.controller.run_step(target_speed, waypoint)
        return control_command
```

Therefore, if you want to use a controller other than pid controller, you can just create your `customize_controller.py` under `opencda/core/acutation/` folder, and follow the same input and output data format:
```python
class CustomizeController:
    def update_info(self, ego_pos, ego_spd):
        ########################################
        # this is where you put your algorithm #
        ########################################
        do_some_process(ego_pos, ego_spd)
    
    def run_step(self, target_speed, target_loc):
        ########################################
        # this is where you put your algorithm #
        ########################################
        control_command = control(target_speed, target_loc)
        assert control_command == carla.libcarla.VehicleControl
        return control_command
```
Then put your controller's name in your yaml file:
```yaml
vehicle_base:
    controller:
        type: customize_controller # this has to be exactly the same name as the controller py file
        args: ......
```

