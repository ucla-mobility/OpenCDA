## Algorithm Customization

Due the high modularity of OpenCDA, you can conveniently replace any default module with your own
algorithms. The best way for customization is to  <strong> put your customized module under `opencda/customize/...` , </strong> and 
<strong>use inheritance to overwrite the default algorithms</strong>. Afterwards, import your customized module in
`VehicleManager` class. The only thing you need to pay attention is to make the input and output format the same
as origin module if you only want to change a single module. Below we will show a detailed instruction 
of customizations for localization and perception module as an example.

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
If a user wants to remain the whole structure of localization and just replace the filter(e.g. Extended Kalman Filter),
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
### Perception Manager
The class `PerceptionManager` is responsible for object detection. The core function `detect(ego_pos)` takes the
ego position from localization module as the input, and return a dictionary `objects` whose keys are the object
categories and the values are each object's attributes(e.g. 3d poses, static or dynamic) under world coordinate system in this category.

To customize your own object detection algorithms, create a `perception_manager.py` under
`opencda/customize/core/sensing/perception/` folder. 
```python
import cv2
from opencda.core.sensing.perception.perception_manager import PerceptionManager
from opencda.core.sensing.perception.obstacle_vehicle import ObstacleVehicle

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
        assert type(objects['vehicles'])
        
     return objects

```