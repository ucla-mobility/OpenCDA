import cv2
from opencda.core.sensing.perception.perception_manager import PerceptionManager
from opencda.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from opencda.core.sensing.perception.static_obstacle import TrafficLight
from opencda.core.sensing.localization.localization_manager import CustomizedLocalizationManager

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
        objects = abs(rgb_images, lidar_data)
        assert type(objects['vehicles']) == ObstacleVehicle
        assert type(objects['traffic_lights']) == TrafficLight

      # return objects