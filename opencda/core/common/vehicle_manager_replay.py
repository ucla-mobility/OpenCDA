import uuid
import os

from opencda.core.sensing.localization.localization_manager \
    import LocalizationManager
from opencda.core.sensing.perception.perception_manager_replay \
    import PerceptionManagerReplay, PerceptionManagerReplayBEV
from opencda.core.common.data_dumper_replay \
    import DataDumperReplay
from opencda.core.map.map_manager import MapManager

class VehicleManagerReplay(object):
    
    def __init__(
            self,
            vehicle,
            config_yaml,
            application,
            carla_map,
            cav_world,
            current_time='',
            data_dumping=None,
            PerceptionManager=PerceptionManagerReplay,
            DataDumper=DataDumperReplay):

        # an unique uuid for this vehicle
        self.vid = str(uuid.uuid1())
        self.vehicle = vehicle
        self.carla_map = carla_map

        # retrieve the configure for different modules
        sensing_config = config_yaml['sensing']
        map_config = config_yaml['map_manager']

        # localization module
        self.localizer = LocalizationManager(
            vehicle, sensing_config['localization'], carla_map)
        # perception module
        self.perception_manager = PerceptionManager(
            vehicle, sensing_config['perception'], cav_world,
            data_dumping)
        
        # map manager
        self.map_manager = MapManager(vehicle,
                                      carla_map,
                                      map_config)
        
        # behavior agent
        self.agent = None
        
        if data_dumping:
            save_path = os.path.join(data_dumping, current_time, str(vehicle.id))
            self.data_dumper = DataDumper(self.perception_manager, save_path)
        else:
            self.data_dumper = None

        cav_world.update_vehicle_manager(self)

    def update_info(self):
        """
        Call perception and localization module to
        retrieve surrounding info an ego position.
        """
        # localization
        self.localizer.localize()

        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()

        # object detection
        objects = self.perception_manager.detect(ego_pos)

        # update the ego pose for map manager
        self.map_manager.update_information(ego_pos)

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation.
        """

        # dump data
        if self.data_dumper:
            self.data_dumper.run_step(self.perception_manager,
                                      self.localizer,
                                      self.agent)

    def destroy(self):
        """
        Destroy the actor vehicle
        """
        self.perception_manager.destroy()
        self.localizer.destroy()
        self.vehicle.destroy()
        self.map_manager.destroy()

class VehicleManagerReplayBEV(VehicleManagerReplay):
    
    def __init__(
            self,
            vehicle,
            config_yaml,
            application,
            carla_map,
            cav_world,
            current_time='',
            data_dumping=None):

        super(VehicleManagerReplayBEV, self).__init__(vehicle, config_yaml, application, carla_map, cav_world,
                                                      current_time,data_dumping,PerceptionManagerReplayBEV,
                                                      DataDumperReplay)