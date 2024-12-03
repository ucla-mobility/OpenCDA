from opencda.core.common.data_dumper_replay import DataDumperReplay, DataDumperReplayPly
from opencda.core.common.rsu_manager import RSUManager
from opencda.core.sensing.perception.perception_manager_replay import PerceptionManagerReplay
from opencda.core.sensing.localization.rsu_localization_manager import LocalizationManager
import os

class RSUManagerReplay(RSUManager):

    def __init__(
            self,
            carla_world,
            config_yaml,
            carla_map,
            cav_world,
            current_time='',
            data_dumping=None,
            PerceptionManager=PerceptionManagerReplay,
            DataDumper=DataDumperReplay):

        self.rid = config_yaml['id']
        # The id of rsu is always a negative int
        if self.rid > 0:
            self.rid = -self.rid

        # read map from the world everytime is time-consuming, so we need
        # explicitly extract here
        self.carla_map = carla_map

        # retrieve the configure for different modules
        # todo: add v2x module to rsu later
        sensing_config = config_yaml['sensing']
        sensing_config['localization']['global_position'] = \
            config_yaml['spawn_position']
        sensing_config['perception']['global_position'] = \
            config_yaml['spawn_position']

        # localization module
        self.localizer = LocalizationManager(carla_world,
                                             sensing_config['localization'],
                                             self.carla_map)
        # perception module
        self.perception_manager = PerceptionManager(vehicle=None,
                                                    config_yaml=sensing_config['perception'],
                                                    cav_world=cav_world,
                                                    carla_world=carla_world,
                                                    data_dump=data_dumping,
                                                    infra_id=self.rid)
        if data_dumping:
            save_path = os.path.join(data_dumping, current_time, str(self.rid))
            self.data_dumper = DataDumper(self.perception_manager, save_path)
        else:
            self.data_dumper = None

        cav_world.update_rsu_manager(self)

class RSUManagerReplayPly(RSUManagerReplay):

    def __init__(
            self,
            carla_world,
            config_yaml,
            carla_map,
            cav_world,
            current_time='',
            data_dumping=None):
        
        super(RSUManagerReplayPly ,self).__init__(carla_world,
            config_yaml,
            carla_map,
            cav_world,
            current_time,
            data_dumping,
            PerceptionManager=PerceptionManagerReplay,
            DataDumper=DataDumperReplayPly)
        
    def update_info(self):
        pass

    def run_step(self):

        if self.data_dumper:
            self.data_dumper.run_step()