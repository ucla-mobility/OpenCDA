import carla
from collections import OrderedDict
from omegaconf import OmegaConf

import opencda.scenario_testing.utils.sim_api_replay as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.misc import TOWN_DICTIONARY
import os, time

from opencda.scenario_testing.logreplay \
    import clean_yaml, extract_vehicle_positions, structure_transform_cav, run_scenario

def run_scenario(opt, scenario_params, ScenarioManager=sim_api.ScenarioManagerReplayBEV):
    
    veh_data_dump = True
    # rsu_data_dump = True

    dataset_dir = scenario_params['root_dir']
    scene_name = scenario_params['current_time']
    base_dataset_flag = scenario_params['base_dataset_flag']
    cav_dir = sorted([d for d in os.listdir(os.path.join(dataset_dir, scene_name)) 
                        if os.path.isdir(os.path.join(dataset_dir,scene_name,d))])[0]
    timestamps = set(f[:6] for f in os.listdir(os.path.join(dataset_dir,scene_name,cav_dir)))
    timestamps = sorted(list(timestamps))

    # create CAV world
    cav_world = CavWorld(opt.apply_ml)

    # create scenario manager
    scenario_manager = ScenarioManager(scenario_params, opt.apply_ml, opt.version,
                                                town=TOWN_DICTIONARY[scene_name], cav_world=cav_world)
    
    single_cav_dict = bg_veh_dict = None

    try:

        if opt.record:
            scenario_manager.client. \
                start_recorder(f'{scene_name}.log', True)
        
        bg_veh_bp = None
        
        if base_dataset_flag:
            cav_pos, bg_veh_pos, bg_veh_bp = extract_vehicle_positions(dataset_dir, scene_name, timestamps[0], True)
        else:
            cav_pos, bg_veh_pos = extract_vehicle_positions(dataset_dir, scene_name, timestamps[0])

        veh_data_dump = True
        
        single_cav_dict = scenario_manager.create_vehicle_manager(cav_pos, timestamps[0], application=['single'], data_dump=veh_data_dump)

        bg_veh_dict = scenario_manager.create_traffic_carla(bg_veh_pos, timestamps[0], bg_veh_bp)

        if veh_data_dump:
            for cav_id, cav_content in single_cav_dict.items():
                cav_content['actor'].data_dumper.count = int(timestamps[0])

        spectator = scenario_manager.world.get_spectator()
        
        # run steps

        for t in timestamps:
            print(t)

            scenario_manager.tick()
            anchor_cav = single_cav_dict[list(single_cav_dict.keys())[0]]['actor']
            transform = anchor_cav.vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(transform.location + carla.Location(z=50),
                carla.Rotation(pitch=-90)))
            
            # move vehicles and rsus, adjust data_dumper count

            cav_pos, bg_veh_pos = extract_vehicle_positions(dataset_dir, scene_name, t)
            
            for cav_id, cav_content in single_cav_dict.items():
                if veh_data_dump:
                    cav_content['actor'].data_dumper.count = int(t)
                
                scenario_manager.move_vehicle(cav_content, t, 
                                              structure_transform_cav(cav_pos[cav_id]))
            
            for bg_veh_id, bg_veh_content in bg_veh_dict.items():
                if bg_veh_id in bg_veh_pos:
                    scenario_manager.move_vehicle(bg_veh_content, t, 
                                                structure_transform_cav(bg_veh_pos[bg_veh_id]))
    
            # run_step update traffic and dump data
            
            for cav_id, cav_content in single_cav_dict.items():
                single_cav = cav_content['actor']
                single_cav.update_info()
                single_cav.run_step()

    except KeyboardInterrupt:
        pass

    finally:

        time.sleep(8)

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        if single_cav_dict is not None:
            for cav_id, cav_content in single_cav_dict.items():
                cav_content['actor'].destroy()
        if bg_veh_dict is not None:
            for veh_content in bg_veh_dict.values():
                veh_content['actor'].destroy()
