import carla
from collections import OrderedDict
from omegaconf import OmegaConf

import opencda.scenario_testing.utils.sim_api_replay as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.misc import TOWN_DICTIONARY
import os, time

def clean_yaml(path):

    '''
    remove error/ warning messages
    '''

    with open(path, 'r') as r:

        yaml_r = r.readlines()

        with open(path, 'w') as w:

            del_flag = False

            for textline in yaml_r:

                if '!!python/' not in textline:
                    if del_flag:
                        try:
                            # cast = int(textline[4:])
                            del_flag = False
                        except ValueError:
                            pass
                    else:
                        w.write(textline)
                else:
                    del_flag = True

def extract_vehicle_positions(dataset_dir, scene_name, timestamp, get_bg_veh_bp=False):

    cav_pos = OrderedDict()
    bg_veh_pos = OrderedDict()
    
    bg_veh_bp = OrderedDict() if get_bg_veh_bp else None
    cav_ids = os.listdir(os.path.join(dataset_dir, scene_name))
    cav_ids = sorted([c for c in cav_ids if c.isalnum() and int(c) > 0]) # exclude RSUs with -ve ids
    cav0 = cav_ids[0]

    for cav_id in cav_ids:
        if not os.path.isdir(os.path.join(dataset_dir, scene_name, cav_id)):
            continue
        
        yaml_path = os.path.join(dataset_dir,scene_name,cav_id,timestamp+'.yaml')
        clean_yaml(yaml_path)
        cav_yaml = OmegaConf.load(yaml_path)
        cav0 = cav_ids[0] # remark

        if cav_id == cav0: #in cav_ids:
            cav_pos[cav_id] = cav_yaml['true_ego_pos']
        else:
            bg_veh_pos[int(cav_id)] = cav_yaml['true_ego_pos']

        for bg_veh_id, bg_veh_content in cav_yaml['vehicles'].items():
            if str(bg_veh_id) == cav0 or int(bg_veh_id) < 0:
                continue

            bg_veh_pos[bg_veh_id] = bg_veh_content['location']+bg_veh_content['angle']

            if bg_veh_bp is not None:
                bg_veh_bp[bg_veh_id] = {'bp_id': bg_veh_content['bp_id'], 'color': bg_veh_content['color']}
    
    cav_pos = {cav0: cav_pos[cav0]} # only choose ego vehicle
    
    if get_bg_veh_bp:
        return cav_pos, bg_veh_pos, bg_veh_bp
    else:
        return cav_pos, bg_veh_pos

def structure_transform_cav(pose):
        """
        Convert the pose saved in list to transform format.

        Parameters
        ----------
        pose : list
            x, y, z, roll, yaw, pitch

        Returns
        -------
        carla.Transform
        """
        cur_pose = carla.Transform(carla.Location(x=pose[0],
                                                  y=pose[1],
                                                  z=pose[2]),
                                   carla.Rotation(roll=pose[3],
                                                  yaw=pose[4],
                                                  pitch=pose[5]))

        return cur_pose

def run_scenario(opt, scenario_params, ScenarioManager=sim_api.ScenarioManagerReplay):
    
    veh_data_dump = True
    rsu_data_dump = True

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
    
    single_cav_dict = rsu_list = bg_veh_dict = None

    try:

        if opt.record:
            scenario_manager.client. \
                start_recorder(f'{scene_name}.log', True)
        
        # set up vehicles and rsus
        bg_veh_bp = None
        
        if base_dataset_flag:
            # remark: after replaying original to generate base, veh model and color is available
            cav_pos, bg_veh_pos, bg_veh_bp = extract_vehicle_positions(dataset_dir, scene_name, timestamps[0], True)
        else:
            # original OPV2V dataset did not record background cav details e.g. bp_id, exact replication is not possible
            cav_pos, bg_veh_pos = extract_vehicle_positions(dataset_dir, scene_name, timestamps[0])

        # initialize cav actors
        single_cav_dict = scenario_manager.create_vehicle_manager(cav_pos, timestamps[0], application=['single'], data_dump=veh_data_dump)
        bg_veh_dict = scenario_manager.create_traffic_carla(bg_veh_pos, timestamps[0], bg_veh_bp)
        rsu_list = scenario_manager.create_rsu_manager(data_dump=rsu_data_dump)

        spectator = scenario_manager.world.get_spectator()
        
        # run steps

        for t in timestamps:
            print(t)

            scenario_manager.tick()
            anchor_cav = single_cav_dict[list(single_cav_dict.keys())[0]]['actor']
            transform = anchor_cav.vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(transform.location + carla.Location(z=40),
                carla.Rotation(pitch=-90)))
            
            # move vehicles and rsus, adjust data_dumper count
            # remark: data_dumper normally starts from t=0 and increment by 1 each time
            #           however, recording data starts after t=60 and differs by 2 
            #           according to recording original OPV2V's settings

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
    
            for rsu in rsu_list:
                if rsu_data_dump:
                    rsu.data_dumper.count = int(t)
            
            # run_step update traffic and dump data
            
            for cav_id, cav_content in single_cav_dict.items():
                single_cav = cav_content['actor']
                single_cav.update_info()
                single_cav.run_step()

            for rsu in rsu_list:
                rsu.update_info()
                rsu.run_step()
            
            # break ### debug

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
        if rsu_list is not None:
            for r in rsu_list:
                r.destroy()
        if bg_veh_dict is not None:
            for veh_content in bg_veh_dict.values():
                veh_content['actor'].destroy()
