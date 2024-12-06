""" 
enables exact replay of scenario recordings, as detailed as each vehicle's model and color, and rsu placement
"""

import argparse
import importlib
import os, shutil
import sys
from omegaconf import OmegaConf
import json
from utils.check_missing import create_rsu_list, update_rsus, check_missing_cavs
from opencda.version import __version__
from opencda.core.common.misc import TOWN_DICTIONARY

def arg_parse():

    parser = argparse.ArgumentParser(description="OpenCDA scenario runner.")

    parser.add_argument('-t', "--test_scenario", required=True, type=str,
                        help='Define the name of the scenario you want to test. The given name must'
                             'match one of the testing scripts(e.g. single_2lanefree_carla) in '
                             'opencda/scenario_testing/ folder'
                             ' as well as the corresponding yaml file in opencda/scenario_testing/config_yaml.')
    parser.add_argument("--record", action='store_true',
                        help='whether to record and save the simulation process to .log file')
    parser.add_argument("--apply_ml",
                        action='store_true',
                        help='whether ml/dl framework such as sklearn/pytorch is needed in the testing. '
                             'Set it to true only when you have installed the pytorch/sklearn package.')
    parser.add_argument('-v', "--version", type=str, default='0.9.12',
                        help='Specify the CARLA simulator version, default'
                             'is 0.9.11, 0.9.12 is also supported.')
    
    parser.add_argument('-r', "--root_dir", type=str,
                        help='base data to replay on')
    parser.add_argument('-o', "--output_dir", type=str)
    parser.add_argument('-b', "--base_dataset", action='store_true',
                        help='whether replaying on original OPV2V dataset')
    
    opt = parser.parse_args()
    return opt

def main():
    
    opt = arg_parse()

    print("OpenCDA Version: %s" % __version__)
    
    default_yaml = config_yaml = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        'opencda/scenario_testing/config_yaml/default.yaml')
    config_yaml = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'opencda/scenario_testing/config_yaml/%s.yaml' % opt.test_scenario)
    
    default_dict = OmegaConf.load(default_yaml)
    config_dict = OmegaConf.load(config_yaml)

    print('root_dir', opt.root_dir)
    print('output_dir', opt.output_dir)

    default_dict['root_dir'] = opt.root_dir # '/media/hdd1/opv2v/' + opt.root_dir + '/'
    default_dict['output_dir'] = opt.output_dir # '/media/hdd1/opv2v/opencda_dump_test/' + opt.output_dir + '/'
    default_dict['base_dataset_flag'] = opt.base_dataset
    
    target = check_missing_cavs(default_dict['root_dir'], default_dict['output_dir'])

    root_dir = default_dict['root_dir']
    dataset_dir = root_dir

    with open(default_dict['rsu_loc_dir'], 'r') as j:
        rsu_configs = json.load(j)

    '''
    automate:
    - desired adjustment config passed as arguments
    - iteratively update corresponding carla sensor config parameters
    '''
    pitch = int(opt.output_dir.split('_')[-1])
    update_rsus(rsu_configs, pitch)
    target = [t for t in target if t in rsu_configs['scenes']]

    # future extension: allow dynamic input for lidar type
    # lidar_configs = {
    #     'type': 'livox',
    #     'height': 6,
    #     'pitch': pitch
    # }

    # current simulation chooses 'Town05' map only
    target_towns = ['Town05']
    scenario_folders = sorted([os.path.join(dataset_dir, x) 
                                   for x in os.listdir(dataset_dir) if
                                   os.path.isdir(os.path.join(dataset_dir, x)) and TOWN_DICTIONARY[x] in target_towns])

    for (i, scenario_folder) in enumerate(scenario_folders):

        scene_name = os.path.split(scenario_folder)[-1]
        
        if scene_name not in target: # filter scenes that belong to target_towns
            print(f'{scene_name} not in target')
            continue
        
        scenario_folder_root = scenario_folder.replace(dataset_dir, root_dir)
        protocol_yml = [x for x in os.listdir(scenario_folder_root) if x.endswith('.yaml')]
        scenario_folder_root_yaml = os.path.join(scenario_folder_root,protocol_yml[0])
        scene_dict = OmegaConf.load(scenario_folder_root_yaml)
        
        scene_dict = OmegaConf.merge(default_dict, scene_dict)
        # config_dict has highest priority, hence last merge
        scene_dict = OmegaConf.merge(scene_dict, config_dict) 
        
        scenario_folder_output = os.path.join(default_dict['output_dir'], scene_name)
        
        if not os.path.exists(scenario_folder_output):
            # 'data_protocol.yaml' is needed for each replay recording
            os.makedirs(scenario_folder_output)
            yaml_file = os.path.join(scenario_folder_output, 'data_protocol.yaml')
            if not os.path.exists(yaml_file):
                shutil.copy(scenario_folder_root_yaml, yaml_file)
        else:
            cav_dirs = [d for d in os.listdir(scenario_folder_output) 
                        if os.path.isdir(os.path.join(scenario_folder_output, d)) and int(d) > 0]
            
            if 'metric' in opt.test_scenario:
                # logreplay_metric only records rsu lidars, thus unrelated to vehicle recordings 
                pass
            else:
                print('existing vehicle cav dirs from previous simulation')
                del_flag = input('delete files (y/n)').strip().lower()
                assert del_flag in ['y', 'n']
                if del_flag == 'y':
                    for d in cav_dirs:
                        print('removed', d)
                        shutil.rmtree(os.path.join(scenario_folder_output, d))
        
        # if scene_name not in rsu_configs['scenes']: # debug
        #     print(f'{scene_name} not in rsu_configs[scenes]')
        #     continue
        
        rsu_locations = rsu_configs['locations'][str(rsu_configs['scenes'][scene_name])]
        rsu_list = create_rsu_list(rsu_locations)

        # if len(rsu_list) < 4: # debug
        #     print(f'{scene_name} len(rsu_list)={len(rsu_list)} < 4')
        #     continue

        print('scene_name', scene_name)

        scene_dict = OmegaConf.merge(scene_dict, {'current_time': scene_name, 
                                                  'scenario': {'rsu_list': rsu_list}})
        
        # import the testing script
        testing_scenario = importlib.import_module("opencda.scenario_testing.%s" % opt.test_scenario)

        # check if the yaml file for the specific testing scenario exists
        if not os.path.isfile(config_yaml):
            sys.exit("opencda/scenario_testing/config_yaml/%s.yaml not found!" % opt.test_cenario)

        # get the function for running the scenario from the testing script
        scenario_runner = getattr(testing_scenario, 'run_scenario')

        # run the scenario testing
        scenario_runner(opt, scene_dict)

        break ###

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')