import os 
import glob 
import shutil

def create_rsu_list(rsu_locations):
    """
        Filter and format original RSU json data

        Parameters
        ----------
        rsu_locations : dict { int: tuple(x,y,z,p,y,r) }

        Returns
        -------
        Formatted RSU locations: list [ dict{...} ]
    """

    rsu_list = []
    index = 1

    for key, l in rsu_locations.items():
        
        if key.isnumeric():
            rsu_list.append({
                    'name': f'rsu{index}',
                    'spawn_position': l,
                    'id': -index
                }
            )
            index += 1
        
        # elif key == 'center':
        #     rsu_list.append({
        #             'name': f'rsu0',
        #             'spawn_position': l,
        #             'id': 0
        #         }
        #     )
    else:
        if index == 1:
            raise RuntimeError(f'rsu_locations is empty: {rsu_locations}')
    
    return rsu_list

def update_rsus(rsu_configs, pitch):
    """
        Modify pitch configuration of each RSU location

        Parameters
        ----------
        rsu_configs : dict # i.e. rsu.json

        Returns
        -------
        Updated RSU location dict
    """

    for i, loc in rsu_configs['locations'].items():
        for j, coor in loc.items():

            if not isinstance(coor, list):
                continue
            
            coor[3] = pitch
            rsu_configs['locations'][i][j] = coor

def extract_timestamps(cav_dir, file_types=['.yaml','.pcd']):

    """
        Count timestamps of each given directory and return smallest
        if complete episode recorded, all counts should be equal to that of base data

        Parameters
        ----------
        cav_dir : str # ./test_set/2021_08_22_21_41_24/123

        Returns
        -------
        int
    """
    
    def get_timestamps(target_dir, ext):

        file_list = [f for f in os.listdir(target_dir) if f.endswith(t)]
        timestamps = []

        for file in file_list:
            res = file.split('/')[-1]
            timestamp = res.replace(ext, '')
            timestamps.append(timestamp)

        return timestamps
        
    num_timestamps = []
    for t in file_types:
        # files = [f for f in os.listdir(cav_dir) if f.endswith(t)]
        ts = get_timestamps(cav_dir, t)
        num_timestamps.append(ts)
    
    return min(num_timestamps)

def check_missing_cavs(root_dir, output_dir):
    """
        Carla simulator occassionally crashes and exits with incomplete episodes
        this function helps to detect missing frames/ timestamps

        Iterate through scenes and each cav dirs to check for incomplete recordings

        Parameters
        ----------
        root_dir : str
        output_dir : str 

        Returns
        -------
        list # scene names with inncomplete recordings
    """

    missing_scenes = []
    
    for scene in sorted(os.listdir(root_dir)):

        output_dir_scene = os.path.join(output_dir, scene)

        # case 1: scene recording had not even started yet
        if not os.path.exists(output_dir_scene): 
            missing_scenes.append(scene)
            continue

        
        output_cav = [d for d in sorted(os.listdir(output_dir_scene)) if os.path.isdir(os.path.join(output_dir_scene, d))]
        # if os.path.isdir(...) to filter 'data_protocol.yaml' file
        # case 2: empty scene directory with 0 cavs
        if len(output_cav) == 0:
            missing_scenes.append(scene)
            continue

        root_dir_scene = os.path.join(root_dir, scene)
        root_cav = [d for d in sorted(os.listdir(root_dir_scene)) if os.path.isdir(os.path.join(root_dir_scene, d))][0]
        ts_root = extract_timestamps(os.path.join(root_dir_scene, root_cav))


        for cav in output_cav:
            output_dir_scene_cav = os.path.join(output_dir_scene, cav)
            if not os.path.isdir(output_dir_scene_cav): # skip data_protocol.yaml
                continue

            ts = extract_timestamps(output_dir_scene_cav)
            # ts = extract_timestamps_ply(os.path.join(livox_dir, scene, cav)) ###

            if set(ts_root) != set(ts):
                missing_scenes.append(scene)
                break

    return missing_scenes
