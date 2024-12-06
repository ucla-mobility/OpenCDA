import shutil, random, math, os, json, argparse
import numpy as np

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        pass
    try:
        import unicodedata
        unicodedata.numeric(s)
        return True
    except (TypeError, ValueError):
        pass
    return False
def read_pcd(file):
    # file_split = os.path.splitext(file)
    pcd = []
    obj_tag = []
    with open(file, 'r') as r:
        for line in r.readlines():
            l = line.split(' ')

            if not is_number(l[0]):
                continue
        
            pcd.append(list(map(float, l[:3])))
            obj_tag.append(int(l[5].strip()))
    return np.array(pcd), np.array(obj_tag)
import torch
import torch.nn.functional as F
def project_points_by_matrix_torch(points, transformation_matrix):
    # convert to homogeneous  coordinates via padding 1 at the last dimension.
    # (N, 4)
    points_homogeneous = F.pad(points, (0, 1), mode="constant", value=1)
    # (N, 4)
    projected_points = torch.einsum("ik, jk->ij", points_homogeneous,
                                    transformation_matrix)
    return projected_points[:, :3]

def pcd_coordinate_convertor(file, trans_mat):
    file_split = os.path.splitext(file)
    with open(file, 'r') as r:
        # with open(file+'.new', 'w') as w:
        with open(file_split[0]+'_conv'+file_split[1], 'w') as w:
            for l in r.readlines():
                # print(l.split(' ')[0])
                if is_number(l.split(' ')[0]):
                    #print(l)

                    # loc = np.matrix([float(l[0]), float(l[1]), float(l[2]), 1.0])
                    x = float(l.split(' ')[0])
                    y = float(l.split(' ')[1])
                    z = float(l.split(' ')[2])
                    loc = np.matrix([x, y, z, 1.0])

                    loc_target = np.dot(trans_mat, loc.T)
                    loc_target = loc_target.tolist()
                    # print(loc_target)

                    l_new_list = [0, 0, 0, 0, 0, 0]

                    l_new_list[0] = str(loc_target[0][0])
                    l_new_list[1] = str(loc_target[1][0])
                    l_new_list[2] = str(loc_target[2][0])
                    l_new_list[3] = l.split(' ')[3]
                    l_new_list[4] = l.split(' ')[4]
                    l_new_list[5] = l.split(' ')[5]
                    l_new = ' '.join(l_new_list)
                    w.write(l_new)

                else:
                    w.write(l)
    print('converted', file_split[0]+'_conv'+file_split[1])
                # if 'nan' not in l:
                #     w.write(l)
    # shutil.move(file+'.new', file)
random.seed(123)
def computeNUC(file, d, disk_radius, p, target_field):
    '''
    file -- semantic ply file path
    d -- the number of disk
    disk_radius -- the radius of disk
    p -- disk area percentage
    '''
    point_list = semantic_filter(file, target_field)
    total_point = len(point_list) # total number of point
    random_d_index = random.sample(range(total_point), d) # choose d indexes from total_point randomly, d must be smaller than total_point
    total_nums_in_disk = 0
    viz(np.array(point_list)) ###

    tmp_val = [] # n_i / (N * p)
    for index in random_d_index:
        nums_in_index = nums_in_disk(point_list, index, disk_radius) # compute the point number in index_th disk
        total_nums_in_disk = total_nums_in_disk + nums_in_index
        tmp_val.append(nums_in_index / (total_point *p))

    avg = total_nums_in_disk / (d * total_point * p)

    nuc = 0
    for val in tmp_val:
        nuc = nuc + math.pow(val - avg, 2)
    nuc = math.sqrt(nuc / d)

    return total_point, nuc

def nums_in_disk(point_list, point_index, disk_radius):
    disk_center_point = point_list[point_index]
    d_x = disk_center_point[0]
    d_y = disk_center_point[1]
    # d_z = disk_center_point[2]
    point_nums = 0
    for point in point_list:
        p_x = point[0]
        p_y = point[1]
        # p_z = point[2]
        dist = math.sqrt(math.pow(d_x - p_x, 2) + math.pow(d_y - p_y, 2))
        if dist <= disk_radius:
            point_nums = point_nums + 1
    return point_nums

def nums_in_disk_mat(point_list, point_index, disk_radius):
    point_list = np.array(point_list)
    disk_center_point = point_list[point_index]
    
    dist = np.linalg.norm(point_list[:,:2]-disk_center_point[:2], axis=1)
    point_nums = (dist <= disk_radius).sum()
    return point_nums

def semantic_filter_mat(pcd, obj_tag):
    target_tags = np.array([7, 8]) # ['road', 'sidewalk']
    semantic_mask = np.in1d(obj_tag, target_tags)
    return pcd[semantic_mask]

def area_filter(pcd, target_field):
    lidar_maks = (target_field[0] < pcd[:,0]) & (pcd[:,0] < target_field[1]) & (target_field[2] < pcd[:,1]) & (pcd[:,1] < target_field[3])
    return pcd[lidar_maks]

def semantic_filter(file, target_field):
    target_tag_1 = 7 # 7 means Road
    target_tag_2 = 8 # 8 means SideWalk
    filtered_list = []
    
    file_split = os.path.splitext(file)
    file = file_split[0]+'_conv'+file_split[1]

    with open(file, 'r') as r:
        with open(file+'.new', 'w') as w:
            for l in r.readlines():
                # print(l.split(' ')[0])
                
                if is_number(l.split(' ')[0]):
                    if int(l.split(' ')[5]) != target_tag_1 and int(l.split(' ')[5]) != target_tag_2:
                        continue
                    l_x = float(l.split(' ')[0])
                    l_y = float(l.split(' ')[1])
                    if (l_x < target_field[0]) or (l_x > target_field[1]) or (l_y < target_field[2]) or (l_y > target_field[3]):
                        continue
                    l_new_list = [l.split(' ')[0],
                                    l.split(' ')[1],
                                    l.split(' ')[2],
                                    l.split(' ')[3],
                                    l.split(' ')[4],
                                    l.split(' ')[5]]
                    # l_new_list[2] = l.split(' ')[2]
                    # l_new_list[3] = l.split(' ')[3]
                    # l_new_list[4] = l.split(' ')[4]
                    # l_new_list[5] = l.split(' ')[5]

                    pos_list = [float(l.split(' ')[0]), float(l.split(' ')[1]), float(l.split(' ')[2])] # save (x, y, z)
                    filtered_list.append(pos_list)

                    l_new = ' '.join(l_new_list)
                    w.write(l_new)

                else:
                    w.write(l)
                
    with open(file+'.new', 'r') as rd:
        with open(file + '.filtered', 'w') as wt:
            for i, line in enumerate(rd.readlines()):
                if i == 2:
                    line_new = []
                    line_new.append(line.split(' ')[0])
                    line_new.append(line.split(' ')[1])
                    line_new.append(str(len(filtered_list)))
                    write_str = ' '.join(line_new) + '\n'
                    wt.write(write_str)
                else:
                    wt.write(line)
    os.remove(file+'.new')
    new_name = file[:-4] + '_filtered.ply'  
    shutil.move(file + '.filtered', new_name)
    print('filtered', new_name)
    return filtered_list

# def computeNUC_mat(pcd, obj_tag, d, disk_radius, p, target_field):
def computeNUC_mat(point_list, d, disk_radius, p):
    '''
    file -- semantic ply file path
    d -- the number of disk
    disk_radius -- the radius of disk
    p -- disk area percentage
    '''
    # point_list = semantic_filter_mat(pcd, obj_tag, target_field)
    total_point = len(point_list) # total number of point
    random_d_index = random.sample(range(total_point), d) # choose d indexes from total_point randomly, d must be smaller than total_point
    total_nums_in_disk = 0

    tmp_val = [] # n_i / (N * p)
    for index in random_d_index:
        nums_in_index = nums_in_disk_mat(point_list, index, disk_radius) # compute the point number in index_th disk
        total_nums_in_disk = total_nums_in_disk + nums_in_index
        tmp_val.append(nums_in_index)

    avg = total_nums_in_disk / (d * total_point * p)

    tmp_val = np.array(tmp_val) / (total_point *p)
    nuc = math.sqrt(((tmp_val - avg)**2).sum() / d)

    return total_point, nuc

# from opencood.utils.transformation_utils import x1_to_x2
def x_to_world(pose):

    x, y, z, roll, yaw, pitch = pose[:]

    # used for rotation matrix
    c_y = np.cos(np.radians(yaw))
    s_y = np.sin(np.radians(yaw))
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))

    matrix = np.identity(4)
    # translation matrix
    matrix[0, 3] = x
    matrix[1, 3] = y
    matrix[2, 3] = z

    # rotation matrix
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r

    return matrix
def x1_to_x2(x1, x2):

    x1_to_world = x_to_world(x1)
    x2_to_world = x_to_world(x2)
    world_to_x2 = np.linalg.inv(x2_to_world)

    transformation_matrix = np.dot(world_to_x2, x1_to_world)
    return transformation_matrix

BOUND = 10
rsu_loc_dir = '/media/hdd1/OpenCOOD/logreplay/scenario/rsu.json'
with open(rsu_loc_dir, 'r') as j:
    rsu_configs = json.load(j)
lidar_range = {}
def get_range(scenario_folder):
    # if '/' in scenario_folder:
    #     scenario_folder = os.path.basename(scenario_folder)
    # if scenario_folder in lidar_range:
    #     return lidar_range[scenario_folder]
    coors = rsu_configs['locations'][str(rsu_configs['scenes'][scenario_folder])]
    coor_x = [c[0] for c in coors.values()]
    coor_y = [c[1] for c in coors.values()]
    # GT_RANGE = [-140, -40, -3, 140, 40, 1]
    r = (min(coor_x)-BOUND, min(coor_y)-BOUND, -3, max(coor_x)+BOUND, max(coor_y)+BOUND, 1)
    # lidar_range[scenario_folder] = r
    # return lidar_range[scenario_folder]
    return r
def get_target_field(r):
    return r[0], r[3], r[1], r[4]
def get_center(r):
    return [(r[0]+r[3])/2, (r[1]+r[4])/2] + [0]*4
def get_lidar_pose(scenario_folder, id):
    if int(id) < 0:
        id = str(-int(id)-1)
    coors = rsu_configs['locations'][str(rsu_configs['scenes'][scenario_folder])]
    return coors[id]

import open3d as o3d
colors = {
            0: [0, 0, 0],  # None
            1: [70, 70, 70],  # Buildings
            2: [190, 153, 153],  # Fences
            3: [72, 0, 90],  # Other
            4: [220, 20, 60],  # Pedestrians
            5: [153, 153, 153],  # Poles
            6: [157, 234, 50],  # RoadLines
            7: [128, 64, 128],  # Roads
            8: [244, 35, 232],  # Sidewalks
            9: [107, 142, 35],  # Vegetation
            10: [0, 0, 255],  # Vehicles
            11: [102, 102, 156],  # Walls
            12: [220, 220, 0],  # TrafficSigns
            13: [70, 130, 180],  # Sky
            14: [81, 0, 81],  # Ground
            15: [150, 100, 100],  # Bridge
            16: [230, 150, 140],  # RailTrack
            17: [180, 165, 180],  # All types of guard rails/crash barriers.
            18: [250, 170, 30],  # Traffic Light
            19: [110, 190, 160],  # Static
            20: [170, 120, 50],  # Dynamic
            21: [45, 60, 150],  # Water
            22: [145, 170, 100]  # Terrain
        }
from matplotlib import pyplot as plt
def viz(pcd, path=None):
    o3d_pcd = o3d.geometry.PointCloud()
    o3d_pcd.points = o3d.utility.Vector3dVector(pcd[:, :3])

    int_color = np.ones((pcd.shape[0], 3))
    int_color[:, 0] *= 0 #247 / 255
    int_color[:, 1] *= 0 #244 / 255
    int_color[:, 2] *= 1 # 237 / 255
    o3d_pcd.colors = o3d.utility.Vector3dVector(int_color)

    
    if path:
        # vis = o3d.visualization.Visualizer()
        # vis.create_window(visible = False)
        # vis.add_geometry(o3d_pcd)
        # img = vis.capture_screen_float_buffer(True)
        # plt.imsave(path, img) #plt.imshow(np.asarray(img))

        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False) #works for me with False, on some systems needs to be true
        vis.add_geometry(o3d_pcd) # vis.update_geometry(your_mesh)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(path)
        vis.destroy_window()
        
        print('saved', path)
    else:
        o3d.visualization.draw_geometries([o3d_pcd])

# def viz(pcd, save_path='', mode='constant'):
#     """
#     Visualize the prediction, groundtruth with point cloud together.

#     Parameters
#     ----------
#     pred_tensor : torch.Tensor
#         (N, 8, 3) prediction.

#     gt_tensor : torch.Tensor
#         (N, 8, 3) groundtruth bbx

#     pcd : torch.Tensor
#         PointCloud, (N, 4).

#     show_vis : bool
#         Whether to show visualization.

#     save_path : str
#         Save the visualization results to given path.

#     mode : str
#         Color rendering mode.
#     """

#     def custom_draw_geometry(pcd):
#         vis = o3d.visualization.Visualizer()
#         vis.create_window()

#         opt = vis.get_render_option()
#         opt.background_color = np.asarray([0, 0, 0])
#         opt.point_size = 1.0

#         vis.add_geometry(pcd)

#         vis.run()
#         vis.destroy_window()

#     if len(pcd.shape) == 3:
#         pcd = pcd[0]
#     origin_lidar = pcd
#     # if not isinstance(pcd, np.ndarray):
#     #     origin_lidar = common_utils.torch_tensor_to_numpy(pcd)

#     # origin_lidar_intcolor = \
#     #     color_encoding(origin_lidar[:, -1] if mode == 'intensity'
#     #                    else origin_lidar[:, 2], mode=mode)
#     # left -> right hand
#     origin_lidar[:, :1] = -origin_lidar[:, :1]

#     o3d_pcd = o3d.geometry.PointCloud()
#     o3d_pcd.points = o3d.utility.Vector3dVector(origin_lidar[:, :3])
#     # o3d_pcd.colors = o3d.utility.Vector3dVector(origin_lidar_intcolor)

#     # if pred_tensor is None:
#     #     pred_tensor = torch.empty((0,8,3))
#     # oabbs_pred = bbx2oabb(pred_tensor, color=(1, 0, 0))
#     # oabbs_gt = bbx2oabb(gt_tensor, color=(0, 1, 0))

#     visualize_elements = [o3d_pcd]# + oabbs_pred + oabbs_gt
#     custom_draw_geometry(o3d_pcd)#, oabbs_pred, oabbs_gt)
#     # if save_path:
#     #     save_o3d_visualization(visualize_elements, save_path)

def eval(scene_dir, scene_name, pitch):
    print(scene_dir.split('/')[-2], scene_name)

    # scene_dir = f'/media/hdd1/opv2v/opencda_dump/test_livox_6-20_metric2/'
    # scene_name = '2021_08_23_12_58_19'
    root_dir = f'{scene_dir}/{scene_name}/'
    ply_file = '/'+[f for f in sorted(os.listdir(root_dir+'-1')) if f.endswith('.ply')][0]
    pitch = int(pitch)

    lidar_range = get_range(scene_name)
    target_field = get_target_field(lidar_range) #[92, 112, 80, 110]
    center = get_center(lidar_range)

    # ---------test------------
    index = 1
    # file = './7_placement.ply'
    d = 800# 6000
    disk_radius = 0.7
    area = (target_field[1] - target_field[0]) * (target_field[3] - target_field[2])
    p = math.pi * (disk_radius**2) / area

    pcd_list = []

    for index in range(-4, 0):

        file = root_dir + str(index) + ply_file
        lidar_pose = get_lidar_pose(scene_name, str(index))
        lidar_pose[3] = pitch; lidar_pose = lidar_pose[:3] + list(reversed(lidar_pose[3:]))
        trans_matrix = x1_to_x2(lidar_pose, [0]*6)

        # pcd_coordinate_convertor(file, trans_matrix)
        # total_nums, NUC = computeNUC(file, d, disk_radius, p, target_field)

        pcd, obj_tag = read_pcd(file) 
        pcd = semantic_filter_mat(pcd, obj_tag)
        pcd = project_points_by_matrix_torch(torch.Tensor(pcd), torch.Tensor(trans_matrix)).numpy()
        pcd_list.append(pcd)
        # viz(pcd)

    origin_lidar = np.vstack(pcd_list)
    point_list = area_filter(origin_lidar, target_field)
    
    try:
        total_nums, NUC = computeNUC_mat(point_list, d, disk_radius, p)
        print("total_nums:", total_nums, 'area', area)#, "\nNUC:", NUC)
        print("density:", total_nums/area, "\nNUC:", NUC, '\n')
        # viz(origin_lidar)
        viz(point_list)#, root_dir+'-1/pcd.png')
    except ValueError:
        print('point number == 0')

def main():

    # eval('/media/hdd1/opv2v/opencda_dump_test/metric_test_livox_6_-20', '2021_08_22_21_41_24', '-20')
    # return

    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--dir',type=str)
    args = argparser.parse_args()

    root_dir = args.dir
    pitch = root_dir.split('_')[-1]
    for scene in os.listdir(root_dir):
        eval(root_dir, scene, pitch)

if __name__ == '__main__':
    main()