import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import opencda.core.sensing.perception.sensor_transformation as st

def draw_ground(inlier_cloud, outlier_cloud):
    """Draw ground points and non-ground points."""
    inlier_cloud.paint_uniform_color([0, 1.0, 0.0])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


def draw_cluster(point_cloud: o3d.geometry.PointCloud,
                 clusters: np.ndarray):
    """Draw clusters."""
    max_label = clusters.max()
    colors = plt.get_cmap("tab20")(clusters / (max_label if max_label > 0 else 1))
    colors[clusters < 0] = 0
    point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([point_cloud])


def visualize_bbx_o3d(aabb_list, point_cloud):
    # Visualize the point cloud and bounding boxes in Open3D
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for aabb in aabb_list:
        vis.add_geometry(aabb)

    vis.add_geometry(point_cloud)
    vis.run()
    vis.destroy_window()

def array_to_aabb(bb_corner):
    min_corner = np.min(bb_corner, axis=0)
    max_corner = np.max(bb_corner, axis=0)

    aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_corner,
                                               max_bound=max_corner)
    return aabb

def array_to_aabb_list(bounding_boxes_array, debug=False, point_cloud=None):
    aabb_list = []

    for bb_corners in bounding_boxes_array:
        # when we really run the simulation, the y axis needs to be flipped
        # for visualization purpose
        if not debug:
            tmp = bb_corners.copy()
            tmp[:, 0] = -tmp[:, 0]
            aabb = array_to_aabb(tmp)
        else:
            aabb = array_to_aabb(bb_corners)
        aabb_list.append(aabb)

    if debug:
        visualize_bbx_o3d(aabb_list, point_cloud)
    return aabb_list


def project_bbx2world(corner, lidar_pose):
    # covert back to unreal coordinate
    # corner[:, :1] = -corner[:, :1]
    corner = corner.transpose()
    # extend (3, 8) to (4, 8) for homogenous transformation
    corner = np.r_[corner, [np.ones(corner.shape[1])]]
    # project to world reference
    corner = st.sensor_to_world(corner, lidar_pose)
    corner = corner.transpose()[:, :3]
    return corner
