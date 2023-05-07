import open3d as o3d
import numpy as np
from opencda.customize.core.sensing.perception.utils import \
    draw_ground, draw_cluster, array_to_aabb_list
from sklearn.decomposition import PCA


def remove_ego(point_cloud: o3d.geometry.PointCloud):
    """
    Remove the points that hit the ego vehicle itself.

    Parameters
    ----------
    point_cloud : o3d.geometry.PointCloud

    Returns
    -------
    open3d.geometry.PointCloud: Point cloud without ego vehicle hit on.
    """
    points = np.asarray(point_cloud.points)
    # filter out the points that are -3 < x < 3 and -1.5 < y < 1.5
    mask = np.logical_and(np.logical_and(points[:, 0] > -2.2, points[:, 0] < 2.5),
                         np.logical_and(points[:, 1] > -1.5, points[:, 1] < 1.5))
    index = np.where(np.logical_not(mask))[0]

    return point_cloud.select_by_index(index)


def remove_ground(point_cloud: o3d.geometry.PointCloud,
                  threshold: float = 0.1, ransac_n: int = 3,
                  num_iterations: int = 100,
                  debug: bool = True) -> o3d.geometry.PointCloud:
    """Remove ground points from point cloud.

    Args:
        point_cloud (open3d.geometry.PointCloud): Point cloud.
        threshold (float, optional): Threshold to remove ground points. Defaults to 0.1.

    Returns:
        open3d.geometry.PointCloud: Point cloud without ground points.
    """
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=threshold,
                                                     ransac_n=ransac_n,
                                                     num_iterations=num_iterations)
    inlier_cloud = point_cloud.select_by_index(inliers)
    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)

    if debug:
        draw_ground(inlier_cloud, outlier_cloud)
    return outlier_cloud


def clusering(point_cloud: o3d.geometry.PointCloud,
              min_cluster_size: int = 10,
              debug: bool = False):

    """Clustering point cloud.

    Args:
        point_cloud (open3d.geometry.PointCloud): Point cloud.
        min_cluster_size (int, optional): Minimum cluster size. Defaults to 10.
        debug (bool, optional): Whether to visualize the clustering result. Defaults to False.
    """
    clusters = np.array(point_cloud.cluster_dbscan(eps=1,
                                          min_points=min_cluster_size,
                                          print_progress=False), dtype=int)
    # remove noise points that does not belong to any cluster. Those noise
    # points will be assigned to cluster with negative integer.
    select_index = np.where(clusters >= 0)[0]
    clusters = clusters[select_index]

    # remove noise points
    point_cloud = point_cloud.select_by_index(select_index)

    if debug:
        draw_cluster(point_cloud, np.array(clusters))
    return clusters, point_cloud


def compute_3d_bounding_boxes(cluster_labels, point_cloud):
    unique_labels = np.unique(cluster_labels)

    bounding_boxes = []

    for label in unique_labels:
        cluster_points = np.asarray(point_cloud.points)[cluster_labels == label]

        # Skip if there's only one point in the cluster
        if len(cluster_points) <= 1:
            continue

        # Estimate the yaw angle using PCA
        pca = PCA(n_components=2)
        xy_points = cluster_points[:, :2]
        pca.fit(xy_points)
        main_direction = pca.components_[0]
        yaw = np.arctan2(main_direction[1], main_direction[0])

        # Rotate the points using the estimated yaw angle
        rot_mat = np.array([[np.cos(yaw), -np.sin(yaw)],
                            [np.sin(yaw), np.cos(yaw)]])
        rotated_points = np.dot(xy_points, rot_mat)

        # Compute the bounding box dimensions in the rotated frame
        min_x, min_y = np.min(rotated_points, axis=0)
        max_x, max_y = np.max(rotated_points, axis=0)
        min_z, max_z = np.min(cluster_points[:, 2]), np.max(cluster_points[:, 2])

        # Compute the center and extents of the bounding box
        center = np.array([(min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2])
        extents = np.array([max_x - min_x, max_y - min_y, max_z - min_z])

        # # exclude the bbx that is too large
        if (extents > 6).any() or (extents < 0.5).any():
            continue
        # Transform the center back to the original coordinate system
        center[:2] = np.dot(center[:2], rot_mat.T)

        # Generate the 8 corners of the bounding box
        corners = np.array([
            [center[0] - extents[0] / 2, center[1] - extents[1] / 2, center[2] - extents[2] / 2],
            [center[0] + extents[0] / 2, center[1] - extents[1] / 2, center[2] - extents[2] / 2],
            [center[0] - extents[0] / 2, center[1] + extents[1] / 2, center[2] - extents[2] / 2],
            [center[0] + extents[0] / 2, center[1] + extents[1] / 2, center[2] - extents[2] / 2],
            [center[0] - extents[0] / 2, center[1] - extents[1] / 2, center[2] + extents[2] / 2],
            [center[0] + extents[0] / 2, center[1] - extents[1] / 2, center[2] + extents[2] / 2],
            [center[0] - extents[0] / 2, center[1] + extents[1] / 2, center[2] + extents[2] / 2],
            [center[0] + extents[0] / 2, center[1] + extents[1] / 2, center[2] + extents[2] / 2]])
        bounding_boxes.append(corners)

    bounding_boxes = np.array(bounding_boxes)
    return bounding_boxes


def detect_by_clustering(point_cloud: np.ndarray,
                         min_cluster_size: int = 10,
                         threshold: float = 0.3,
                         ransac_n: int = 10,
                         num_iterations: int = 200,
                         debug: bool = False):
    """Detect objects from point cloud.

    Args:
        point_cloud (np.ndarray): Point cloud.
        min_cluster_size (int, optional): Minimum cluster size. Defaults to 10.
        threshold (float, optional): Threshold for removing ground. Defaults to 0.3.
        ransac_n (int, optional): RANSAC n. Defaults to 10.
        num_iterations (int, optional): Number of iterations. Defaults to 200.
        debug (bool, optional): Whether to visualize the clustering result. Defaults to False.
    Returns:
        aabb_list (list): List of AxisAlignedBoundingBox.
        bbx_corners (np.ndarray): Bounding box corners, shape:(N, 8, 3).
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3])
    pcd = remove_ego(pcd)
    pcd = remove_ground(pcd, threshold=threshold, ransac_n=ransac_n, num_iterations=num_iterations, debug=debug)
    clusters, pcd = clusering(pcd, min_cluster_size=min_cluster_size, debug=debug)
    bbx_corners = compute_3d_bounding_boxes(clusters, pcd)
    # this is just for visualization during debug
    aabb_list = array_to_aabb_list(bbx_corners, debug=debug, point_cloud=pcd)
    return aabb_list, bbx_corners


if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud('debug.pcd')
    _, _ = detect_by_clustering(np.asarray(pcd.points), debug=True)
    print('done')
