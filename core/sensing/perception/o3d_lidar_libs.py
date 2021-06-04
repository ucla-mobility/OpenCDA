# -*- coding: utf-8 -*-
"""
Utility functions for 3d lidar visualization and processing by utilizing open3d.
"""

# Author: CARLA Team, Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import time

import open3d as o3d
import numpy as np

from matplotlib import cm

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255),  # None
    (70, 70, 70),  # Building
    (100, 40, 40),  # Fences
    (55, 90, 80),  # Other
    (220, 20, 60),  # Pedestrian
    (153, 153, 153),  # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),  # Vehicle
    (102, 102, 156),  # Wall
    (220, 220, 0),  # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),  # Ground
    (150, 100, 100),  # Bridge
    (230, 150, 140),  # RailTrack
    (180, 165, 180),  # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160),  # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),  # Water
    (145, 170, 100),  # Terrain
]) / 255.0  # normalize each channel [0-1] since is what Open3D uses


def o3d_pointcloud_encode(raw_data, point_cloud):
    """
    Encode the raw point cloud to Open3d PointCloud object.
    Args:
        raw_data (np.ndarray): Raw lidar points (N, (x, y, z, i)) obtained from lidar sensor.
        point_cloud (o3d.PointCloud):  Open3d PointCloud.

    Returns:
        (o3d.PointCloud): PointCloud with added points.
    """

    # Isolate the intensity and compute a color for it
    intensity = raw_data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    # Isolate the 3D data
    points = raw_data[:, :-1]
    # We're negating the y to correclty visualize a world that matches
    # what we see in Unreal since Open3D uses a right-handed coordinate system
    points[:, :1] = -points[:, :1]

    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(int_color)


def o3d_visualizer_init(actor_id):
    """
    Initialize the visualizer.
    Args:
        actor_id (int): Vehicle's id.
    Returns:
        (o3d.visualizer): Initialize Open3d visualizer.

    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=str(actor_id),
                      width=720,
                      height=405,
                      left=480,
                      top=270)
    vis.get_render_option().background_color = [0.05, 0.05, 0.05]
    vis.get_render_option().point_size = 1
    vis.get_render_option().show_coordinate_frame = True

    return vis


def o3d_visualizer_show(vis, count, point_cloud):
    """
    Visualize the point cloud at runtime.
    Args:
        vis (o3d.Visualizer): Visualization interface.
        count (int): current step since simulation started.
        point_cloud (o3d.PointCLoud): Open3d point clouds.

    Returns:

    """
    if count == 2:
        vis.add_geometry(point_cloud)
    vis.update_geometry(point_cloud)

    vis.poll_events()
    vis.update_renderer()
    # # This can fix Open3D jittering issues:
    time.sleep(0.001)