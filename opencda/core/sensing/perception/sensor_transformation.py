# -*- coding: utf-8 -*-
"""
This script contains the transformations between world and different sensors.
"""
# Credit to https://github.com/MukhlasAdib/CARLA-2DBBox/blob/master/carla_vehicle_annotator.py
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import numpy as np
from matplotlib import cm

from opencda.opencda_carla import Transform

VIRIDIS = np.array(cm.get_cmap('viridis').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])



def get_camera_intrinsic(sensor):
    """
    Retrieve the camera intrinsic matrix
    Args:
        -sensor (carla.sensor.camera.rgb): The CARLA sensor object.

    Returns:
        -matrix_k (np.ndarray): The 2D intrinsic matrix.
    """
    VIEW_WIDTH = int(sensor.attributes['image_size_x'])
    VIEW_HEIGHT = int(sensor.attributes['image_size_y'])
    VIEW_FOV = int(float(sensor.attributes['fov']))

    matrix_k = np.identity(3)
    matrix_k[0, 2] = VIEW_WIDTH / 2.0
    matrix_k[1, 2] = VIEW_HEIGHT / 2.0
    matrix_k[0, 0] = matrix_k[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))

    return matrix_k


def create_bb_points(vehicle):
    """
    Extract the eight vertices of the bounding box from the vehicle.
    Args:
        -vehicle (carla.Vehicle or ObstacleVehicle): The object vehicle. 

    Returns:
        - bbx(np.ndarray): 3d bounding box.
    """
    bbx = np.zeros((8, 4))
    extent = vehicle.bounding_box.extent

    bbx[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
    bbx[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
    bbx[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
    bbx[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
    bbx[4, :] = np.array([extent.x, extent.y, extent.z, 1])
    bbx[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
    bbx[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
    bbx[7, :] = np.array([extent.x, -extent.y, extent.z, 1])

    return bbx


def x_to_world_transformation(transform):
    """
    Get the transformation matrix from x(it can be vehicle or sensor) coordinates to world coordinate.
    Args:
        -transform (carla.Transform): The transform that contains location and rotation.

    Returns:
        -matrix (np.ndarray): The transformation matrix
    """
    rotation = transform.rotation
    location = transform.location

    # used for rotation matrix
    c_y = np.cos(np.radians(rotation.yaw))
    s_y = np.sin(np.radians(rotation.yaw))
    c_r = np.cos(np.radians(rotation.roll))
    s_r = np.sin(np.radians(rotation.roll))
    c_p = np.cos(np.radians(rotation.pitch))
    s_p = np.sin(np.radians(rotation.pitch))

    matrix = np.identity(4)
    # translation matrix
    matrix[0, 3] = location.x
    matrix[1, 3] = location.y
    matrix[2, 3] = location.z

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


def bbx_to_world(cords, vehicle):
    """
    Convert bounding box coordinate at vehicle reference to world reference.
    Args:
        -cords (np.ndarray): Bounding box coordinates with 8 vertices, shape (n, 4).
        -vehicle (carla.vehicle or ObstacleVehicle): vehicle object.

    Returns:
        -bb_world_cords (np.ndarray): Bounding box coordinates under word reference.
    """

    bb_transform = Transform(vehicle.bounding_box.location)

    # bounding box to vehicle transformation matrix
    bb_vehicle_matrix = x_to_world_transformation(bb_transform)

    # vehicle to world transformation matrix
    vehicle_world_matrix = x_to_world_transformation(vehicle.get_transform())
    # bounding box to world transformation matrix
    bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)

    # 8 vertices are relative to bbx center, thus multiply with bbx_2_world to get the world coords.
    bb_world_cords = np.dot(bb_world_matrix, np.transpose(cords))

    return bb_world_cords


def world_to_sensor(cords, sensor_transform):
    """
    Transform coordinate from world reference to sensor reference.
    Args:
        -cords (np.ndarray): Coordinates under world reference, shape:(4, n).
        -sensor_transform (carla.Transform): sensor position in the world, shape:(3, 1).

    Returns:
        -sensor_cords(np.ndarray): Coordinates in sensor reference.
    """
    sensor_world_matrix = x_to_world_transformation(sensor_transform)
    world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
    sensor_cords = np.dot(world_sensor_matrix, cords)

    return sensor_cords


def sensor_to_world(cords, sensor_transform):
    """
    Project
    Args:
        -cords (np.ndarray): Coordinates under sensor reference.
        -sensor_transform (carla.Transform): sensor position in the world
    Returns:
        -world_cords (np.ndarray): Coordinates projected to world space.
    """
    sensor_world_matrix = x_to_world_transformation(sensor_transform)
    world_cords = np.dot(sensor_world_matrix, cords)

    return world_cords


def vehicle_to_sensor(cords, vehicle, sensor_transform):
    """
    Transform coordinates from vehicle reference to sensor reference
    Args:
        -cords (np.ndarray): Coordinates under vehicle reference, shape (n, 4).
        -vehicle (carla.vehicle or ObstacleVehicle): vehicle object.
        -sensor_transform (carla.Transform): sensor position in the world, shape(3, 1).

    Returns:
        -(np.ndarray): Coordinates in sensor reference, shape(4, n).
    """
    world_cord = bbx_to_world(cords, vehicle)
    sensor_cord = world_to_sensor(world_cord, sensor_transform)

    return sensor_cord


def get_bounding_box(vehicle, camera, sensor_transform):
    """
    Get vehicle bounding box and project to sensor image.
    Args:
        -vehicle (carla.vehicle or ObstacleVehicle): vehicle object.
        -camera (carla.sensor.camera.rgb): The CARLA sensor object.
        -sensor_transform (carla.Transform): sensor position in the world

    Returns:
        -camera_bbox (np.ndarray): Bounding box coordinates in sensor image.
    """
    camera_k_matrix = get_camera_intrinsic(camera)
    # bb_cords is relative to bbx center(approximate the vehicle center)
    bb_cords = create_bb_points(vehicle)

    # bbx coordinates in sensor coordinate system. shape: (3, 8)
    cords_x_y_z = vehicle_to_sensor(bb_cords, vehicle, sensor_transform)[:3, :]
    # refer to https://github.com/carla-simulator/carla/issues/553
    cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :].reshape(1, 8),
                                        -cords_x_y_z[2, :].reshape(1, 8),
                                        cords_x_y_z[0, :].reshape(1, 8)])
    # bounding box in sensor image. Shape:(8, 3)
    bbox = np.transpose(np.dot(camera_k_matrix, cords_y_minus_z_x))

    new_x = (bbox[:, 0] / bbox[:, 2]).reshape(8, 1)
    new_y = (bbox[:, 1] / bbox[:, 2]).reshape(8, 1)
    new_z = bbox[:, 2].reshape(8, 1)
    camera_bbox = np.concatenate([new_x, new_y, new_z], axis=1)

    return camera_bbox


def p3d_to_p2d_bb(p3d_bb):
    """
    Draw 2D bounding box (4 vertices) from 3D bounding box (8 vertices) in image.
    2D bounding box is represented by two corner points
    Args:
        -p3d_bb (np.array): The objective 3D bounding box. 

    Returns:

    """
    min_x = np.amin(p3d_bb[:, 0])
    min_y = np.amin(p3d_bb[:, 1])
    max_x = np.amax(p3d_bb[:, 0])
    max_y = np.amax(p3d_bb[:, 1])
    p2d_bb = np.array([[min_x, min_y], [max_x, max_y]])
    return p2d_bb


def get_2d_bb(vehicle, sensor, senosr_transform):
    """
    Summarize 2D bounding box creation
    Args:
        -vehicle (carla.vehicle or ObstacleVehicle): vehicle object.
        -sensor (carla.sensor.camera.rgb): The CARLA sensor object.
        -senosr_transform (carla.Transform): sensor position in the world

    Returns:
        -p2d_bb (np.ndarray): 2d bounding box in camera image.

    """
    p3d_bb = get_bounding_box(vehicle, sensor, senosr_transform)
    p2d_bb = p3d_to_p2d_bb(p3d_bb)
    return p2d_bb


def project_lidar_to_camera(lidar, camera, point_cloud, rgb_image):
    """
    Project lidar to camera space.
    Args:
        -lidar (carla.Sensor): Lidar sensor.
        -camera (carla.Sensor): Camera seonsor.
        -point_cloud (np.ndarray): cloud points, (x, y, z, intensity).
        -rgb_image (np.ndarray): rgb image from camera.

    Returns:
        -rgb_image (np.ndarray): new rgb image with lidar points projected.
        -points_2d (np.ndarray): point clouds projected to camera space.
    """

    # Lidar intensity array of shape (p_cloud_size,) but, for now, let's
    # focus on the 3D points.
    intensity = np.array(point_cloud[:, 3])

    # Point cloud in lidar sensor space array of shape (3, p_cloud_size).
    local_lidar_points = np.array(point_cloud[:, :3]).T

    # Add an extra 1.0 at the end of each 3d point so it becomes of
    # shape (4, p_cloud_size) and it can be multiplied by a (4, 4) matrix.
    local_lidar_points = np.r_[
        local_lidar_points, [np.ones(local_lidar_points.shape[1])]]

    # This (4, 4) matrix transforms the points from lidar space to world space.
    lidar_2_world = x_to_world_transformation(lidar.get_transform())

    # transform lidar points from lidar space to world space
    world_points = np.dot(lidar_2_world, local_lidar_points)

    # project world points to camera space
    sensor_points = world_to_sensor(world_points, camera.get_transform())

    # (x, y ,z) -> (y, -z, x)
    point_in_camera_coords = np.array([
        sensor_points[1],
        sensor_points[2] * -1,
        sensor_points[0]])

    # retrieve camera intrinsic
    K = get_camera_intrinsic(camera)
    # project the 3d points in camera space to image space
    points_2d = np.dot(K, point_in_camera_coords)

    # normalize x,y,z
    points_2d = np.array([
        points_2d[0, :] / points_2d[2, :],
        points_2d[1, :] / points_2d[2, :],
        points_2d[2, :]])

    image_w = int(camera.attributes['image_size_x'])
    image_h = int(camera.attributes['image_size_y'])

    # remove points out the camera scope
    points_2d = points_2d.T
    intensity = intensity.T
    points_in_canvas_mask = \
        (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < image_w) & \
        (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < image_h) & \
        (points_2d[:, 2] > 0.0)
    new_points_2d = points_2d[points_in_canvas_mask]
    new_intensity = intensity[points_in_canvas_mask]

    # Extract the screen coords (uv) as integers.
    u_coord = new_points_2d[:, 0].astype(np.int)
    v_coord = new_points_2d[:, 1].astype(np.int)

    # Since at the time of the creation of this script, the intensity function
    # is returning high values, these are adjusted to be nicely visualized.
    new_intensity = 4 * new_intensity - 3
    color_map = np.array([
        np.interp(new_intensity, VID_RANGE, VIRIDIS[:, 0]) * 255.0,
        np.interp(new_intensity, VID_RANGE, VIRIDIS[:, 1]) * 255.0,
        np.interp(new_intensity, VID_RANGE, VIRIDIS[:, 2]) * 255.0]).astype(np.int).T

    for i in range(len(new_points_2d)):
        rgb_image[v_coord[i] - 1: v_coord[i] + 1, u_coord[i] - 1: u_coord[i] + 1] = color_map[i]

    return rgb_image, points_2d