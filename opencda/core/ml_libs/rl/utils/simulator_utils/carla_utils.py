'''
Copyright 2021 OpenDILab. All Rights Reserved:
Description:carla utils for DI-drive
'''

import numpy as np
import math
import carla

BACKGROUND = [0, 0, 0]


def control_to_signal(control):
    for k, v in control.items():
        if k in ['steer', 'throttle', 'brake', 'manual_gear_shift', 'gear']:
            control[k] = float(v)
    control_signal = carla.VehicleControl()
    control_signal.steer = control['steer'] if 'steer' in control else 0.0
    control_signal.throttle = control['throttle'] if 'throttle' in control else 0.0
    control_signal.brake = control['brake'] if 'brake' in control else 0.0
    if 'manual_gear_shift' in control:
        control_signal.manual_gear_shift = control['manual_gear_shift']
    if 'gear' in control:
        control_signal.gear = control['gear']
    return control_signal


def signal_to_control(signal):
    control = {
        'steer': signal.steer,
        'throttle': signal.throttle,
        'brake': signal.brake,
        'manual_gear_shift': signal.manual_gear_shift,
        'gear': signal.gear,
    }
    return control


def compute_angle(vec1, vec2):
    arr1 = np.array([vec1.x, vec1.y, vec1.z])
    arr2 = np.array([vec2.x, vec2.y, vec2.z])
    cosangle = arr1.dot(arr2) / (np.linalg.norm(arr1) * np.linalg.norm(arr2))
    angle = min(np.pi / 2, np.abs(np.arccos(cosangle)))
    return angle


def get_birdview(bev_data):
    birdview = [
        bev_data['road'],
        bev_data['lane'],
        bev_data['traffic'],
        bev_data['vehicle'],
        bev_data['pedestrian'],
        bev_data['hero'],
        bev_data['route'],
    ]
    birdview = [x if x.ndim == 3 else x[..., None] for x in birdview]
    birdview = np.concatenate(birdview, 2)
    birdview[birdview > 0] = 1

    return birdview


def visualize_birdview(birdview):
    """
    0 road
    1 lane
    2 red light
    3 yellow light
    4 green light
    5 vehicle
    6 pedestrian
    7 hero
    8 route
    """
    bev_render_colors = [
        (85, 87, 83),
        (211, 215, 207),
        (255, 0, 0),
        (255, 255, 0),
        (0, 255, 0),
        (252, 175, 62),
        (173, 74, 168),
        (32, 74, 207),
        (41, 239, 41),
    ]
    h, w, c = birdview.shape
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas[...] = BACKGROUND
    index_list = []
    for i in [0, 1, 2, 3, 4, 8, 5, 6, 7]:
        if i < c:
            index_list.append(i)

    for i in index_list:
        canvas[birdview[:, :, i] > 0.5] = bev_render_colors[i]

    return canvas


def calculate_speed(actor):
    """
    Method to calculate the velocity of a actor
    """
    speed_squared = actor.get_velocity().x ** 2
    speed_squared += actor.get_velocity().y ** 2
    speed_squared += actor.get_velocity().z ** 2
    return math.sqrt(speed_squared)


def convert_waypoint_to_transform(waypoint_vec):
    transform_vec = []
    for waypoint_tuple in waypoint_vec:
        transform_vec.append((waypoint_tuple[0].transform, waypoint_tuple[1]))

    return transform_vec


def lane_mid_distance(waypoint_location_list, location):
    num = min(len(waypoint_location_list), 5)  # use next 4 lines for lane mid esitimation
    if num <= 1:
        return 0
    #waypoint_location_list = 0.99 * waypoint_location_list[:-1] + 0.01 * waypoint_location_list[1:]
    start = waypoint_location_list[:num - 1, :2]  # start points of the 4 lines
    end = waypoint_location_list[1:num, :2]  # end   points of the 4 lines
    rotate = np.array([[0.0, -1.0], [1.0, 0.0]])
    normal_vec = (end - start).dot(rotate)
    loc = location[None, :2]
    dis = np.min(np.abs(np.sum(normal_vec * (loc - start), axis=1)) / np.sqrt(np.sum(normal_vec * normal_vec, axis=1)))
    return dis
