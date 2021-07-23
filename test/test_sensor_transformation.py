# -*- coding: utf-8 -*-
"""
Unit test for sensor transformation.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import os
import sys
import unittest

import numpy as np

# temporary solution for relative imports in case opencda is not installed
# if opencda is installed, no need to use the following line
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

import mocked_carla as mcarla
from opencda.core.sensing.perception.sensor_transformation import *


class TestSensorTransformation(unittest.TestCase):
    def setUp(self):
        # random cords, [(x, y, z, 1), n]
        self.cords = np.random.random(size=(4, 10))
        self.vehicle = mcarla.Vehicle()
        self.camera = mcarla.Camera({'image_size_x': 600,
                                     'image_size_y': 800,
                                     'fov': 90})
        self.lidar = mcarla.Lidar({'channels': 32,
                                   'range': 50})

        self.camera_transform = mcarla.Transform(x=11, y=11, z=11)
        self.lidar_transform = mcarla.Transform(x=10, y=10, z=10)

        self.rgb_image = np.random.randint(0, 255, size=(800, 600, 3)).astype('uint8')
        self.point_cloud = np.random.random(size=(100, 4))

    def test_x_to_world_transformation(self):
        assert x_to_world_transformation(self.lidar_transform).shape == (4, 4)
        assert x_to_world_transformation(self.lidar_transform)[3, 3] == 1

    def test_world_to_sensor(self):
        assert world_to_sensor(self.cords, self.lidar_transform).shape == (4, self.cords.shape[1])

    def test_sensor_to_world(self):
        assert sensor_to_world(self.cords, self.lidar_transform).shape == (4, self.cords.shape[1])

    def test_get_camera_intrinsic(self):
        assert get_camera_intrinsic(self.camera).shape == (3, 3)
        assert get_camera_intrinsic(self.camera)[2, 2] == 1

    def test_create_bb_points(self):
        assert create_bb_points(self.vehicle).shape == (8, 4)
        assert create_bb_points(self.vehicle)[:, 3].all() == 1

    def test_bbx_to_world(self):
        assert bbx_to_world(self.cords.T, self.vehicle).shape == (4, self.cords.shape[1])

    def test_vehicle_to_sensor(self):
        assert vehicle_to_sensor(self.cords.T, self.vehicle, self.camera_transform).shape == (4, self.cords.shape[1])

    def test_get_bounding_box(self):
        assert get_bounding_box(self.vehicle, self.camera, self.camera_transform).shape == (8, 3)

    def test_get_2d_bb(self):
        assert get_2d_bb(self.vehicle, self.camera, self.camera_transform).shape == (2, 2)

    def test_project_lidar_to_camera(self):
        assert project_lidar_to_camera(self.lidar, self.camera, self.point_cloud, self.rgb_image)[1].shape == \
               (self.point_cloud.shape[0], 3)
        assert project_lidar_to_camera(self.lidar, self.camera, self.point_cloud, self.rgb_image)[0].shape == \
               self.rgb_image.shape


if __name__ == '__main__':
    unittest.main()
