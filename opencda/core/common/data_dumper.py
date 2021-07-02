# -*- coding: utf-8 -*-
"""
Dumping sensor data.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import os

import cv2


class DataDumper:
    """
    Data dumper class to save data in local disk.

    Parameters
    ----------
    perception_manager : opencda object
        The perception manager contains rgb camera data and lidar data.

    localization_manager : opencda object
        The localization manager contains gnss and imu data.

    vehicle_id : int
        The carla.Vehicle id.

    save_time : str
        The timestamp at the beginning of the simulation.

    Attributes
    ----------
    rgb_camera : list
        A list of opencda.CameraSensor that containing all rgb sensor data
        of the managed vehicle.

    save_parent_folder : str
        The parent folder to save all data related to a specific vehicle.

    """
    def __init__(self,
                 perception_manager,
                 localization_manager,
                 vehicle_id,
                 save_time):

        self.rgb_camera = perception_manager.rgb_camera

        self.save_time = save_time
        self.vehicle_id = vehicle_id

        current_path = os.path.dirname(os.path.realpath(__file__))
        self.save_parent_folder = \
            os.path.join(current_path,
                         '../../../data_dumping',
                         save_time,
                         str(self.vehicle_id))

        if not os.path.exists(self.save_parent_folder):
            os.makedirs(self.save_parent_folder)

    def run_step(self):
        """
        Dump data at running time.
        Returns
        -------

        """
        for (i, camera) in enumerate(self.rgb_camera):

            timestamp = camera.timestamp
            image = camera.image

            if i == 0:
                camera_position = 'front'
            elif i == 1:
                camera_position = 'right'
            else:
                camera_position = 'left'

            image_name = str(timestamp) + '_' + camera_position + '.png'

            cv2.imwrite(os.path.join(self.save_parent_folder, image_name),
                        image)