# -*- coding: utf-8 -*-
"""
Obstacle vehicle class to save object detection.
"""

# Author: XH
# License: TDG-Attribution-NonCommercial-NoDistrib
import sys

import carla
import numpy as np
import open3d as o3d

import opencda.core.sensing.perception.sensor_transformation as st
from opencda.core.common.misc import get_speed_sumo

class TrafficSignal(object):
    """
    A class for traffic signals. The attributes are designed to work with CARLA 
    traffi light class.

    Parameters
    ----------
    corners : nd.nparray
        Eight corners of the bounding box. shape:(8, 3).

    Attributes
    ----------
    bounding_box : BoundingBox
        Bounding box of the osbject vehicle.

    """

    def __init__(self, traffic_light, light_id, type_id):
        '''
        Note: Traffic light is inherited from traffic sign, which is inherited from actor.
        '''
        self.traffic_light = traffic_light
        self.light_id = light_id
        self.type_id = type_id
        # note: potentially need bounding box, but not implemented yet.
        self.current_state = None
        self.green_time = 0.0
        self.red_time = 0.0
        self.yellow_time = 0.0
        self.elapsed_time = 0.0
        self.time_left = 0.0

    def calculate_time_left(self):
        if self.current_state == carla.TrafficLightState.Red:
            # red phase
            self.time_left = self.red_time - self.elapsed_time
        
        elif self.current_state == carla.TrafficLightState.Yellow:
            # yellow phase
            self.time_left = self.yellow_time - self.elapsed_time
        
        elif self.current_state == carla.TrafficLightState.Green:
            # green 
            self.time_left = self.green_time - self.elapsed_time

        else:
            # traffic signal not working 
            self.time_left = 0.0
            print('*** Warning stream: The current traffic signal is not working. ***')

    def get_location(self):
        """
        Return the location of the object vehicle.
        """
        return self.traffic_light.get_location()

    def get_spat_data(self):
        self.current_state = self.traffic_light.get_state()
        self.green_time = self.traffic_light.get_green_time()
        self.red_time = self.traffic_light.get_red_time()
        self.yellow_time = self.traffic_light.get_yellow_time()
        self.elapsed_time = self.traffic_light.get_elapsed_time()
        self.time_left = self.calculate_time_left()

    def run_step(self):
        # run along with opencda and calculate SPaT. 
        self.get_spat_data()
        print('*** Debug Stream Traffic Signal ***')
        print('[Signal LightID] --- ' + self.light_id)
        print('[Current state]  --- ' + self.current_state)
        print('[Elapsed time]   --- ' + self.elapsed_time)
        print('[Time Left]      --- ' + self.time_left)
    