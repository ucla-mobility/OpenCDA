# -*- coding: utf-8 -*-
"""
Localization module
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import weakref
from collections import deque

import carla
import numpy as np

from core.common.misc import get_speed
from core.sensing.localization.coordinate_transform import geo_to_transform


class GnssSensor(object):
    """
    Class for gnss sensors
    """

    def __init__(self, vehicle, config):
        """
        Construct class
        :param vehicle: carla actor
        :param config: gnss configuration
        """
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')

        # set the noise for gps
        blueprint.set_attribute('noise_alt_stddev', str(config['noise_alt_stddev']))
        blueprint.set_attribute('noise_lat_stddev', str(config['noise_lat_stddev']))
        blueprint.set_attribute('noise_lon_stddev', str(config['noise_lon_stddev']))
        # spawn the sensor
        self.sensor = world.spawn_actor(blueprint, carla.Transform(carla.Location(x=0.0, y=0.0, z=2.8)),
                                        attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)

        # latitude and longitude at current timestamp
        self.lat, self.lon, self.alt, self.timestamp = 0.0, 0.0, 0.0, 0.0
        # create weak reference to avoid circular reference
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        """GNSS method"""
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude
        self.alt = event.altitude
        self.timestamp = event.timestamp


class ImuSensor(object):
    """
    IMU Sensor
    """
    def __init__(self, vehicle):
        """
        Construct class
        :param vehicle: Carla Actor
        """
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            blueprint, carla.Transform(), attach_to=vehicle)

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: ImuSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        # m/s^2
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        # rad/s
        self.gyroscope = (
            max(limits[0], min(limits[1], sensor_data.gyroscope.x)),
            max(limits[0], min(limits[1], sensor_data.gyroscope.y)),
            max(limits[0], min(limits[1], sensor_data.gyroscope.z)))
        self.compass = sensor_data.compass


class LocalizationManager(object):
    """
    The core class that manages localization estimation.
    """

    def __init__(self, vehicle, config_yaml, carla_map):
        """
        Construction class
        :param vehicle: carla actor
        :param config_yaml: configuration related to localization
        """
        self.vehicle = vehicle
        self.activate = config_yaml['activate']
        self.map = carla_map
        self.geo_ref = self.map.transform_to_geolocation(carla.Location(x=0, y=0, z=0))

        # speed and transform and current timestamp
        self._ego_pos = None
        self._speed = 0

        # history track
        self._ego_pos_history = deque(maxlen=100)
        self._timestamp_history = deque(maxlen=100)

        self.gnss = GnssSensor(vehicle, config_yaml['gnss'])
        # heading direction noise
        self.heading_noise_std = config_yaml['heading_direction_stddev']

    def localize(self):
        """
        Currently implemented in a naive way.
        :return:
        """

        if not self.activate:
            self._ego_pos = self.vehicle.get_transform()
            self._speed = get_speed(self.vehicle)
        else:
            if len(self._ego_pos_history) == 0:
                # assume initialization is accurate
                self._ego_pos = self.vehicle.get_transform()
            else:
                x, y, z = geo_to_transform(self.gnss.lat, self.gnss.lon, self.gnss.alt,
                                           self.geo_ref.latitude, self.geo_ref.longitude, 2.8)
                location = self.vehicle.get_transform().location  # todo debug purpose

                # We add synthetic noise to the heading direction
                rotation = self.vehicle.get_transform().rotation
                heading_angle = self.add_heading_direction_noise(rotation.yaw)

                print('-------------------------------------------------------------------')
                print('ground truth location: x: %f, y: %f, yaw: %f' % (location.x, location.y, rotation.yaw))
                print('gnss location: x: %f, y: %f, yaw: %f' % (x, y, heading_angle))

                self._ego_pos = carla.Transform(carla.Location(x=x, y=y, z=z),
                                                carla.Rotation(pitch=0, yaw=heading_angle, roll=0))

            # for simplicity, we directly retrieve the true speed
            self._speed = get_speed(self.vehicle)
            # save the track for future use
            self._ego_pos_history.append(self._ego_pos)
            self._timestamp_history.append(self.gnss.timestamp)

    def add_heading_direction_noise(self, heading_direction):
        """
        Add synthetic noise to heading direction
        :param heading_direction: groundtruth heading_direction obtained from the server
        :return: heading direction with noise
        """
        noise = np.random.normal(0, self.heading_noise_std)
        return heading_direction + noise

    def get_ego_pos(self):
        """
        Retrieve ego vehicle position
        :return: vehicle position
        """
        return self._ego_pos

    def get_ego_spd(self):
        """
        Retrieve ego vehicle speed
        :return:
        """
        return self._speed
