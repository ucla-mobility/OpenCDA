# -*- coding: utf-8 -*-
"""
Localization module
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import weakref
from collections import deque

import carla
import numpy as np

from opencda.core.common.misc import get_speed
from opencda.core.sensing.localization.localization_debug_helper \
    import LocDebugHelper
from opencda.core.sensing.localization.kalman_filter import KalmanFilter
from opencda.core.sensing.localization.coordinate_transform \
    import geo_to_transform


class GnssSensor(object):
    """
    The default GNSS sensor module.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    config : dict
        The configuration dictionary of the localization module.

    Attributes
    ----------
    sensor : CARLA actor
        The current sensor actors that will be attach to the vehicles.
    """

    def __init__(self, vehicle, config):
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')

        # set the noise for gps
        blueprint.set_attribute(
            'noise_alt_stddev', str(
                config['noise_alt_stddev']))
        blueprint.set_attribute(
            'noise_lat_stddev', str(
                config['noise_lat_stddev']))
        blueprint.set_attribute(
            'noise_lon_stddev', str(
                config['noise_lon_stddev']))
        # spawn the sensor
        self.sensor = world.spawn_actor(
            blueprint,
            carla.Transform(
                carla.Location(
                    x=0.0,
                    y=0.0,
                    z=0.0)),
            attach_to=vehicle,
            attachment_type=carla.AttachmentType.Rigid)

        # latitude and longitude at current timestamp
        self.lat, self.lon, self.alt, self.timestamp = 0.0, 0.0, 0.0, 0.0
        # create weak reference to avoid circular reference
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: GnssSensor._on_gnss_event(
                weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        """GNSS method that returns the current geo location."""
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude
        self.alt = event.altitude
        self.timestamp = event.timestamp


class ImuSensor(object):
    """
    Default ImuSensor module.

    Parameters
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    Attributes
    world : carla.world
        The caral world of the current vehicle.

    blueprint : carla.blueprint
        The current blueprint of the sensor actor.

    weak_self : opencda Object
        A weak reference point to avoid circular reference.

    sensor : CARLA actor
        The current sensor actors that will be attach to the vehicles.
    """

    def __init__(self, vehicle):
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            blueprint, carla.Transform(), attach_to=vehicle)

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: ImuSensor._IMU_callback(
                weak_self, sensor_data))
        self.gyroscope = None

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        """
        IMU method that returns the 3-D (x,y,z)
        acceleration and gyroscope values.
        """
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
    Default localization module.

    Parameters
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.
    config_yaml : dict
        The configuration dictionary of the localization module.
    carla_map : carla.Map
        The carla HDMap. We need this to find the map origin to
        convert wg84 to enu coordinate system.

    Attributes
    gnss : opencda object
        GNSS sensor manager for spawning gnss sensor and listen to the data
        transmission.
    ImuSensor : opencda object
        Imu sensor manager for spawning gnss sensor and listen to the data
        transmission.
    kf : opencda object
        The filter used to fuse different sensors.
    debug_helper : opencda object
        The debug helper is used to visualize the accuracy of
        the localization and provide evaluation functions.
    """

    def __init__(self, vehicle, config_yaml, carla_map):

        self.vehicle = vehicle
        self.activate = config_yaml['activate']
        self.map = carla_map
        self.geo_ref = self.map.transform_to_geolocation(
            carla.Location(x=0, y=0, z=0))

        # speed and transform and current timestamp
        self._ego_pos = None
        self._speed = 0

        # history track
        self._ego_pos_history = deque(maxlen=100)
        self._timestamp_history = deque(maxlen=100)

        self.gnss = GnssSensor(vehicle, config_yaml['gnss'])
        self.imu = ImuSensor(vehicle)

        # heading direction noise
        self.heading_noise_std = \
            config_yaml['gnss']['heading_direction_stddev']
        self.speed_noise_std = config_yaml['gnss']['speed_stddev']

        self.dt = config_yaml['dt']
        # Kalman Filter
        self.kf = KalmanFilter(self.dt)

        # DebugHelper
        self.debug_helper = LocDebugHelper(
            config_yaml['debug_helper'], self.vehicle.id)

    def localize(self):
        """
        Currently implemented in a naive way.
        """

        if not self.activate:
            self._ego_pos = self.vehicle.get_transform()
            self._speed = get_speed(self.vehicle)
        else:
            speed_true = get_speed(self.vehicle)
            speed_noise = self.add_speed_noise(speed_true)

            # gnss coordinates under ESU(Unreal coordinate system)
            x, y, z = geo_to_transform(self.gnss.lat,
                                       self.gnss.lon,
                                       self.gnss.alt,
                                       self.geo_ref.latitude,
                                       self.geo_ref.longitude, 0.0)

            # only use this for debugging purpose
            location = self.vehicle.get_transform().location

            # We add synthetic noise to the heading direction
            rotation = self.vehicle.get_transform().rotation
            heading_angle = self.add_heading_direction_noise(rotation.yaw)

            # assume the initial position is accurate
            if len(self._ego_pos_history) == 0:
                x_kf, y_kf, heading_angle_kf = x, y, heading_angle
                self._speed = speed_true
                self.kf.run_step_init(
                    x, y, np.deg2rad(heading_angle), self._speed / 3.6)
            else:
                x_kf, y_kf, heading_angle_kf, speed_kf = self.kf.run_step(
                    x, y, np.deg2rad(heading_angle),
                    speed_noise / 3.6,
                    self.imu.gyroscope[2])
                self._speed = speed_kf * 3.6
                heading_angle_kf = np.rad2deg(heading_angle_kf)

            # add data to debug helper
            self.debug_helper.run_step(x,
                                       y,
                                       heading_angle,
                                       speed_noise,
                                       x_kf,
                                       y_kf,
                                       heading_angle_kf,
                                       self._speed,
                                       location.x,
                                       location.y,
                                       rotation.yaw,
                                       speed_true)

            # the final pose of the vehicle
            self._ego_pos = carla.Transform(
                carla.Location(
                    x=x_kf, y=y_kf, z=z), carla.Rotation(
                    pitch=0, yaw=heading_angle_kf, roll=0))

            # save the track for future use
            self._ego_pos_history.append(self._ego_pos)
            self._timestamp_history.append(self.gnss.timestamp)

    def add_heading_direction_noise(self, heading_direction):
        """
        Add synthetic noise to heading direction.

        Parameters
        __________
        heading_direction : float
            groundtruth heading_direction obtained from the server.

        Returns
        -------
        heading_direction : float
            heading direction with noise.
        """
        return heading_direction + np.random.normal(0, self.heading_noise_std)

    def add_speed_noise(self, speed):
        """
        Add gaussian white noise to the current speed.

        Parameters
        __________
        speed : float
            m/s, current speed.

        Returns
        -------
        speed : float
            the speed with noise.
        """
        return speed + np.random.normal(0, self.speed_noise_std)

    def get_ego_pos(self):
        """
        Retrieve ego vehicle position
        """
        return self._ego_pos

    def get_ego_spd(self):
        """
        Retrieve ego vehicle speed
        """
        return self._speed

    def destroy(self):
        """
        Destroy the sensors
        """
        self.gnss.sensor.destroy()
        self.imu.sensor.destroy()
