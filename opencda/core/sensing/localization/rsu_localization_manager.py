# -*- coding: utf-8 -*-
"""
Localization module for RSU.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import weakref
from collections import deque

import carla

from opencda.core.common.misc import get_speed
from opencda.core.sensing.localization.coordinate_transform \
    import geo_to_transform


class GnssSensor(object):
    """
    The default GNSS sensor module for rsu.

    Parameters
    world : carla.world
        Carla world.

    config : dict
        The configuration dictionary of the localization module.

    global_position : list
        The global position of the rsu.

    Attributes

    blueprint : carla.blueprint
        The current blueprint of the sensor actor.

    weak_self : opencda Object
        A weak reference point to avoid circular reference.

    sensor : CARLA actor
        The current sensor actors that will be attach to the vehicles.
    """

    def __init__(self, world, config, global_position):
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
                    x=global_position[0],
                    y=global_position[1],
                    z=global_position[2])))

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


class LocalizationManager(object):
    """
    Default localization module for infrastructure.

    Parameters
    world : carla.world
        CARLA world.
    config_yaml : dict
        The configuration dictionary of the localization module.
    carla_map : carla.Map
        The carla HDMap. We need this to find the map origin to
        convert wg84 to enu coordinate system.

    Attributes
    gnss : opencda object
        GNSS sensor manager for spawning gnss sensor and listen to the data
        transmission.
    """

    def __init__(self, world, config_yaml, carla_map):

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

        self.gnss = GnssSensor(world,
                               config_yaml['gnss'],
                               config_yaml['global_position'])
        self.true_ego_pos = carla.Transform(
            carla.Location(x=config_yaml['global_position'][0],
                           y=config_yaml['global_position'][1],
                           z=config_yaml['global_position'][2]))
        self._speed = 0

    def localize(self):
        """
        Currently implemented in a naive way.
        """

        if not self.activate:
            self._ego_pos = self.true_ego_pos
        else:
            x, y, z = geo_to_transform(self.gnss.lat,
                                       self.gnss.lon,
                                       self.gnss.alt,
                                       self.geo_ref.latitude,
                                       self.geo_ref.longitude, 0.0)
            self._ego_pos = carla.Transform(
                carla.Location(x=x, y=y, z=z))

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
