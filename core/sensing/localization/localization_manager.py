# -*- coding: utf-8 -*-
"""
Localization module TODO: Will add more content next version
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import weakref
from collections import deque

import carla
import pymap3d as pm

from core.common.misc import get_speed


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

        # coordinates at current timestamp
        self.x, self.y, self.z, self.timestamp = 0.0, 0.0, 0.0, 0.0
        # latitude and longitude at current timestamp
        self.lat, self.lon, self.alt = 0.0, 0.0, 0.0
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

        # map origin in WG84 system #todo currently hard-coded
        lat_0 = 0.0
        lon_0 = 0.0
        alt_0 = 2.8

        enu_cordinates = pm.geodetic2enu(self.lat, self.lon, self.alt, lat_0, lon_0, alt_0)
        self.x, self.y, self.z = enu_cordinates[0], enu_cordinates[1], enu_cordinates[2]


class LocalizationManager(object):
    """
    The core class that manages localization estimation.
    """

    def __init__(self, vehicle, config_yaml):
        """
        Construction class
        :param vehicle: carla actor
        :param config_yaml: configuration related to localization
        """
        self.vehicle = vehicle
        self.activate = config_yaml['activate']

        # speed and transform and current timestamp
        self._ego_pos = None
        self._speed = 0

        # history track
        self._ego_pos_history = deque(maxlen=100)
        self._timestamp_history = deque(maxlen=100)

        self.gnss = GnssSensor(vehicle, config_yaml['gnss'])

    def localize(self):
        """
        Currently implemented in a naive way. todo: will add more contents next version
        :return:
        """

        if not self.activate:
            self._ego_pos = self.vehicle.get_transform()
            self._speed = get_speed(self.vehicle)
        else:
            # carla is ESU, while we convert the coordinates to ENU
            x, y, z = self.gnss.x, -self.gnss.y, self.gnss.z

            # in real world, it is very easy and accurate to get the vehicle yaw angle
            # hence here we retrieve the "groundtruth" directly from the server
            location = self.vehicle.get_transform().location  # todo remove this later
            print('ground truth location: %f, %f' % (location.x, location.y))
            print('gnss location: %f, %f' % (x, y))

            rotation = self.vehicle.get_transform().rotation
            self._ego_pos = carla.Transform(carla.Location(x=x, y=y, z=z),
                                            rotation)

            # for simplicity, we directly retrieve the true speed
            self._speed = get_speed(self.vehicle)

            self._ego_pos_history.append(self._ego_pos)
            self._timestamp_history.append(self.gnss.timestamp)

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
