# -*- coding: utf-8 -*-
"""
Customized Localization Module.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from opencda.core.sensing.localization.localization_manager \
    import LocalizationManager
from opencda.customize.core.sensing.localization.extented_kalman_filter \
    import ExtentedKalmanFilter


class CustomizedLocalizationManager(LocalizationManager):
    """Customized Localization module to replace the default module.

    Parameters
    -vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.
    -config_yaml: dict
        The configuration dictionary of the localization module.
    -carla_map: carla.Map
        The carla HDMap. We need this to find the map origin
        to convert wg84 to enu coordinate system.

    Attributes
    -kf : opencda object
        The filter used to fuse different sensors.
    """

    def __init__(self, vehicle, config_yaml, carla_map):
        super(
            CustomizedLocalizationManager,
            self).__init__(
            vehicle,
            config_yaml,
            carla_map)
        self.kf = ExtentedKalmanFilter(self.dt)
