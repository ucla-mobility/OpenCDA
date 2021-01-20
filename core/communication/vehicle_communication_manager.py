# -*- coding: utf-8 -*-

"""A class to manage ego-vehicle's communication
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla


class VehicleCommunicationManager(object):
    """
    Used to communicate with platoon/other vehicles
    """

    def __init__(self, search_range=35):
        """
        Construct class
        :param range: the searching range for communication
        """
        self._range = search_range
