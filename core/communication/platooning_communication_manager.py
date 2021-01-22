# -*- coding: utf-8 -*-

""" Platooning communication interface
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import carla


class PlatooningCommunicationManager(object):
    """
    Used to communicate with other CDA vehicles
    """
    def __init__(self):
        # TODO: Complete the dictionary
        self.status = {}
