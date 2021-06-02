# -*- coding: utf-8 -*-

"""
Since multiple CAV normally use the same ML/DL model, here we have this class to enable different
CAVs share the same model to avoid duplicate memory consumption.
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import torch
import sklearn


class MLManager(object):
    """
    A class that should contain all the ML models you want to initialize.
    """
    def __init__(self):
        """
        Construction class.
        """
        self.ob_detector = torch.hub.load('ultralytics/yolov5', 'yolov5s')