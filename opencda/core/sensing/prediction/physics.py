# -*- coding: utf-8 -*-
"""
Physcis-based trajectory prediction model
"""


# Author: CARLA Team, Hao Xiang <haxiang@g.ucla.edu>
# License: MIT

import numpy as np
from collections import deque

class TrajectoryData:
    def __init__(self, observed_length):
        self.observed_traj = deque(maxlen=observed_length)
        self.observed_velocity = deque(maxlen=observed_length)
        self.observed_yaw = deque(maxlen=observed_length)
        self.observed_yaw_rate = deque(maxlen=observed_length)
    def add(self, pose, v, yaw, yaw_rate):
        self.observed_traj.add(pose)
        self.observed_velocity.add(v)
        self.observed_yaw.add(yaw)
        self.observed_yaw_rate(yaw_rate)





class Baseline:
    """
    Baseline class for physics-based trajectory prediction model.
    Parameters
    ----------
    observed_length : int
        Observed trajectory length.

    predict_length : int
        Predicted trajectory length.

    dt : float
        Time discretization interval.

    """

    def __init__(self, observed_length, predict_length, dt):
        assert isinstance(observed_length,
                          int) and observed_length > 0, "observed_length must be int and greater than 0"
        assert isinstance(predict_length, int) and predict_length > 0, "predict_length must be int and greater than 0"
        assert (isinstance(dt, int) or isinstance(dt, float)) and dt > 0, "dt must be real number and greater than 0"
        self.observed_length = observed_length
        self.predict_length = predict_length
        self.dt = dt

class ConstantVelocityHeading(Baseline):
    def __call__(self, observed_traj, observed_velocity, yaw):
        x = observed_velocity[-1][0]
        y = observed_velocity[-1][1]
        v = observed_velocity[-1]



def constant_velocity_heading(self, x, y, vx, vy, predict_length, dt):
    """
    Predict trajectories based on constant velocity.
    Parameters
    ----------
    x : float
        Vehicle's current x-axis position.

    y : float
        Vehicle's current x-axis position.

    vx : float
        Vehicle's current x-axis velocity.

    vy : float
        Vehicle's current y-axis velocity.

    predict_length : int
        Predicted trajectory length.

    dt : float
        Time discretization interval.

    Returns
    -------

    """
    preds = []
    for i in range(1, predict_length+1):
        t = i * dt
        preds.append((x + vx * t, y + vy * t))
    return np.array(preds)


