# -*- coding: utf-8 -*-
"""
Physcis-based trajectory prediction model
"""

# Author: CARLA Team, Hao Xiang <haxiang@g.ucla.edu>
# License: MIT

import numpy as np
from collections import deque


def angle_diff(x, y):
    """
    Get the smallest angle difference between 2 angles: the angle from y to x.
    Parameters
    ----------
    x : float
        To angle

    y : float
        From angle

    Returns
    -------
    diff : float
        Angle difference from y to x.

    """
    # calculate angle difference, modulo to [0, 2*pi]
    period = 2 * np.pi
    diff = (x - y + period / 2) % period - period / 2
    if diff > np.pi:
        diff = diff - (2 * np.pi)  # shift (pi, 2*pi] to (-pi, 0]
    return diff


class TrajectoryData:
    def __init__(self, observed_length):
        self.observed_traj = deque(maxlen=observed_length)
        self.observed_velocity = deque(maxlen=observed_length)
        self.observed_yaw = deque(maxlen=observed_length)

    def add(self, pose, v, yaw):
        self.observed_traj.add(pose)
        self.observed_velocity.add(v)
        self.observed_yaw.add(yaw)


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
    def __call__(self, observed_traj, v, yaw):
        x = observed_traj[-1][0]
        y = observed_traj[-1][1]
        vx, vy = v * np.cos(yaw), v * np.sin(yaw)
        pred_traj = constant_velocity_heading(x, y, vx, vy, self.predict_length, self.dt)
        return pred_traj


def constant_velocity_heading(x, y, vx, vy, predict_length, dt):
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
    preds : np.array
        Predicted trajectory with shape (predict_length, 2)

    """
    preds = []
    for i in range(1, predict_length + 1):
        t = i * dt
        preds.append((x + vx * t, y + vy * t))
    return np.array(preds)

