# -*- coding: utf-8 -*-
"""
Physics-based trajectory prediction model
"""
import math
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
    def __init__(self, l):
        self.observed_traj = deque(maxlen=l)
        self.observed_velocity = deque(maxlen=l)
        self.observed_yaw = deque(maxlen=l)
        self.yaw_rate = 0
        self.acc = [0, 0]

    def add(self, pose, v, time_diff, current_yaw):
        # use current yaw from system to avoid getting 0.0
        # yaw when velocity is very slow.
        # current_yaw = np.arctan2(v[1], v[0])
        if len(self.observed_velocity):
            # update acceleration
            past_v = self.observed_velocity[-1]
            self.acc = [(v[i] - past_v[i]) / time_diff for i in range(2)]
            # update yaw rate
            past_yaw = self.observed_yaw[-1]
            self.yaw_rate = angle_diff(current_yaw, past_yaw) / time_diff

        self.observed_traj.append(pose)
        self.observed_velocity.append(v)
        self.observed_yaw.append(current_yaw)


def get_kinematics(trajectory_data, observed_length):
    observed_traj = list(trajectory_data.observed_traj)[-observed_length:]
    velocity = list(trajectory_data.observed_velocity)[-1]
    yaw = trajectory_data.observed_yaw[-1]
    yaw_rate = trajectory_data.yaw_rate
    acc = trajectory_data.acc
    return observed_traj, velocity, acc, yaw, yaw_rate


class PredictionManager:
    def __init__(self, observed_length, predict_length, dt, model="ConstantVelocityHeading"):
        assert isinstance(observed_length,
                          int) and observed_length > 0, "observed_length must be int and greater than 0"
        assert isinstance(predict_length, int) and predict_length > 0, "predict_length must be int and greater than 0"
        assert (isinstance(dt, int) or isinstance(dt, float)) and dt > 0, "dt must be real number and greater than 0"
        assert model in ["ConstantVelocityHeading", "ConstantAccelerationHeading",
                         "ConstantSpeedYawRate", "ConstantMagnitudeAccelAndYawRate", "PhysicsOracle"]
        self.observed_length = observed_length
        self.predict_length = predict_length
        self.dt = dt
        self.model = eval(model)(self.observed_length, self.predict_length, self.dt)
        self.vehicle_trajectory_data = {}
        self.objects = None
        self.vehicles = {}

    def update_information(self, objects):
        # only keep track of the vehicles that is visible in current frame
        self.vehicle_trajectory_data = {
            v.get_carla_id(): self.vehicle_trajectory_data.get(v.get_carla_id(), TrajectoryData(self.observed_length)) \
            for v in objects['vehicles']}
        self.objects = objects
        self.vehicles = {v.get_carla_id(): v for v in objects['vehicles']}
        for vehicle in objects['vehicles']:
            location = vehicle.get_location()
            x, y = location.x, location.y
            v = vehicle.get_velocity()
            yaw = math.radians(vehicle.get_transform().rotation.yaw)
            self.vehicle_trajectory_data[vehicle.get_carla_id()].add([x, y], [v.x, v.y], self.dt, yaw)

    def predict(self):
        predictions = {}
        for vehicle_id, vehicle in self.vehicles.items():
            kinematics_data = get_kinematics(self.vehicle_trajectory_data[vehicle_id], self.observed_length)
            predictions.update({
                vehicle_id: {
                    'vehicle': vehicle,
                    'points': self.model(kinematics_data)
                }
            })
        return predictions


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
    def __call__(self, kinematics_data):
        observed_traj, velocity, acc, yaw, yaw_rate = kinematics_data
        x = observed_traj[-1][0]
        y = observed_traj[-1][1]
        # vx, vy = v * np.cos(yaw), v * np.sin(yaw)
        vx, vy = velocity
        pred_traj = constant_velocity_heading(x, y, vx, vy, yaw, self.predict_length, self.dt)
        return pred_traj


class ConstantAccelerationHeading(Baseline):
    def __call__(self, kinematics_data):
        observed_traj, velocity, acc, yaw, yaw_rate = kinematics_data
        x = observed_traj[-1][0]
        y = observed_traj[-1][1]
        # vx, vy = v * np.cos(yaw), v * np.sin(yaw)
        vx, vy = velocity
        ax, ay = acc
        pred_traj = constant_acceleration_and_heading(x, y, vx, vy, yaw, ax, ay, self.predict_length, self.dt)
        return pred_traj


class ConstantSpeedYawRate(Baseline):
    def __call__(self, kinematics_data):
        observed_traj, velocity, acc, yaw, yaw_rate = kinematics_data
        x = observed_traj[-1][0]
        y = observed_traj[-1][1]
        # vx, vy = v * np.cos(yaw), v * np.sin(yaw)
        vx, vy = velocity
        pred_traj = constant_speed_and_yaw_rate(x, y, vx, vy, yaw, yaw_rate, self.predict_length, self.dt)
        return pred_traj


class ConstantMagnitudeAccelAndYawRate(Baseline):
    def __call__(self, kinematics_data):
        observed_traj, velocity, acc, yaw, yaw_rate = kinematics_data
        x = observed_traj[-1][0]
        y = observed_traj[-1][1]
        # vx, vy = v * np.cos(yaw), v * np.sin(yaw)
        vx, vy = velocity
        ax, ay = acc
        pred_traj = constant_magnitude_accel_and_yaw_rate(x, y, vx, vy, ax, ay,
                                                          yaw, yaw_rate, self.predict_length, self.dt)
        return pred_traj


class PhysicsOracle(Baseline):
    def __call__(self, kinematics_data, ground_truth):
        assert len(ground_truth.shape) == 2 and ground_truth.shape[0] == self.predict_length \
               and ground_truth.shape[1] == 2
        models = [
            ConstantVelocityHeading,
            ConstantAccelerationHeading,
            ConstantSpeedYawRate,
            ConstantMagnitudeAccelAndYawRate
        ]
        models = [model(self.observed_length, self.predict_length, self.dt) for model in models]
        all_preds = [model(kinematics_data) for model in models]
        oracles = sorted(all_preds,
                         key=lambda x: np.linalg.norm(np.array(x) - ground_truth, ord="fro"))
        oracle = oracles[0]
        return oracle


def constant_velocity_heading(x, y, vx, vy, yaw, predict_length, dt):
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

    yaw : float
        Vehicle's current yaw angle

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
        preds.append((x + vx * t, y + vy * t, yaw))
    return np.array(preds)


def constant_acceleration_and_heading(x, y, vx, vy, yaw, ax, ay, predict_length, dt):
    preds = []
    for i in range(1, predict_length + 1):
        t = i * dt
        half_time_squared = 1 / 2 * t * t
        preds.append((x + vx * t + half_time_squared * ax, y + vy * t + half_time_squared * ay, yaw))
    return np.array(preds)


def constant_speed_and_yaw_rate(x, y, vx, vy, yaw, yaw_rate, predict_length, dt):
    preds = []
    distance_step = np.sqrt(vx ** 2 + vy ** 2) * dt
    yaw_step = yaw_rate * dt
    for i in range(1, predict_length + 1):
        x += distance_step * np.cos(yaw)
        y += distance_step * np.sin(yaw)
        yaw += yaw_step
        preds.append((x, y, yaw))
    return np.array(preds)


def constant_magnitude_accel_and_yaw_rate(x, y, vx, vy, ax, ay, yaw, yaw_rate, predict_length, dt):
    preds = []
    a_value = np.sqrt(ax ** 2 + ay ** 2)
    v_value = np.sqrt(vx ** 2 + vy ** 2)
    speed_step = a_value * dt
    yaw_step = yaw_rate * dt
    for i in range(1, predict_length + 1):
        distance_step = dt * v_value
        x += distance_step * np.cos(yaw)
        y += distance_step * np.sin(yaw)
        v_value += speed_step
        yaw += yaw_step
        preds.append((x, y, yaw))
    return np.array(preds)
