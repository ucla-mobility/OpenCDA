# -*- coding: utf-8 -*-

"""Load the dumped yaml files and generate prediction/observed trajectory
for each vehicle
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os
import concurrent

from concurrent.futures import ThreadPoolExecutor
from opencda.scenario_testing.utils.yaml_utils import load_yaml, save_yaml


def retrieve_future_params(yaml_params, index, seconds=8):
    """
    Retrieve the yaml parameters for the next n seconds.

    Parameters
    ----------
    yaml_params : list
        The list that contains all loaded yaml parameters.

    index : int
        Current timestamp index.

    seconds : int
        The next n seconds we want to collect. Note the data collection is
        10hz.

    Returns
    -------
    future_params : list
        The list contains next n seconds' yaml parameters
    """
    start_index = min(index + 1, len(yaml_params) - 1)
    end_index = min(index + seconds * 10 + 1, len(yaml_params) - 1)
    future_params = yaml_params[start_index:end_index]

    return future_params


def retrieve_past_params(yaml_params, index, seconds=1):
    """
    Retrieve the yaml parameters for the past n seconds.

    Parameters
    ----------
    yaml_params : list
        The list that contains all loaded yaml parameters.

    index : int
        Current timestamp index.

    seconds : int
        The previous n seconds we want to collect. Note the data collection is
        10hz.

    Returns
    -------
    past_params : list
        The list contains previous n seconds' yaml parameters
    """
    end_index = max(index - 1, 0)
    start_index = max(index - seconds * 10, 0)
    past_params = yaml_params[start_index:end_index]

    return past_params


def extract_trajectories_by_id(object_id, yaml_param_list):
    """
    Extract a certain vehicle's future trajectory.

    Parameters
    ----------
    object_id : str
        Target object id.
    yaml_param_list : list
        The list contains next n seconds' yaml parameters.

    Returns
    -------
    predictions : list
        The future trajectory of object_id.
    """
    trajectories = []

    for yaml_param in yaml_param_list:
        vehicles = yaml_param['vehicles']

        if int(object_id) not in vehicles:
            break

        target_vehicle = vehicles[int(object_id)]

        location = target_vehicle['location']
        center = target_vehicle['center']
        rotation = target_vehicle['angle']
        speed = target_vehicle['speed']

        # we regard the center of the bbx as the vehicle true location
        trajectory = (location[0] + center[0],
                      location[1] + center[1],
                      location[2] + center[2],
                      rotation[0], rotation[1], rotation[2],
                      speed)
        trajectories.append(trajectory)

    return trajectories


def extract_trajectories_by_file(yaml_params,
                                 cur_index,
                                 past_seconds=1,
                                 future_seconds=8):
    """
    Extract the predictions and observation of all vehicles
    at the current index.

    Parameters
    ----------
    yaml_params : list
        All loaded yaml dictionaries.

    cur_index : int
        Current file index.

    past_seconds : int
        Previous n seconds for observation trajectory.

    future_seconds : int
        Next n seconds for prediction trajectory.

    Returns
    -------
    new_param : dict
        Update yaml params with the predictions.
    """
    cur_param = yaml_params[cur_index]

    for vehicle_id, vehicle in cur_param['vehicles'].items():
        future_yaml_params = retrieve_future_params(yaml_params, cur_index,
                                                    future_seconds)
        predictions = extract_trajectories_by_id(vehicle_id,
                                                 future_yaml_params)
        cur_param['vehicles'][vehicle_id].update({'predictions': predictions})

        past_yaml_params = retrieve_past_params(yaml_params, cur_index,
                                                past_seconds)
        observations = extract_trajectories_by_id(vehicle_id,
                                                  past_yaml_params)
        cur_param['vehicles'][vehicle_id]. \
            update({'observations': observations})

    return cur_param


def generate_prediction_by_scenario(scenario,
                                    future_seconds=8,
                                    past_seconds=1):
    """
    Generate prediction and observation trajectories by scenario.

    Parameters
    ----------
    future_seconds : int
        The number of seconds look ahead for prediction trajectory.

    past_seconds : int
        The number of seconds look back for observation trajectory.

    scenario : dict
        The scenario dictionary.
    """
    cavs = [os.path.join(scenario, x) for x in os.listdir(scenario)
            if not x.endswith('.yaml')]
    for (j, cav) in enumerate(cavs):
        yaml_files = \
            sorted([os.path.join(cav, x) for x in os.listdir(cav)
                    if x.endswith('.yaml')])

        # load all dictionarys at one time
        yaml_params = [load_yaml(x) for x in yaml_files]
        for k in range(len(yaml_files)):
            new_param = \
                extract_trajectories_by_file(yaml_params, k,
                                             past_seconds, future_seconds)
            save_yaml(new_param, yaml_files[k])


def generate_prediction_yaml(root_dir, future_seconds=8, past_seconds=1):
    """
    Overwrite the origin yaml files with the new yaml files that have
    Parameters
    ----------
    root_dir : str
        The data root directories.

    future_seconds : int
        The number of seconds look ahead for prediction trajectory.

    past_seconds : int
        The number of seconds look back for observation trajectory.
    """

    scenarios = [os.path.join(root_dir, x) for x in os.listdir(root_dir)]

    with ThreadPoolExecutor(max_workers=10) as executor:
        futures = [executor.submit(generate_prediction_by_scenario,
                                   scenario, future_seconds, past_seconds)
                   for scenario in scenarios]
        concurrent.futures.wait(futures)


if __name__ == '__main__':
    root_dir = '../data_dumping/'
    generate_prediction_yaml(root_dir)
