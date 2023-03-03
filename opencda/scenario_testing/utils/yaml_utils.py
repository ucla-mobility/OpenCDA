# -*- coding: utf-8 -*-
"""
Used to load and write yaml files
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import re
import yaml
from datetime import datetime
from omegaconf import OmegaConf

def load_yaml(file):
    """
    Load yaml file and return a dictionary.
    Parameters
    ----------
    file : string
        yaml file path.

    Returns
    -------
    param : dict
        A dictionary that contains defined parameters.
    """

    stream = open(file, 'r')
    loader = yaml.Loader
    loader.add_implicit_resolver(
        u'tag:yaml.org,2002:float',
        re.compile(u'''^(?:
         [-+]?(?:[0-9][0-9_]*)\\.[0-9_]*(?:[eE][-+]?[0-9]+)?
        |[-+]?(?:[0-9][0-9_]*)(?:[eE][-+]?[0-9]+)
        |\\.[0-9_]+(?:[eE][-+][0-9]+)?
        |[-+]?[0-9][0-9_]*(?::[0-5]?[0-9])+\\.[0-9_]*
        |[-+]?\\.(?:inf|Inf|INF)
        |\\.(?:nan|NaN|NAN))$''', re.X),
        list(u'-+0123456789.'))
    param = yaml.load(stream, Loader=loader)

    # load current time for data dumping and evaluation
    current_time = datetime.now()
    current_time = current_time.strftime("%Y_%m_%d_%H_%M_%S")

    param['current_time'] = current_time

    return param


def add_current_time(params):
    """
    Add current time to the params dictionary.
    """
    # load current time for data dumping and evaluation
    current_time = datetime.now()
    current_time = current_time.strftime("%Y_%m_%d_%H_%M_%S")

    params['current_time'] = current_time

    return params


def save_yaml(data, save_name):
    """
    Save the dictionary into a yaml file.

    Parameters
    ----------
    data : dict
        The dictionary contains all data.

    save_name : string
        Full path of the output yaml file.
    """
    if isinstance(data, dict):
        with open(save_name, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
    else:
        with open(save_name, "w") as f:
            OmegaConf.save(data, f)

