# -*- coding: utf-8 -*-
"""
Used to load and write yaml files
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import re
import yaml


def load_yaml(file):
    """
    Load yaml file and return a dictionary.

    Args:
        -file (string): yaml file path.
    Returns: 
        - param (dict): A dictionary that contains defined parameters.
    """

    stream = open(file, 'r')
    loader = yaml.SafeLoader
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

    return param
