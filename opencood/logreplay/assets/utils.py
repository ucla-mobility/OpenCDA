import numpy as np

from logreplay.assets.presave_lib import TOWN_DICTIONARY, BLUE_PRINT_LIB


def find_town(scenario_name):
    """
    Given the scenario name, find the corresponding town name,

    Parameters
    ----------
    scenario_name : str
        The scenario name.

    Returns
    -------
    The corresponding town's name.
    """
    return TOWN_DICTIONARY[scenario_name]


def find_blue_print(extent):
    for blueprint_name, blueprint_content in BLUE_PRINT_LIB.items():
        bp_extent = blueprint_content['extent']
        if abs(extent[0] - bp_extent[0]) < 0.001 and \
                abs(extent[1] - bp_extent[1]) < 0.001:
            return blueprint_name

    return None
