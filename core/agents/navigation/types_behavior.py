# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains the different parameters sets for each behavior. """


class Cautious(object):
    """Class for Cautious agent."""
    max_speed = 20
    tailgate_speed = 25
    speed_lim_dist = 6
    speed_decrease = 12
    inter_gap = 1.3
    safety_time = 3
    min_proximity_threshold = 12
    braking_distance = 6
    overtake_counter = -1
    tailgate_counter = 0


class Normal(object):
    """Class for Normal agent."""
    max_speed = 120 #90
    tailgate_speed = 55
    warm_up_speed = max_speed / 1.8
    speed_lim_dist = 3
    speed_decrease = 10
    inter_gap = 0.8
    open_gap = 1.8
    safety_time = 3
    min_proximity_threshold = 10
    braking_distance = max_speed / 3.6 * 0.1
    overtake_counter = 0
    tailgate_counter = 0


class Aggressive(object):
    """Class for Aggressive agent."""
    max_speed = 95
    tailgate_speed = 40
    speed_lim_dist = 1
    speed_decrease = 8
    inter_gap = 0.7
    safety_time = 3
    min_proximity_threshold = 8
    braking_distance = 3
    overtake_counter = 0
    tailgate_counter = -1
