"""
Utility functions for planner.
"""
from enum import Enum

class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to other.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6

class AgentState(Enum):
    """
    AGENT_STATE represents the possible states of a roaming agent
    """
    VOID = -1
    NAVIGATING = 1
    BLOCKED_BY_VEHICLE = 2
    BLOCKED_BY_WALKER = 3
    BLOCKED_RED_LIGHT = 4
    BLOCKED_BY_BIKE = 5
    IDLE = 6

# types behaviors
class Cautious(object):
    """Class for Cautious agent."""
    max_speed = 40
    speed_lim_dist = 6
    speed_decrease = 12
    safety_time = 3
    min_proximity_threshold = 12
    braking_distance = 6
    overtake_counter = -1
    tailgate_counter = 0


class Normal(object):
    """Class for Normal agent."""
    max_speed = 50
    speed_lim_dist = 3
    speed_decrease = 10
    safety_time = 3
    min_proximity_threshold = 10
    braking_distance = 5
    overtake_counter = 0
    tailgate_counter = 0


class Aggressive(object):
    """Class for Aggressive agent."""
    max_speed = 70
    speed_lim_dist = 1
    speed_decrease = 8
    safety_time = 3
    min_proximity_threshold = 8
    braking_distance = 4
    overtake_counter = 0
    tailgate_counter = -1