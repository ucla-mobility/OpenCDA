from opencda.core.ml_libs.reinforcement_learning.utils.simulator_utils import RoadOption

def get_next_until_junction(start_waypoint, max_dis=float('inf')):
    start_loc = start_waypoint.transform.location
    cur_dis = -1
    end_wpt = start_waypoint
    while cur_dis < max_dis:
        cur_wpt = end_wpt.next(1)[0]
        if cur_wpt.is_junction:
            break
        end_wpt = cur_wpt
        cur_dis = end_wpt.transform.location.distance(start_loc)
    return end_wpt, cur_dis


def generate_change_lane_route(
    waypoint,
    change='left',
    distance_same_lane=10,
    distance_change_lane=5,
    distance_other_lane=25,
):
    """
    This methods generates a waypoint list which leads the vehicle to a parallel lane.
    The change input must be 'left' or 'right', depending on which lane you want to change.

    The step distance between waypoints on the same lane is 2m.
    The step distance between the lane change is set to 25m.

    @returns a waypoint list from the starting point to the end point on a right or left parallel lane.
    """
    plan = []
    plan.append((waypoint, RoadOption.LANEFOLLOW))  # start position

    step_distance = 2

    # same lane
    distance = 0
    while distance < distance_same_lane:
        next_wp = plan[-1][0].next(step_distance)
        distance += next_wp[0].transform.location.distance(plan[-1][0].transform.location)
        plan.append((next_wp[0], RoadOption.LANEFOLLOW))

    distance = 0
    if change == 'left':
        # go left
        wp_left = plan[-1][0].get_left_lane()
        wp_left_ahead = wp_left.next(distance_change_lane)[0]
        plan.append((wp_left_ahead, RoadOption.CHANGELANELEFT))
    elif change == 'right':
        # go right
        wp_right = plan[-1][0].get_right_lane()
        wp_right_ahead = wp_right.next(distance_change_lane)[0]
        plan.append((wp_right_ahead, RoadOption.CHANGELANERIGHT))

    # other lane
    distance = 0
    while distance < distance_other_lane:
        next_wp = plan[-1][0].next(step_distance)
        distance += next_wp[0].transform.location.distance(plan[-1][0].transform.location)
        plan.append((next_wp[0], RoadOption.LANEFOLLOW))

    return plan
