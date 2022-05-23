
from velocityplanner import planrelaxvelocity
import math
import matplotlib.pyplot as plt

class TrajectoryPoint:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.dir_x = 0
        self.dir_y = 0
        self.dir_z = 0
        self.theta = 0
        self.velocity = 0
        self.acceleration = 0
        self.curvature = 0
        self.sumdistance = 0
        
def gen_bpp_without_velocity(goal_pos, goal_dir, start_pos, start_dir, interval):

    control_point_shift = 1.0 / 3
    length = math.sqrt((goal_pos[0] - start_pos[0]) ** 2 + (goal_pos[1] - start_pos[1]) ** 2)
    norm_goal_dir = math.sqrt(goal_dir[0]**2 + goal_dir[1]**2)
    goal_dir[0] = goal_dir[0] / norm_goal_dir
    goal_dir[1] = goal_dir[1] / norm_goal_dir
    goal_dir[2] = goal_dir[2] / norm_goal_dir

    norm_start_dir = math.sqrt(start_dir[0]**2 + start_dir[1]**2)
    start_dir[0] = start_dir[0] / norm_start_dir
    start_dir[1] = start_dir[1] / norm_start_dir
    start_dir[2] = start_dir[2] / norm_start_dir

    P1 = [0, 0, 0]
    P2 = [0, 0, 0]
    P1[0] = start_pos[0] + length * control_point_shift * start_dir[0]
    P1[1] = start_pos[1] + length * control_point_shift * start_dir[1]
    P1[2] = start_pos[2] + length * control_point_shift * start_dir[2]

    P2[0] = goal_pos[0] - length * control_point_shift * goal_dir[0]
    P2[1] = goal_pos[1] - length * control_point_shift * goal_dir[1]
    P2[2] = goal_pos[2] - length * control_point_shift * goal_dir[2]

    P0 = start_pos
    P3 = goal_pos
    appro_length = math.sqrt((start_pos[0] - P1[0]) ** 2 + (start_pos[1] - P1[1]) ** 2 + (start_pos[2] - P1[2]) ** 2) + \
                   math.sqrt((goal_pos[0] - P2[0]) ** 2 + (goal_pos[1] - P2[1]) ** 2 + (goal_pos[2] - P2[2]) ) + \
                   math.sqrt((P1[0] - P2[0]) ** 2 + (P1[1] - P2[1]) ** 2 + (P1[2] - P2[2]) ** 2) + \
                   math.sqrt((start_pos[0] - goal_pos[0]) ** 2 + (start_pos[1] - goal_pos[1]) **2 + (start_pos[2] - goal_pos[2]) **2)
    appro_length = appro_length * 0.5
    dt = interval / (appro_length + 0.01)
    if (dt < 1e-3):
        dt = 1e-3
    t = 0
    list_ret=[]
    while t < 1:
        point = TrajectoryPoint()
        point.x = P0[0] * (1 - 3 * t + 3 * t * t - t * t * t) + P1[0] * (3 * t - 6 * t * t + 3 * t * t * t) + P2[0] * (3 * t * t - 3 * t * t * t) + P3[0] * (t * t * t)
        point.y = P0[1] * (1 - 3 * t + 3 * t * t - t * t * t) + P1[1] * (3 * t - 6 * t * t + 3 * t * t * t) + P2[1] * (3 * t * t - 3 * t * t * t) + P3[1] * (t * t * t)
        point.z = P0[2] * (1 - 3 * t + 3 * t * t - t * t * t) + P1[2] * (3 * t - 6 * t * t + 3 * t * t * t) + P2[2] * (3 * t * t - 3 * t * t * t) + P3[2] * (t * t * t)

        point.dir_x = P0[0] * (-3 + 6 * t - 3 * t * t) + P1[0] * (3 - 12 * t + 9 * t * t) + P2[0] * (6 * t - 9 * t * t) + P3[0] * (3 * t * t)
        point.dir_y = P0[1] * (-3 + 6 * t - 3 * t * t) + P1[1] * (3 - 12 * t + 9 * t * t) + P2[1] * (6 * t - 9 * t * t) + P3[1] * (3 * t * t)
        point.dir_z = P0[2] * (-3 + 6 * t - 3 * t * t) + P1[2] * (3 - 12 * t + 9 * t * t) + P2[2] * (6 * t - 9 * t * t) + P3[2] * (3 * t * t)

        pos_dd_x = P0[0] * (6 - 6 * t) + P1[0] * (-12 + 18 * t) + P2[0] * (6 - 18 * t) + P3[0] * (6 * t)
        pos_dd_y = P0[1] * (6 - 6 * t) + P1[1] * (-12 + 18 * t) + P2[1] * (6 - 18 * t) + P3[1] * (6 * t)
        pos_dd_z = P0[2] * (6 - 6 * t) + P1[2] * (-12 + 18 * t) + P2[2] * (6 - 18 * t) + P3[2] * (6 * t)

        point.theta = 0
        if (point.dir_x == 0):
            if (point.dir_y > 0):
                point.theta = math.pi / 2
            elif (point.dir_y < 0):
                point.theta = -math.pi / 2
            else:
                point.theta = 0
        else:
            point.theta = math.atan2(point.dir_y, point.dir_x)
        point.curvature = (point.dir_x * pos_dd_y - pos_dd_x * point.dir_y) / ((point.dir_x * point.dir_x + point.dir_y * point.dir_y) ** (3.0 / 2.0))
        norm_dir = math.sqrt(point.dir_x ** 2 + point.dir_y ** 2 + point.dir_z ** 2)
        point.dir_x = point.dir_x / norm_dir
        point.dir_y = point.dir_y / norm_dir
        point.dir_z = point.dir_z / norm_dir
        list_ret.append(point)
        t = t + dt
    return list_ret

def gen_bpp_with_velocity(goal_pos, goal_dir, start_pos, start_dir, interval, vpurpose, cur_vel, tgt_vel, cur_acc):
    traj_ret = gen_bpp_without_velocity(goal_pos, goal_dir, start_pos, start_dir, interval)
    return planrelaxvelocity(traj_ret, vpurpose, cur_vel, tgt_vel, cur_acc)

class BppPlanner(object):
    def __init__(self, interval):
        self.interval = interval

    def run_step_with_velocity(self, goal_pos, goal_dir, start_pos, start_dir, vpurpose, cur_vel, tgt_vel, cur_acc):
        return gen_bpp_with_velocity(goal_pos, goal_dir, start_pos, start_dir, self.interval, vpurpose, cur_vel, tgt_vel, cur_acc)

    def run_step_without_velocity(self, goal_pos, goal_dir, start_pos, start_dir):
        return gen_bpp_without_velocity(goal_pos, goal_dir, start_pos, start_dir, self.interval)

if __name__ == '__main__':
    test_list=gen_bpp_with_velocity([22, 15], [0, 1, 0], [0, 0], [1, 0, 0], 0.2, 1, 8.333, 2.335, 7.0)
    # Bugs for this 
    test_list=gen_bpp_with_velocity_qp([60, 0], [1, 0], [0, 0], [1, 0], 0.2, 2.7778, 8.3334, 1.0)
    xx=[]
    yy=[]
    for x in test_list:
        xx.append(x.x)
        yy.append(x.y)
    plt.plot(xx, yy, "r")
    plt.figure()
    xx=[]
    yy=[]
    zz=[]
    i=0
    for x in test_list:
        xx.append(i)
        yy.append(x.velocity)
        zz.append(x.acceleration)
        i=i+1
    plt.plot(xx, yy, "r")
    plt.plot(xx, zz, "g")
    plt.show()