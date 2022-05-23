"""
reference:
__author__      = "Yingdi Guo"
"""


from velocityplanner import planrelaxvelocity
from qpvelocityplanner_quad import planqpvelocity
import math
import matplotlib.pyplot as plt
import pickle
# todo: need to fix
# from baselines import logger

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
    norm_start_dir = math.sqrt(start_dir[0]**2 + start_dir[1]**2)
    start_dir[0] = start_dir[0] / (norm_start_dir+1e-5)
    start_dir[1] = start_dir[1] / (norm_start_dir+1e-5)
    P1 = [0, 0]
    P2 = [0, 0]
    P1[0] = start_pos[0] + length * control_point_shift * start_dir[0]
    P1[1] = start_pos[1] + length * control_point_shift * start_dir[1]
    P2[0] = goal_pos[0] - length * control_point_shift * goal_dir[0]
    P2[1] = goal_pos[1] - length * control_point_shift * goal_dir[1]
    P0 = start_pos
    P3 = goal_pos
    appro_length = math.sqrt((start_pos[0] - P1[0]) ** 2 + (start_pos[1] - P1[1]) ** 2) + math.sqrt((goal_pos[0] - P2[0]) ** 2 + \
                             (goal_pos[1] - P2[1]) ** 2) + math.sqrt((P1[0] - P2[0]) ** 2 + (P1[1] - P2[1]) ** 2) + \
                   math.sqrt((start_pos[0] - goal_pos[0]) ** 2 + (start_pos[1] - goal_pos[1]) **2 )
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
        point.dir_x = P0[0] * (-3 + 6 * t - 3 * t * t) + P1[0] * (3 - 12 * t + 9 * t * t) + P2[0] * (6 * t - 9 * t * t) + P3[0] * (3 * t * t)
        point.dir_y = P0[1] * (-3 + 6 * t - 3 * t * t) + P1[1] * (3 - 12 * t + 9 * t * t) + P2[1] * (6 * t - 9 * t * t) + P3[1] * (3 * t * t)
        pos_dd_x = P0[0] * (6 - 6 * t) + P1[0] * (-12 + 18 * t) + P2[0] * (6 - 18 * t) + P3[0] * (6 * t)
        pos_dd_y = P0[1] * (6 - 6 * t) + P1[1] * (-12 + 18 * t) + P2[1] * (6 - 18 * t) + P3[1] * (6 * t)
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
        norm_dir = math.sqrt(point.dir_x ** 2 + point.dir_y ** 2)
        point.dir_x = point.dir_x / norm_dir
        point.dir_y = point.dir_y / norm_dir
        list_ret.append(point)
        t = t + dt
    return list_ret


def gen_bpp_with_velocity(goal_pos, goal_dir, start_pos, start_dir, interval, vpurpose, cur_vel, tgt_vel, cur_acc):
    traj_ret = gen_bpp_without_velocity(goal_pos, goal_dir, start_pos, start_dir, interval)
    return planrelaxvelocity(traj_ret, vpurpose, cur_vel, tgt_vel, cur_acc)

def gen_bpp_with_velocity_qp(goal_pos, goal_dir, start_pos, start_dir, interval, cur_vel, tgt_vel, cur_acc):
    traj_ret = gen_bpp_without_velocity(goal_pos, goal_dir, start_pos, start_dir, interval)
    # index=int(len(traj_ret)*0.7)
    index = len(traj_ret)-1
    return planqpvelocity(traj_ret, index, cur_vel, tgt_vel, cur_acc)

def gen_bpp_based_on_refline(start_pos, start_dir, interval, refline):
    bpp_radius=7
    bpp_maximal_len=30
    bpp_extra_dist=1.0
    if (len(refline) == 0):
        return []
    con_i=0
    norm_sdir=math.sqrt(start_dir[0]**2+start_dir[1]**2)
    while con_i<len(refline):
        if (refline[con_i].x < start_pos[0]):
            con_i=con_i+1
            continue
        dir1=[refline[con_i].dir_x, refline[con_i].dir_y]
        norm_dir1=math.sqrt(dir1[0]**2+dir1[1]**2)
        dirm = [refline[con_i].x - start_pos[0], refline[con_i].y - start_pos[1]]
        norm_dirm=math.sqrt(dirm[0]**2+dirm[1]**2)
        cos_da=(dirm[0]*start_dir[0]+dirm[1]*start_dir[1])/(norm_dirm*norm_sdir)
        cos_db=(dirm[0]*dir1[0]+dirm[1]*dir1[1])/(norm_dirm * norm_dir1)
        import numpy as np
        cos_da = np.clip(cos_da, -1., 1.)
        cos_db = np.clip(cos_da, -1., 1.)
        # import pdb; pdb.set_trace()
        theta=math.fabs(math.acos(cos_da))
        theta=theta+math.fabs(math.acos(cos_db))
        e_len=theta*bpp_radius+bpp_extra_dist
        d_pts=[refline[con_i].x-start_pos[0], refline[con_i].y-start_pos[1]]
        dist=math.sqrt(d_pts[0]**2+d_pts[1]**2)
        if (e_len < dist or dist > bpp_maximal_len):
            break
        con_i=con_i+1
    if con_i>=len(refline):
        con_i=len(refline)-1
        dir1 = [refline[con_i].dir_x, refline[con_i].dir_y]
    traj_ret=gen_bpp_without_velocity([refline[con_i].x, refline[con_i].y], dir1, start_pos, start_dir, interval)
    con_i=con_i+1
    while (con_i<len(refline)):
        traj_ret.append(refline[con_i])
        con_i=con_i+1
    return traj_ret

def gen_bpp_based_on_refline_with_velocity_qp(start_pos, start_dir, interval, refline, cur_vel, tgt_vel, cur_acc):
    traj_ret = gen_bpp_based_on_refline(start_pos, start_dir, interval, refline)

    index=int(len(traj_ret)*0.7)
    return planqpvelocity(traj_ret, index, cur_vel, tgt_vel, cur_acc)

# test_list=gen_bpp_with_velocity_qp([60, 0], [1, 0], [0, 0], [1, 0], 0.2, 2.7778, 8.3334, 1.0)
def loadrefline():
    list_r=[]
    pkl_file = open('example.pkl', 'rb')
    ll = pickle.load(pkl_file)
    # [x, y, dir_x, dir_y, theta, velocity, acceleration, curvature]
    refline_pos_x = []
    refline_pos_y = []
    for x in ll:
        a=TrajectoryPoint()
        a.x=x[0]
        a.y=x[1]
        a.dir_x=x[2]
        a.dir_y=x[3]
        a.theta=x[4]
        a.velocity=x[5]
        a.acceleration=x[6]
        a.curvature=x[7]
        list_r.append(a)
        refline_pos_x.append(-x[1])
        refline_pos_y.append(x[0])
    return list_r, refline_pos_x, refline_pos_y

class BppPlanner(object):
    def __init__(self, interval, vel_index):
        self.interval = interval
        self.vel_index = vel_index

    def run_step(self, start_pos, start_dir, refline, cur_vel, tgt_vel, cur_acc):
        return gen_bpp_based_on_refline_with_velocity_qp(start_pos, start_dir, self.interval, refline, cur_vel, tgt_vel, cur_acc, self.vel_index)

if __name__ == '__main__':
    # test_list, refline_pos_x, refline_pos_y=loadrefline()
    # Step 1
    test_list=gen_bpp_with_velocity_qp([5, 5], [1, 0], [0, 0], [1, 0], 0.1, 2.7778, 8.3334, 1.0)
    # Step 2: Step 1 test_list to step 2 test_list is wired. Don't know what it drift for -2 meters    
    # test_list=gen_bpp_based_on_refline_with_velocity_qp([0, 1], [1, 0], 0.2, test_list, 2.7778, 8.3334, 1.0)
    xx=[]
    yy=[]
    for x in test_list:
        xx.append(-x.y)
        yy.append(x.x)
    plt.plot(xx, yy, "r")
    xx=[]
    yy=[]
    zz=[]
    ss=[]
    i=0
    # for x in test_list:
    #     xx.append(i)
    #     yy.append(x.velocity)
    #     zz.append(x.acceleration)
    #     i=i+1
    s_s = 0
    for j in range(0, len(test_list)):
        xx.append(i)
        yy.append(test_list[j].velocity)
        zz.append(test_list[j].acceleration)
        ss.append(s_s)
        if (j < len(test_list) - 1):
            s = (test_list[j + 1].x - test_list[j].x) ** 2 + (test_list[j + 1].y - test_list[j].y) ** 2
            s = math.sqrt(s)
            v_mid = test_list[j + 1].velocity + test_list[j].velocity
            v_mid = v_mid / 2
            t = s / v_mid
            i = i + t
            s_s = s_s + s
    # plt.plot(refline_pos_x, refline_pos_y, "y")
    plt.figure()
    plt.plot(xx, yy, "g")
    plt.figure()
    plt.plot(xx, zz, "b")

    plt.show()
