"""
reference:
__author__      = "Yingdi Guo"
"""


import math
import numpy

# ---- conf ----
max_jerk = 0.9
max_acc = 2.0
speed_eps = 0.27
thres_keep_distance = 2.0
# --------------

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
        
class BVPStage:
    def __init__(self):
        # type:
        #   0:
        #   1: const acc
        #   2: acc up
        #   3: acc down
        self.type = 0
        # s_v: start velocity
        self.s_v = 0
        # e_v: end velocity
        self.e_v = 0
        # s_a: start acceleration
        self.s_a = 0
        # e_a: end acceleration
        self.e_a = 0
        # d_f_s: stage start distance from start point
        self.d_f_s = 0
        # jerk
        self.j = 0

# purpose:
#   0: stop at the end of `traj_in`
#   1: non zero velocity change or reach the target speed at max acc max jerk
def planrelaxvelocity(traj_in, purpose, cur_velocity, tgt_velocity, cur_acc):
    vm = tgt_velocity - cur_velocity
    if math.fabs(cur_velocity - tgt_velocity) < speed_eps:
        for i in range(0, len(traj_in)):
            traj_in[i].velocity = tgt_velocity
            traj_in[i].acceleration = 0
        return traj_in
    if (vm > 0 and cur_acc < 0) or (vm < 0 and cur_acc > 0):
        cur_acc = 0
    jerk = max_jerk
    d_acc_down_ok = 0
    tt = s_type1 = v_exp = v_cha = 0
    if (cur_acc > 0):
        jerk = -jerk
    if (math.fabs(cur_acc) > 1e-3):
        tt = -cur_acc / jerk
        v_exp = cur_acc * tt + 0.5 * jerk * tt * tt + cur_velocity
        v_cha = v_exp - tgt_velocity
        if ((v_cha < 0 and vm > 0) or (v_cha > 0 and vm < 0)):
            d_acc_down_ok = 0
        else:
            d_acc_down_ok = 1
            d_acc_down_jerk = -cur_acc * cur_acc / (tgt_velocity - cur_velocity) / 2
            tt = -cur_acc / d_acc_down_jerk
            s_type1 = tt * tt * tt * d_acc_down_jerk / 6 + cur_acc * tt * tt / 2 + tt * cur_velocity
    d_onetime_accupdown = 0
    d_maxacc_apply = 0
    if (tgt_velocity > cur_velocity):
        jerk = max_jerk
    else:
        jerk = -max_jerk
    abs_a1 = a1 = s_type2 = stype_2_1 = stype2_2 = v1_s = 0
    abs_a1 = math.sqrt(jerk * (tgt_velocity - cur_velocity) + cur_acc * cur_acc / 2)
    if (math.fabs(cur_acc) < abs_a1 + 1e-3 and abs_a1 < max_acc + 1e-3):
        d_onetime_accupdown = 1
        if (jerk > 0):
            a1 = abs_a1
        else:
            a1 = -abs_a1
        tt = (a1 - cur_acc) / jerk
        v1_s = (-cur_acc * cur_acc / 2 + a1 * a1 / 2) / jerk + cur_velocity
        s_type2_1 = tt * tt * tt * jerk / 6 + cur_acc * tt * tt / 2 + tt * cur_velocity
        tt = a1 / jerk
        s_type2_2 = -tt * tt * tt * jerk / 6 + a1 * tt * tt / 2 + tt  * v1_s
        s_type2 = s_type2_1 + s_type2_2
    else:
        d_onetime_accupdown = 0
        if (abs_a1 > max_acc):
            d_maxacc_apply = 1

    v2_s = acc_max = t1 = t2 = t3 = s_type3_1 = s_type3_2 = s_type3_3 = s_type3 = 0
    maxacc_current = cur_acc
    if (d_maxacc_apply == 1):
        if (tgt_velocity > cur_velocity):
            jerk = max_jerk
            acc_max = max_acc
        else:
            jerk = -max_jerk
            acc_max = -max_acc
        if (math.fabs(maxacc_current) > math.fabs(acc_max)):
            maxacc_current = acc_max
        t1 = (acc_max - maxacc_current) / jerk
        v1_s = jerk * t1 * t1 * 0.5 + maxacc_current * t1 + cur_velocity
        t3 = acc_max / jerk
        v2_s = tgt_velocity - acc_max * t3 / 2
        t2 = (v2_s - v1_s) / acc_max
        s_type3_1 = t1 * t1 * t1 * jerk / 6 + maxacc_current * t1 * t1 / 2 + t1 * cur_velocity
        s_type3_2 = v1_s * t2 + 0.5 * acc_max * t2 * t2
        s_type3_3 = t3 * (-jerk * t3 * t3 / 6 + t3 * acc_max / 2 + v2_s)
        s_type3 = s_type3_1 + s_type3_2 + s_type3_3

    sum_dist = 0
    for i in range(0, len(traj_in) - 1):
        aa = math.sqrt((traj_in[i].x - traj_in[i + 1].x) ** 2 + (traj_in[i].y - traj_in[i + 1].y) ** 2)
        sum_dist = sum_dist + aa

    stop_jerk = s_ss1_1 = s_ss1_2 = s_ss1 = a_ssf = 0
    use_fallback_stop = 0
    stop_jerk = max_jerk
    v_ss1_1 = cur_acc * cur_acc / 2 / stop_jerk
    keep_current_acceleration = 0
    if (v_ss1_1 > cur_velocity or (math.fabs(cur_acc) < 1e-5)):
        use_fallback_stop = 1
    else:
        t1 = v_ss1_1 - cur_velocity
        t1 = t1 / cur_acc
        s_ss1_1 = cur_acc * t1 * t1 * 0.5 + cur_velocity *  41
        t2 = -cur_acc / stop_jerk
        s_ss1_2 = t2 * (stop_jerk * 1.0 / 6 * t2 * t2 + 0.5 * t2 * cur_acc + v_ss1_1)
        s_ss1 = s_ss1_1 + s_ss1_2
        if (math.fabs(sum_dist - s_ss1) < thres_keep_distance):
            keep_current_acceleration = 1
        else:
            use_fallback_stop = 1

    if (use_fallback_stop == 1):
        a_ssf = -cur_velocity * cur_velocity / (2.0 * sum_dist)

    bvs=[]
    if (purpose == 1):
        if (d_acc_down_ok == 1):
            ba = BVPStage()
            ba.s_v = cur_velocity
            ba.e_v = tgt_velocity
            ba.s_a = cur_acc
            ba.e_a = 0
            ba.type = 3
            ba.j = d_acc_down_jerk
            # if (cur_acc > 0):
            #     ba.j = -max_jerk
            # else:
            #     ba.j = max_jerk
            ba.d_f_s = s_type1
            bvs.append(ba)
        elif (d_onetime_accupdown == 1):
            ba = BVPStage()
            bb = BVPStage()
            ba.s_a = cur_acc
            ba.e_a = a1
            if (tgt_velocity > cur_velocity):
                ba.j = max_jerk
            else:
                ba.j = -max_jerk
            ba.d_f_s = s_type2_1
            ba.s_v = cur_velocity
            ba.e_v = v1_s
            ba.type = 2
            bvs.append(ba)
            bb.s_a = a1
            bb.e_a = 0
            bb.s_v = v1_s
            bb.e_v = tgt_velocity
            bb.d_f_s = s_type2
            bb.type = 3
            bb.j = -ba.j
            bvs.append(bb)
        elif (d_maxacc_apply):
            ba = BVPStage()
            bb = BVPStage()
            bc = BVPStage()
            if (tgt_velocity > cur_velocity):
                ba.j = max_jerk
                acc_max = max_acc
            else:
                ba.j = -max_jerk
                acc_max = -max_acc
            ba.s_a = maxacc_current
            ba.e_a = acc_max
            ba.s_v = cur_velocity
            ba.e_v = v1_s
            ba.d_f_s = s_type3_1
            ba.type = 2
            bvs.append(ba)
            bb.s_v = v1_s
            bb.e_v = v2_s
            bb.s_a = acc_max
            bb.e_a = acc_max
            bb.j = 0
            bb.d_f_s = s_type3_1 + s_type3_2
            bb.type = 1
            bvs.append(bb)
            bc.s_v = v2_s
            bc.e_v = tgt_velocity
            bc.s_a = acc_max
            bc.e_a = 0
            bc.d_f_s = s_type3
            bc.j = -ba.j
            bc.type = 3
            bvs.append(bc)
        else:
            print("Final fallback, directly use target_speed_")
            for i in range(0, len(traj_in)):
                traj_in[i].velocity = tgt_velocity
                traj_in[i].acceleration = 0
            return traj_in
    elif (purpose == 0):
        if (keep_current_acceleration == 1):
            ba=BVPStage()
            bb=BVPStage()
            ba.s_a = cur_acc
            ba.type = 1
            ba.e_a = cur_acc
            ba.d_f_s = s_ss1_1
            ba.s_v = cur_velocity
            ba.e_v = v_ss1_1
            ba.j = 0
            bvs.append(ba)
            bb.s_v = v_ss1_1
            bb.e_v = 0
            bb.d_f_s = s_ss1
            bb.s_a = cur_acc
            bb.e_a = 0
            bb.j = stop_jerk
            bb.type = 3
            bvs.append(bb)
        elif(use_fallback_stop == 1):
            ba = BVPStage()
            ba.s_a = a_ssf
            ba.e_a = a_ssf
            ba.s_v = cur_velocity
            ba.e_v = 0
            ba.j = 0
            ba.type = 1
            ba.d_f_s = sum_dist
            bvs.append(ba)
    return applybvpstage(traj_in, bvs)


def applybvpstage(traj_in, bvs):
    if (len(bvs) == 0):
        return traj_in
    accumulate_sum_distance = 0
    current_stage = -1
    early_stage = 0
    tn = 0
    for i in range(0, len(traj_in)):
        norm_now = 0
        if (i==0):
            norm_now = math.sqrt(traj_in[i].x ** 2 + traj_in[i].y ** 2)
        else:
            norm_now = math.sqrt((traj_in[i].x - traj_in[i-1].x)**2 + (traj_in[i].y - traj_in[i-1].y)**2)
        d = accumulate_sum_distance + norm_now
        accumulate_sum_distance = accumulate_sum_distance + norm_now
        while (current_stage == -1 or (current_stage < len(bvs) and d > bvs[current_stage].d_f_s)):
            early_stage = current_stage
            current_stage = current_stage + 1
            tn = 0
        use_goal_directly = 0
        if (current_stage >= len(bvs) or d > bvs[current_stage].d_f_s):
            use_goal_directly = 1
        if (early_stage >= 0):
            d = d - bvs[early_stage].d_f_s
        if (current_stage >= 0 and current_stage < len(bvs) and use_goal_directly == 0):
            a = 0
            b = 0
            c = 0
            if (bvs[current_stage].type == 1):
                a = 0
                b = bvs[current_stage].s_a / 2.0
                c = bvs[current_stage].s_v
            elif (bvs[current_stage].type == 2 or bvs[current_stage].type == 3):
                a = bvs[current_stage].j / 6.0
                b = bvs[current_stage].s_a / 2.0
                c = bvs[current_stage].s_v
            cand_t = numpy.roots([a, b, c, -d])
            tmp_nowt = -1
            for j in range(len(cand_t)):
                if numpy.isreal(cand_t[j]):
                    tmp_tn = float(numpy.real(cand_t[j]))
                    if (tmp_tn >= 0 and (tmp_tn - tn >= 0 or (math.fabs(tmp_tn - tn) < 1e-5)) and (tmp_nowt < 0 or tmp_nowt - tn > tmp_tn - tn)):
                        tmp_nowt = tmp_tn
            if (tmp_nowt >= 0):
                traj_in[i].velocity = a * 3.0 * tmp_nowt * tmp_nowt + b * 2 * tmp_nowt + bvs[current_stage].s_v
                traj_in[i].acceleration = a * 6 * tmp_nowt + b * 2
                tn = tmp_nowt
            else:
                if (i > 0):
                    traj_in[i].velocity = traj_in[i - 1].velocity
                    traj_in[i].acceleration = 0
                else:
                    traj_in[i].velocity = bvs[current_stage].s_v
                    traj_in[i].acceleration = bvs[current_stage].s_a

        else:
            traj_in[i].velocity = bvs[early_stage].e_v
            traj_in[i].acceleration = bvs[early_stage].e_a

    return traj_in