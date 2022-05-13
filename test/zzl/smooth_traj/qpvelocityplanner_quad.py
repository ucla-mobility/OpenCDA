"""
reference:
__author__      = "Yingdi Guo, Jiarong Li"
"""

import sys
import os
import string
# todo: need to check if this library needs to install
from qpsolvers import solve_qp
import numpy as np
import math
from velocityplanner import BVPStage
from velocityplanner import applybvpstage

# simple version

eps_H = 1e-2
est_minacc=0.5
# input V_0 a_0 (S_0 = 0), V_tgt (a_tgt = 0) S_tgt = S
# Solution
# 1. segment whole acc/dacc into N=5 parts
# 2. minimize sampled t -> s''(t) + s'''(t)
# 3. equal constrains: V_0 a_0 S_0 V_tgt a_tgt S_tgt V_previous = V_next a_previous = a_next
# 4. inequal constrains: sampled t -> Vmax, t-> s''(t) >= 0 for acceleration or t-> s''(t) <= 0 for deceleration
# V_0, a_0, S_0 = 7.775, 0.8, 0
# init_para = np.array([V_0, a_0])
# S_tgt = 29.4
# V_tgt = 8.333
# des_para = np.array([S_tgt, V_tgt, 0])
degree, num_seg = 5, 5 # degree of polynomials + 1, number of spline segments
# degree * num_seg
T_sampled = 20 # sampled T interval

def ti(t):
    return np.array([0, t, t * t, pow(t, 3), pow(t, 4)])


def dti(t):
    return np.array([0, 1, 2 * t, 3 * t * t, 4 * pow(t, 3)])


def ddti(t):
    return np.array([0, 0, 2, 6 * t, 12 * t * t])


def dddti(t):
    return np.array([0, 0, 0, 6, 24 * t])


def calcspeed(X, V_0, a_0, V_tgt):
    S_tgt = X
    V_mid = V_tgt + V_0
    V_mid = V_mid / 2.0
    t_e = S_tgt / V_mid
    t_est = t_e / num_seg

    t_int = t_est / T_sampled  # time interval after split into time segments
    H = np.zeros((degree * num_seg, degree * num_seg))
    q = np.zeros((degree * num_seg, 1))
    q = q.reshape((degree * num_seg,))

    for i in range(0, num_seg):
        Hi = np.zeros((degree, degree))
        for j in range(1, T_sampled + 1):
            t = j * t_int
            Hi1_slash = 8 * np.array([[1, 3 * t, 6 * t * t],
                                      [3 * t, 9 * t * t, 18 * pow(t, 3)],
                                      [6 * t * t, 18 * pow(t, 3), 36 * pow(t, 4)]])
            #Hi[2:, 2:] += Hi1_slash
            Hi[2:, 2:] = np.array(Hi[2:, 2:].tolist())+Hi1_slash
            Hi2_slash = 72 * np.array([[1, 4 * t],
                                       [4 * t, 16 * t * t]])
            #Hi[3:, 3:] += Hi2_slash
            Hi[3:, 3:] = np.array(Hi[3:, 3:].tolist())+Hi2_slash
        #H[5 * i:5 * (i + 1), 5 * i:5 * (i + 1)] += Hi
        H[5 * i:5 * (i + 1), 5 * i:5 * (i + 1)] = np.array(H[5 * i:5 * (i + 1), 5 * i:5 * (i + 1)].tolist())+Hi
    for i in range(0, degree * num_seg):
        H[i, i] = H[i, i] + eps_H
        # print(Hi)
        # print("\n\n")
    # print(H)
    # print("\n\n")
    # equal
    # sum(S_i) = S 0 <= i < N=5
    # S*1 a_0*1 v_0*1 a_1*1 v_1*1 minus(continues)*2*degree
    init_para = np.array([V_0, a_0])
    A_e1 = np.zeros((4, degree * num_seg))
    A_e1_0 = np.diag([1, 1, 2, 0, 0])
    des_para = np.array([V_tgt, 0])
    b_e1 = np.vstack((init_para, des_para)).reshape(-1, 1)
    #A_e1[:2, :5] += A_e1_0[1:3, :]
    A_e1[:2, :5] = np.array(A_e1[:2, :5].tolist())+np.array(A_e1_0[1:3, :].tolist())

    #A_e1[2:, -5:] += np.array(np.vstack((dti(t_est), ddti(t_est))))
    A_e1[2:, -5:] = np.array(A_e1[2:, -5:].tolist())+np.array(np.vstack((dti(t_est), ddti(t_est))))
    b_e1 = np.vstack((b_e1, np.zeros((4, 1)))).reshape(-1, 1)
    A_e11 = np.zeros((4, degree * num_seg))
    for i in range(1, num_seg):
        #A_e11[i - 1, i * degree:i * degree + num_seg] += ti(0)
        A_e11[i - 1, i * degree:i * degree + num_seg] = np.array(A_e11[i - 1, i * degree:i * degree + num_seg].tolist())+ti(0)
    A_e1 = np.vstack((A_e1, A_e11))
    # print("A_e1\n")
    # print(A_e1)
    # print("b_e1\n")
    # print(b_e1)
    # print("\n\n success")
    dT = np.zeros((num_seg - 1, degree * num_seg))
    ddT = np.zeros((num_seg - 1, degree * num_seg))
    dddT = np.zeros((num_seg - 1, degree * num_seg))
    for i in range(0, num_seg - 1):
        # t_cur = t_est * (i + 1)
        t_cur = t_est
        dT[i, degree * (i + 1) + 1] = -1
        #dT[i, degree * i:degree * (i + 1)] += dti(t_cur)
        dT[i, degree * i:degree * (i + 1)] = np.array(dT[i, degree * i:degree * (i + 1)].tolist())+dti(t_cur)
        ddT[i, degree * (i + 1) + 2] = -2
        #ddT[i, degree * i:degree * (i + 1)] += ddti(t_cur)
        ddT[i, degree * i:degree * (i + 1)] = np.array(ddT[i, degree * i:degree * (i + 1)].tolist())+ddti(t_cur)
        dddT[i, degree * (i + 1) + 3] = -6
        #dddT[i, degree * i:degree * (i + 1)] += dddti(t_cur)
        dddT[i, degree * i:degree * (i + 1)] = np.array(dddT[i, degree * i:degree * (i + 1)].tolist())+dddti(t_cur)
    # print("dt\n")
    # print(dT)
    # print("\nddt\n")
    # print(ddT)
    A_e2 = np.vstack((dT, ddT, dddT))
    b_e2 = np.zeros((3 * num_seg - 3, 1))
    A_e3 = np.array([ti(t_est).tolist() for i in range(0, num_seg)]).reshape((-1,))
    b_e3 = np.array([S_tgt])
    A_eq = np.vstack((A_e1, A_e2, A_e3))
    b_eq = np.vstack((b_e1, b_e2, b_e3)).reshape((-1,))
    # print(H.shape)
    # print(A_eq.shape)
    # print(b_eq.shape)
    A_ueq = np.zeros((num_seg * T_sampled, num_seg * degree))
    for i in range(0, num_seg):
        for j in range(1, T_sampled + 1):
            t = j * t_int
            #A_ueq[i * T_sampled + j - 1, i * degree:(i + 1) * degree] += ddti(t)
            A_ueq[i * T_sampled + j - 1, i * degree:(i + 1) * degree] = np.array(A_ueq[i * T_sampled + j - 1, i * degree:(i + 1) * degree].tolist())+ddti(t)
    A_ueq *= -1 if V_tgt > V_0 else 1
    b_ueq = np.zeros((num_seg * T_sampled,))
    # print(A_ueq.shape)
    # print(b_ueq.shape)
    init_gp = {'gp': True}
    try:
        # sols = solve_qp(H, q, A_ueq, b_ueq, A_eq, b_eq, solver="ecos", initvals=init_gp)
        sols = solve_qp(H, q, A_ueq, b_ueq, A_eq, b_eq, solver="ecos")
    except:
        sols = []
    # print("sols:", sols)
    # print("t_est:", t_est)
    return sols, t_est

def planqpvelocity(traj_in, index, cur_velocity, tgt_velocity, cur_acc):
    if (index >= len(traj_in)):
        index = len(traj_in) - 1
    if index < 0:
        return
    S_tgt = 0
    for i in range(0, index + 1):
        norm_now = 0
        if (i==0):
            norm_now = math.sqrt(traj_in[i].x ** 2 + traj_in[i].y ** 2)
        else:
            norm_now = math.sqrt((traj_in[i].x - traj_in[i-1].x)**2 + (traj_in[i].y - traj_in[i-1].y)**2)
        S_tgt = S_tgt + norm_now
        traj_in[i].sumdistance=S_tgt
    vm = tgt_velocity - cur_velocity
    if (vm < 0 and cur_acc > 0):
        cur_acc = 0
    elif (vm > 0 and cur_acc < 0):
        cur_acc = 0 
    est_acc = cur_acc
    if (math.fabs(cur_acc) < est_minacc / 2.0):
        est_acc = est_minacc / 2.0
    if (vm < 0):
        est_acc = -math.fabs(est_acc)
    eS = (tgt_velocity ** 2 - cur_velocity ** 2) / 2 / est_acc
    S_tgt_bakup = S_tgt
    if (eS < S_tgt):
        S_tgt = eS
    print("S_tgt",S_tgt)
    coeff, t_est = calcspeed(S_tgt, cur_velocity, cur_acc, tgt_velocity)
    
    if (len(coeff) == 0):
        # Fallback
        print("Fall back")
        bvp=BVPStage()
        a = (tgt_velocity ** 2 - cur_velocity ** 2) / 2 / S_tgt_bakup
        bvp.j = 0
        bvp.s_a = a
        bvp.e_a = a
        bvp.d_f_s = S_tgt_bakup
        bvp.s_v = cur_velocity
        bvp.e_v = tgt_velocity
        bvp.type = 1
        bvs=[bvp]
        return applybvpstage(traj_in, bvs)
    SS=[]
    for i in range(0, num_seg):
        n_s=coeff[1+i*5]*t_est+coeff[2+i*5]*(t_est**2)+coeff[3+i*5]*(t_est**3)+coeff[4+i*5]*(t_est**4)
        if (i>0):
            n_s=n_s+SS[i-1]
        SS.append(n_s)
    ii=0
    tn=0
    i=0
    while i < len(traj_in):
        while (ii < num_seg and traj_in[i].sumdistance>SS[ii]):
            ii=ii+1
            tn=0
        if (ii>=num_seg):
            break
        s=traj_in[i].sumdistance
        if (ii>0):
            s=s-SS[ii-1]
        a=coeff[ii*5+4]
        b=coeff[ii*5+3]
        c=coeff[ii*5+2]
        d=coeff[ii*5+1]
        e=-s
        cand_t = np.roots([a, b, c, d, e])
        tmp_nowt = -1
        for j in range(len(cand_t)):
            if np.isreal(cand_t[j]):
                tmp_tn = float(np.real(cand_t[j]))
                if (tmp_tn >= 0 and (tmp_tn - tn >= 0 or (math.fabs(tmp_tn - tn) < 1e-5)) and (
                        tmp_nowt < 0 or tmp_nowt - tn > tmp_tn - tn)):
                    tmp_nowt = tmp_tn
        if (tmp_nowt > -0.5):
            tn=tmp_nowt
            traj_in[i].velocity=4*a*(tn**3)+3*b*(tn**2)+2*c*tn+d
            traj_in[i].acceleration=12*a*(tn**2)+6*b*tn+2*c
        else:
            if (i>0):
                traj_in[i].velocity = traj_in[i-1].velocity
                traj_in[i].acceleration = traj_in[i-1].acceleration
            else:
                traj_in[i].velocity = cur_velocity
                traj_in[i].acceleration = cur_acc
        i=i+1
    while i < len(traj_in):
        traj_in[i].velocity=tgt_velocity
        traj_in[i].acceleration=0
        i=i+1
    return traj_in

if __name__ == "__main__":
    pass
    # coeff, t_est = calcspeed(S_tgt, cur_velocity, cur_acc, tgt_velocity)