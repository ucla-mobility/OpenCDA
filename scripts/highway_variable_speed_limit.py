# -*- coding: utf-8 -*-
"""
Created on Mon Jul 24 11:05:53 2023

@author: hanxu
"""

import io 
import warnings
import scipy.signal

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def draw_multiple_plot(data_list, y_label):
    """
    Draw velocity profiles in a single plot.
    Parameters
    ----------
    velocity_list : list
         The vehicle velocity profile saved in a list.
    """
    
    for i, data in enumerate(data_list):
        x_s = np.arange(len(data)) * 0.05
        if i == 0:
            plt.plot(x_s, data, '--', label="speed limit = 90km/h")
        if i == 1:
            plt.plot(x_s, data, '--', label="speed limit = 120km/h")
        if i == 2:
            plt.plot(x_s, data, '-', label="variable speed limit")
        if i == 3: 
            plt.plot(x_s, data, '--', label="overtake not allowed")
        
    plt.xlabel("Time (s)")
    plt.ylabel(y_label)
    plt.legend(loc="lower right")
    fig = plt.gcf()
    fig.set_size_inches(11, 8.5)

def draw_intersection_plot(data_list, y_label):
    """
    Draw velocity profiles in a single plot.
    Parameters
    ----------
    velocity_list : list
         The vehicle velocity profile saved in a list.
    """
    
    for i, data in enumerate(data_list):
        x_s = np.arange(len(data)) * 0.05
        if i == 0:
            plt.plot(x_s, data, '-', label="right turn on red")
        if i == 1:
            plt.plot(x_s, data, '--', label="no turn on red")
        
    plt.xlabel("Time (s)")
    plt.ylabel(y_label)
    plt.legend(loc="lower right")
    fig = plt.gcf()
    fig.set_size_inches(11, 8.5)

    
def dtg_to_dtd(dtg_90):
    """
    change distance to goal to down track distance 

    """
    total_dist = dtg_90[0]
    total_dist_array = total_dist*np.ones_like(dtg_90)
    return total_dist_array - dtg_90

    
    
##C:/Users/hanxu/OneDrive/Desktop/evaluation_outputs/speed_90_constant_single_2lanefree_carla_2023_07_20_16_08_52

# 1. Varaible speed limit 
file_dir = "C:/Users/hanxu/OneDrive/Desktop/evaluation_outputs/"
speed_90_const_dir = "speed_90_constant_single_2lanefree_carla_2023_07_20_16_08_52/"
speed_120_const_dir = "speed_120_constant_single_2lanefree_carla_2023_07_20_16_09_45/"
variable_speed_dir = "variable_speed_limit_120_90_single_2lanefree_carla_2023_07_20_16_13_02/"

speed_90 = np.loadtxt(file_dir + speed_90_const_dir + "kinematics_speed.csv", delimiter=',')*3.6
acc_90= np.loadtxt(file_dir + speed_90_const_dir + "kinematics_acc.csv", delimiter=',')
dtg_90 =np.loadtxt(file_dir + speed_90_const_dir + "kinematics_dtg.csv", delimiter=',')
dtd_90 = dtg_to_dtd(dtg_90)

speed_120 = np.loadtxt(file_dir + speed_120_const_dir + "kinematics_speed.csv", delimiter=',')*3.6
acc_120= np.loadtxt(file_dir + speed_120_const_dir + "kinematics_acc.csv", delimiter=',')
dtg_120 =np.loadtxt(file_dir + speed_120_const_dir + "kinematics_dtg.csv", delimiter=',')
dtd_120 = dtg_to_dtd(dtg_120)

speed_var = np.loadtxt(file_dir + variable_speed_dir + "kinematics_speed.csv", delimiter=',')*3.6
acc_var= np.loadtxt(file_dir + variable_speed_dir + "kinematics_acc.csv", delimiter=',')
dtg_var =np.loadtxt(file_dir + variable_speed_dir + "kinematics_dtg.csv", delimiter=',')
dtd_var = dtg_to_dtd(dtg_var)

# # draw plots 1: Varaible speed limit
plt.subplot(211)
dtd_list = [dtd_90.tolist(), dtd_120.tolist(), dtd_var.tolist()]
draw_multiple_plot(dtd_list, 'Downtrack Distance (m)')
plt.axvline(x=13, color='g') 
plt.text(14, 600, 'vehicle\napproach\nslow\nspeed limit', fontsize=12)

plt.subplot(212)
speed_list = [speed_90[0:550], speed_120[0:550], speed_var[0:550]]
draw_multiple_plot(speed_list, 'Vehicle Speed (km/h)')
plt.axvline(x=13, color='g') 
plt.text(13.5, 90, 'vehicle\napproach\nslow\nspeed limit', fontsize=12)

# 2. Overtake 
no_overtake_dir = "no_overtake/"
overtake_90_dir = "overtake_90_no_blocksingle_2lanefree_carla_2023_07_20_15_51_33/"
overtake_120_dir = "overtake_120_no_block_single_2lanefree_carla_2023_07_20_15_50_38/"
overtake_varaible = "variable_speed_limit_120_90_single_2lanefree_carla_2023_07_20_16_13_02/"

overtake_90_speed = np.loadtxt(file_dir + overtake_90_dir + "kinematics_speed.csv", delimiter=',')*3.6
overtake_90_dtg =np.loadtxt(file_dir + overtake_90_dir + "kinematics_dtg.csv", delimiter=',')
overtake_90_dtd = dtg_to_dtd(overtake_90_dtg)

overtake_120_speed = np.loadtxt(file_dir + overtake_120_dir + "kinematics_speed.csv", delimiter=',')*3.6
overtake_120_dtg =np.loadtxt(file_dir + overtake_120_dir + "kinematics_dtg.csv", delimiter=',')
overtake_120_dtd = dtg_to_dtd(overtake_120_dtg)

overtake_var_speed = np.loadtxt(file_dir + overtake_varaible + "kinematics_speed.csv", delimiter=',')*3.6
overtake_var_dtg =np.loadtxt(file_dir + overtake_varaible + "kinematics_dtg.csv", delimiter=',')
overtake_var_dtd = dtg_to_dtd(overtake_var_dtg)

no_overtake_speed = np.loadtxt(file_dir + no_overtake_dir + "kinematics_speed.csv", delimiter=',')*3.6
no_overtake_dtg =np.loadtxt(file_dir + no_overtake_dir + "kinematics_dtg.csv", delimiter=',')
no_overtake_dtd = dtg_to_dtd(no_overtake_dtg)

# draw plots 2: Overtake
# plt.subplot(211)
# dtd_list = [overtake_90_dtd.tolist(), overtake_120_dtd.tolist(), 
#             overtake_var_dtd.tolist(), no_overtake_dtd.tolist()]
# draw_multiple_plot(dtd_list, 'Downtrack Distance (m)')
# plt.axvline(x=1, color='r')
# plt.text(1.5, 600, 'vehicle\napproach\nslow\nvehicle', fontsize=12)
# plt.axvline(x=14.5, color='g') 
# plt.text(15, 600, 'vehicle\napproach\nslow\nspeed limit', fontsize=12)


# plt.subplot(212)
# dtd_list = [overtake_90_speed.tolist()[0:600], overtake_120_speed.tolist()[0:600],
#             overtake_var_speed.tolist()[0:600], no_overtake_speed.tolist()[0:600]]
# draw_multiple_plot(dtd_list, 'Vehicle Speed (km/h)')
# plt.axvline(x=0.5, color='r')
# plt.text(1.0, 80, 'vehicle\napproach\nslow\nvehicle', fontsize=12)
# plt.axvline(x=13, color='g') 
# plt.text(13.5, 80, 'vehicle\napproach\nslow\nspeed limit', fontsize=12)



# 3. intersection
right_on_red_dir = "right_turn_on_red_stop_once_single_intersection_town06_carla_2023_07_20_18_07_12/"
no_turn_on_red_dir = "stop_before_turn_right_single_intersection_town06_carla_2023_07_20_17_55_33/"

right_on_red_speed = np.loadtxt(file_dir + right_on_red_dir + "kinematics_speed.csv", delimiter=',')*3.6  
right_on_red_dtg = np.loadtxt(file_dir + right_on_red_dir + "kinematics_dtg.csv", delimiter=',')
right_on_red_dtd = dtg_to_dtd(right_on_red_dtg)

no_turn_on_red_speed = np.loadtxt(file_dir + no_turn_on_red_dir + "kinematics_speed.csv", delimiter=',')*3.6 
no_turn_on_red_dtg = np.loadtxt(file_dir + no_turn_on_red_dir + "kinematics_dtg.csv", delimiter=',')
no_turn_on_red_dtd = dtg_to_dtd(no_turn_on_red_dtg) 

# draw plots 3: Overtake
# plt.subplot(211)
# dtd_list = [right_on_red_dtd.tolist(), no_turn_on_red_dtd.tolist()]
# draw_intersection_plot(dtd_list, 'Downtrack Distance (m)')
# plt.axvline(x=4, color='r')
# plt.text(4.3, 80, 'vehicle\nstop at \nred light', fontsize=12)
# plt.axvline(x=9.5, color='g') 
# plt.text(9.8, 80, 'vehicle proceed \nwith right turn', fontsize=12)
# plt.axvline(x=27, color='g')  
# plt.text(27.3, 80, 'vehicle right turn \nat green light', fontsize=12)

# plt.subplot(212)
# dtd_list = [right_on_red_speed.tolist()[0:700], no_turn_on_red_speed.tolist()[0:700]]
# draw_intersection_plot(dtd_list, 'Vehicle Speed (km/h)')
# plt.axvline(x=4, color='r')
# plt.text(4.3,25, 'vehicle\nstop at \nred light', fontsize=12)
# plt.axvline(x=9.5, color='g') 
# plt.text(9.8, 25, 'vehicle proceed \nwith right turn', fontsize=12)
# plt.axvline(x=27, color='g')  
# plt.text(27.3, 25, 'vehicle right turn \nat green light', fontsize=12)
