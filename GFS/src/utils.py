#
# Copyright (c) 2017 Michele Segata <segata@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

import sys
import os
import src.ccparams as cc
import random
import math

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci

# constants for lane change mode
# add a new lane change mode to accelerate the lane change; avoid immediate collision
# AVOID_COL_LC = 0b010101010101
# DEFAULT_LC = 0b011001010101
# DEFAULT_NOTRACI_LC = 0b1010101010
# FIX_LC = 0b0000000000
AVOID_COL_LC =  0b100110010101
# DEFAULT_LC =    0b101010101010
DEFAULT_LC =    0b101001010101
DEFAULT_NOTRACI_LC = 0b1010101010
FIX_LC = 0b0000000000


def set_par(vid, par, value):
    """
    Shorthand for the setParameter method
    :param vid: vehicle id
    :param par: parameter name
    :param value: numeric or string value for the parameter
    """
    traci.vehicle.setParameter(vid, "carFollowModel.%s" % par, str(value))


def get_par(vid, par):
    """
    Shorthand for the getParameter method
    :param vid: vehicle id
    :param par: parameter name
    :return: the parameter value
    """
    return traci.vehicle.getParameter(vid, "carFollowModel.%s" % par)

def get_pos(vid):
    """
        @Author: Yi
        @Date: Dec 9, 2020
        Updated shorthand for the get Parameter method
        :param vid: vehicle ID
        :return: the 2-D coordination of the ego vehicle
    """
    return traci.vehicle.getPosition(vid)

def get_par_new(vid):
    """
    @Author: Yi
    @Date: Dec 9, 2020
    Updated shorthand for the get Parameter method
    :param vid: vehicle ID
    :return: the required parameter values, v, a, u, x, y, t
    """


def change_lane(vid, lane):
    """
    Let a vehicle change lane without respecting any safety distance
    :param vid: vehicle id
    :param lane: lane index
    """
    # traci.vehicle.setLaneChangeMode(vid, FIX_LC)
    traci.vehicle.setLaneChangeMode(vid, DEFAULT_LC)
    traci.vehicle.changeLane(vid, lane, 10000.0)


def add_vehicle(vid, position, lane, speed, cacc_spacing, real_engine=False):
    """
    Adds a vehicle to the simulation
    :param vid: vehicle id to be set
    :param position: position of the vehicle
    :param lane: lane
    :param speed: starting speed
    :param cacc_spacing: spacing to be set for the CACC
    :param real_engine: use the realistic engine model or the first order lag
    model
    """
    # add method has been updated. The original add method is named as "addLegacy" now
    # traci.vehicle.addLegacy(vid, "platoon_route",
    #                   pos=position, speed=speed, lane=lane,
    #                   typeID="vtypeauto")
    traci.vehicle.add(vid, "!flow_0",
                      departPos=str(position), departSpeed=str(speed), departLane=str(lane)) #typeID="vtypeauto","platoon_route",typeID="cacc",
    traci.vehicle.setTau(vid, 0.6)
    # set_par(vid, cc.CC_PAR_CACC_C1, 0.5)s
    # set_par(vid, cc.CC_PAR_CACC_XI, 2)
    # set_par(vid, cc.CC_PAR_CACC_OMEGA_N, 1)
    # set_par(vid, cc.PAR_CACC_SPACING, cacc_spacing)
    # set_par(vid, cc.PAR_CC_DESIRED_SPEED, speed)
    if real_engine:
        set_par(vid, cc.CC_PAR_VEHICLE_ENGINE_MODEL,
                cc.CC_ENGINE_MODEL_REALISTIC)
        set_par(vid, cc.CC_PAR_VEHICLES_FILE, "vehicles.xml")
        set_par(vid, cc.CC_PAR_VEHICLE_MODEL, "alfa-147")
    traci.vehicle.setColor(vid, (random.uniform(0, 255),
                                 random.uniform(0, 255),
                                 random.uniform(0, 255), 255))


def get_distance(v1, v2):
    """
    Returns the distance between two vehicles, removing the length
    :param v1: id of first vehicle
    :param v2: id of the second vehicle
    :return: distance between v1 and v2
    """
    x1, y1 = get_pos(v1)
    x2, y2 = get_pos(v2)
    # v_data = get_par(v1, cc.PAR_SPEED_AND_ACCELERATION)
    # (v, a, u, x1, y1, t) = cc.unpack(v_data)
    # v_data = get_par(v2, cc.PAR_SPEED_AND_ACCELERATION)
    # (v, a, u, x2, y2, t) = cc.unpack(v_data)
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) - 4


def communicate(topology):
    """
    Performs data transfer between vehicles, i.e., fetching data from
    leading and front vehicles to feed the CACC algorithm
    :param topology: a dictionary pointing each vehicle id to its front
    vehicle and platoon leader. each entry of the dictionary is a dictionary
    which includes the keys "leader" and "front"
    """
    for vid, links in topology.items(): # iteritems -> items
        # get data about platoon leader
        leader_data = get_par(links["leader"], cc.PAR_SPEED_AND_ACCELERATION)
        (l_v, l_a, l_u, l_x, l_y, l_t) = cc.unpack(leader_data)
        leader_data = cc.pack(l_v, l_u, l_x, l_y, l_t)
        # get data about front vehicle
        front_data = get_par(links["front"], cc.PAR_SPEED_AND_ACCELERATION)
        (f_v, f_a, f_u, f_x, f_y, f_t) = cc.unpack(front_data)
        front_data = cc.pack(f_v, f_u, f_x, f_y, f_t)
        # pass leader and front vehicle data to CACC
        set_par(vid, cc.PAR_LEADER_SPEED_AND_ACCELERATION, leader_data)
        set_par(vid, cc.PAR_PRECEDING_SPEED_AND_ACCELERATION, front_data)
        # compute GPS distance and pass it to the fake CACC
        f_d = get_distance(vid, links["front"])
        set_par(vid, cc.PAR_LEADER_FAKE_DATA, cc.pack(l_v, l_u))
        set_par(vid, cc.PAR_FRONT_FAKE_DATA, cc.pack(f_v, f_u, f_d))


def start_sumo(config_file, already_running):
    """
    Starts or restarts sumo with the given configuration file
    :param config_file: sumo configuration file
    :param already_running: if set to true then the command simply reloads
    the given config file, otherwise sumo is started from scratch
    """
    arguments = ["-c"]
    sumo_cmd = [sumolib.checkBinary('sumo-gui')]
    arguments.append(config_file)
    arguments.append("--ignore-route-errors")
    arguments.append("True")
    if already_running:
        traci.load(arguments)
    else:
        sumo_cmd.extend(arguments)
        traci.start(sumo_cmd)


def running(demo_mode, step, max_step):
    """
    Returns whether the demo should continue to run or not. If demo_mode is
    set to true, the demo should run indefinitely, so the function returns
    true. Otherwise, the function returns true only if step <= max_step
    :param demo_mode: true if running in demo mode
    :param step: current simulation step
    :param max_step: maximum simulation step
    :return: true if the simulation should continue
    """
    if demo_mode:
        return True
    else:
        return step <= max_step
