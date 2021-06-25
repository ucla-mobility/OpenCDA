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

# active controller
DRIVER = 0
ACC = 1
CACC = 2
FAKED_CACC = 3
PLOEG = 4
CONSENSUS = 5

CC_ENGINE_MODEL_FOLM = 0x00
CC_ENGINE_MODEL_REALISTIC = 0x01

ENGINE_PAR_VEHICLE = "vehicle"
ENGINE_PAR_XMLFILE = "xmlFile"
ENGINE_PAR_DT = "dt_s"

CC_PAR_VEHICLE_DATA = "ccvd"
CC_PAR_VEHICLE_POSITION = "ccvp"
CC_PAR_PLATOON_SIZE = "ccps"

CC_PAR_CACC_XI = "ccxi"
CC_PAR_CACC_OMEGA_N = "ccon"
CC_PAR_CACC_C1 = "ccc1"
CC_PAR_ENGINE_TAU = "cctau"

CC_PAR_PLOEG_H = "ccph"
CC_PAR_PLOEG_KP = "ccpkp"
CC_PAR_PLOEG_KD = "ccpkd"

CC_PAR_VEHICLE_ENGINE_MODEL = "ccem"

CC_PAR_VEHICLE_MODEL = "ccvm"
CC_PAR_VEHICLES_FILE = "ccvf"

PAR_CACC_SPACING = "ccsp"
PAR_ACC_ACCELERATION = "ccacc"
PAR_CRASHED = "cccr"
PAR_FIXED_ACCELERATION = "ccfa"
PAR_SPEED_AND_ACCELERATION = "ccsa"
PAR_LEADER_SPEED_AND_ACCELERATION = "cclsa"
PAR_LANES_COUNT = "cclc"
PAR_CC_DESIRED_SPEED = "ccds"
PAR_ACTIVE_CONTROLLER = "ccac"
PAR_RADAR_DATA = "ccrd"
PAR_LEADER_FAKE_DATA = "cclfd"
PAR_FRONT_FAKE_DATA = "ccffd"
PAR_DISTANCE_TO_END = "ccdte"
PAR_DISTANCE_FROM_BEGIN = "ccdfb"
PAR_PRECEDING_SPEED_AND_ACCELERATION = "ccpsa"
PAR_ACC_HEADWAY_TIME = "ccaht"
PAR_ENGINE_DATA = "cced"

SEP = ':'
ESC = '\\'
QUO = '"'


def pack(*args):
    a = []
    for arg in args:
        esc = str(arg).replace(ESC, ESC + ESC)
        esc = esc.replace(SEP, ESC + SEP)
        if esc == "" or (esc[0] == QUO and esc[-1] == QUO):
            esc = QUO + esc + QUO
        a.append(esc)
    return SEP.join(a)


def _next(string):
    sep = -1
    while True:
        sep = string.find(SEP, sep + 1)
        if sep == -1 or sep == 0 or string[(sep-1):sep] != ESC:
            break

    if sep == -1:
        next_string = string
        string = ""
    else:
        next_string = string[0:sep]
        string = string[(sep+1):]
    return next_string, string


def unpack(string):
    ret = list()
    while True:
        value, string = _next(string)
        if value == "" and string == "":
            break
        value = value.replace(ESC + ESC, ESC)
        value = value.replace(ESC + SEP, SEP)
        if value[0] == QUO and value[-1] == QUO:
            value = value[1:-1]
        try:
            ret.append(int(value))
        except ValueError:
            try:
                ret.append(float(value))
            except ValueError:
                ret.append(value)
    return ret
