# -*- coding: utf-8 -*-
"""
Obstacle vehicle class to save object detection.
"""

# Author: XH
# License: TDG-Attribution-NonCommercial-NoDistrib


import socket
import json
import csv
import binascii as ba
import math, sys
import numpy as np
import socket
import time
import readline
#SAE J275 DECODER
from opencda.message_decoders import J2735_201603_combined_voices_mr_fix as J2735

lightStatusDict = {'protected-Movement-Allowed': 'green',
                   'permissive-Movement-Allowed': 'green',
                   'permissive-clearance': 'yellow',
                   'protected-clearance': 'yellow',
                   'caution-Conflicting-Traffic': 'yellow',
                   'stop-Then-Proceed': 'red',
                   'stop-And-Remain': 'red'}

class J2735Manager(object):
    """
    A class for handeling J2735 messages.

    Parameters
    ----------
    corners : nd.nparray
        Eight corners of the bounding box. shape:(8, 3).

    Attributes
    ----------
    bounding_box : BoundingBox
        Bounding box of the osbject vehicle.

    """

    def __init__(self, UDP_IP, UDP_PORT, save_dir=None):
        '''
        Note: if directory is presented, save the decoded msg.
        '''
        self.is_print = is_print
        if save_dir:
            self.is_save = True
            self.save_dir = save_dir
        else:
            self.is_save = False
 
        self.hex_data = None
        self.SPaT_flag = False
        self.SPaT_data = None

        # IP ports (needs to match VOICES config files)
        self.UDP_IP = UDP_IP
        self.UDP_PORT = UDP_PORT
        
        # connect to message ports
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.UDP_IP, self.UDP_PORT))

    def get_spat_flag(self):
        return self.SPaT_flag

    def get_spat_data(self):
        return self.SPaT_data

    def get_hex_data(self):
        return self.hex_data

    def decode_hex_data(self, j2735_hex):
        # place holder function
        try:
            # specify message type inside J2735.py
            decoded_msg = J2735.DSRC.MessageFrame
            # convert from hex using unhexlify then from uper using library
            decoded_msg.from_uper(ba.unhexlify(j2735_hex))
            # format data into json
            # decoded_msg_json = decoded_msg.to_json()

            print('')
            # print(decoded_msg_json)
        except Exception as err:
            print(f"Unexpected {err}, {type(err)}")
            raise
        return decoded_msg

    def convert_time_format(self, meta_J2735_intersection, \
                            reference_timestamp, minEndTime):
        # convert time format
        spatTimestamp = meta_J2735_intersection['timeStamp']
        moy = meta_J2735_intersection['moy']
        timeOfDayInMin = moy % 1440
        hour = timeOfDayInMin//60
        minute = timeOfDayInMin % 60
        seconds = spatTimestamp/1000
        currentTime = '{}:{}:{}'.format(hour, minute, seconds)
        currentTimestamp = datetime.strptime(currentTime, '%H:%M:%S.%f')
        # convert current time
        print('#####Current Time: ', currentTimestamp)
        currentTimeReference = \
            (currentTimestamp.hour * 3600 + currentTimestamp.minute * 60 + currentTimestamp.second) - \
            ((reference_timestamp.hour * 60 + reference_timestamp.minute) * 60 + reference_timestamp.second)
        
        # reformat min end time
        minEndTimeSecond = round(float(minEndTime / 10 - minute * 60), 3)
        if minEndTimeSecond < 60:
            print('{}:{}:{}'.format(hour, minute, minEndTimeSecond))
            minEndTimeStamp = datetime.strptime(
                '{}:{}:{:.3f}'.format(hour, minute, minEndTimeSecond), '%H:%M:%S.%f')
        else:
            minEndTimeStamp = datetime.strptime(
                '{}:{}:{:.3f}'.format(hour, (minute + int(minEndTimeSecond // 60)), \
                                      minEndTimeSecond % 60), '%H:%M:%S.%f')
        
        return currentTimeReference, minEndTimeStamp

    def getSpatWindow(self, j2735_hex, reference_timestamp, \
                      intersectin_index, phase_index, green_duration, red_duration):
        """
        Covnert meta data in json to spat info. 
        Note: Specific traffic light phase is needed, as phase contains the plan 
            for all forcorners. 
        :param 
            j2735_hex: J2735 metadata as hex string
            reference_timestamp: current timestamp in hour-min-sec format.
            intersectin_index: the target intersection id among the entire map.
            phase_index: the corresponding phase index for the ego vehicle.
            green_duration 
            red_duration

        :return: dict
            'status':
            't1s':
            't1e':
            't2s':
            't2e':
            'r1s':
        """
        # 1. Decode J2735 meta data  
        decoded_msg = decode_hex_data(j2735_hex)
        meta_J2735_msg = decoded_msg()['value'][1]
        meta_J2735_intersection = meta_J2735_msg['intersections'][intersectin_index]
        
        try:
            intersectionName = meta_J2735_intersection['name']
            intersectionID = meta_J2735_intersection['id']['id']
        except:
            intersectionName = ""
            intersectionID = ""

        # extract phase array
        instersectionPhaseArray = meta_J2735_intersection['states']
        print('Note: The intersection %s has a totle of %i phases.' %(intersectionID, len(instersectionPhaseArray)))
        # print('Note: Intersection Phase array: ' + str(instersectionPhaseArray))
        
        # 2. Extract meta spat data for the intersection 
        spatPhaseDict = {}
        phaseIndexArray = []
        # calculate detailed plan for all phases
        for phase in range(len(instersectionPhaseArray)):
            # record all phase indexes
            phaseIndexArray.append(phase)
            # read phase details
            currentPhase = meta_J2735_intersection['states'][phase].get('signalGroup')
            currentState = str(meta_J2735_intersection['states'][phase]['state-time-speed'][0]['eventState'])
            try:
                minEndTime = meta_J2735_intersection['states'][phase]['state-time-speed'][0]['timing']['minEndTime']
            except:
                minEndTime = None

            try:
                maxEndTime = meta_J2735_intersection['states'][phase]['state-time-speed'][0]['timing']['maxEndTime']
            except:
                maxEndTime = None

            phaseState = dict({'state': currentState, 'minEndTime': minEndTime, 'maxEndTime': maxEndTime})
            spatPhaseDict[currentPhase] = phaseState

        # 3. Calculate residual time based on relavent phase and current time 
        # convert phase to red, green, yellow
        phaseState = spatPhaseDict[phase_index]
        lightStatus = lightStatusDict[phaseState['state']]

        # time conversion 
        minEndTime = phaseState['minEndTime']
        currentTimeReference, minEndTimeStamp = convert_time_format(meta_J2735_intersection, reference_timestamp, minEndTime)

        if lightStatus == 'green' or lightStatus == 'yellow':
            t1e = (minEndTimeStamp.hour * 3600 + minEndTimeStamp.minute * 60 + minEndTimeStamp.second) - \
                               ((reference_timestamp.hour * 60 + reference_timestamp.minute) * 60 + reference_timestamp.second)
            t1s = currentTimeReference
            r1s = t1e
        else:
            t1s = (minEndTimeStamp.hour * 3600 + minEndTimeStamp.minute * 60 + minEndTimeStamp.second) - \
                               ((reference_timestamp.hour * 60 + reference_timestamp.minute) * 60 + reference_timestamp.second)
            t1e = t1s + green_duration
            r1s = currentTimeReference

        t2s = t1e + red_duration
        t2e = t2s + green_duration

        spatWindow = {'current Time': currentTimeReference, 'light status': lightStatus, 't1s': t1s, 't1e': t1e, 't2s': t2s, 't2e': t2e, 'r1s': r1s}
        #print(spatWindow)

        return spatWindow

    def process_SPaT(hex_data):
        '''
        '''
        reference_timestamp = datetime.strptime('06:30:00', '%H:%M:%S')

        if hex_data.startswith("0013"):
            print("Received SPaT")
            spatWindown = getSpatWindow(hex_data, 
                                      reference_timestamp, 
                                      intersectin_index=0,
                                      phase_index=2,
                                      green_duration=40, 
                                      red_duration=30)
            return True, spatWindown
        else:
            return False, {}

    def run_step(self):
        # get spat data at each step
        data, addr = sock.recvfrom(4096)
        self.hex_data = data.hex()
        # print('Raw hex data is: ' + str(hex_data))
        self.SPaT_flag, self.SPaT_data = process_SPaT(hex_data)
        if SPaT_data != {}:
            print('SPaT data is: ' + str(SPaT_data))