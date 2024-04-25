# -*- coding: utf-8 -*-
"""
Scenario testing: single vehicle behavior in intersection
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla
import pygame
import sys
import json
import csv
import binascii as ba
import math, sys
import numpy as np
import socket
import time
from datetime import datetime

import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time
# J2735 parser
from opencda.message_decoders.J2735_parser import process_SPaT


def run_scenario(opt, scenario_params):
    try:
        scenario_params = add_current_time(scenario_params)

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)

        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   # use cav world for VOICES tests and mcity map for local test
                                                   # town='mcity_map_v2',
                                                   cav_world=cav_world)

        if opt.record:
            scenario_manager.client. \
                start_recorder("mcity_local_test.log", True)

        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'])

        # # create background traffic in carla
        # traffic_manager, bg_veh_list = \
        #     scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='mcity_local_test',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()

        # ------------- space key press event -------------
        world = scenario_manager.client.get_world()

        print("Press SPACE key to start the vehicle")
        running = False

        # Set up the Pygame window and clock
        pygame.init()

        screen = pygame.display.set_mode((700, 100))
        # Set the font and text for the message
        font = pygame.font.SysFont("monospace", 30)
        text = font.render("Press SPACE to start vehicle movement", True, (255, 255, 255))

        # Draw the message on the screen
        screen.blit(text, (10, 10))
        pygame.display.flip()

        clock = pygame.time.Clock()

        # connect to J2735 adapter
        UDP_IP = "192.168.2.6"
        #UDP_IP = os.getenv("VUG_LOCAL_ADDRESS")  #### VPN testing to get SPaT
        UDP_PORT = 5398

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))

        # # -------------------------------------------------
        # # Stationary view for VOICES
        # transform = single_cav_list[0].vehicle.get_transform()
        # spectator.set_transform(carla.Transform(
        #     transform.location +
        #     carla.Location(
        #         z=70),
        #     carla.Rotation(
        #         pitch=-
        #         90)))

        # run steps
        while True:
            scenario_manager.tick()
            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=70),
                carla.Rotation(
                    pitch=-
                    90)))

            # pygame event 
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                    running = True
                    
            # print spat befor engaging 
            # check target trafficl light state 
            data, addr = sock.recvfrom(4096)
            hex_data = data.hex()
            SPaT_flag, spatInfo = process_SPaT(hex_data)
            # print SPaT
            if spatInfo != {}:
                # print spat data
                print('SPaT data is: ' + str(spatInfo))

            # continue when key pressed 
            if running == True:
                # iterate vehicle control
                for i, single_cav in enumerate(single_cav_list):
                    # send SPaT to eco drive manager here for eco approach
                    if spatInfo != {}:
                        single_cav.agent.set_spat_info(spatInfo)
                    # vehicle control
                    single_cav.update_info()
                    control = single_cav.run_step()
                    single_cav.vehicle.apply_control(control)

            else:
                # brake the vehicle to prevent roll back
                for i, single_cav in enumerate(single_cav_list):
                    single_cav.update_info()
                    control = carla.VehicleControl(brake=1.0)
                    single_cav.vehicle.apply_control(control)

    finally:
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in single_cav_list:
            v.destroy()
        for v in bg_veh_list:
            v.destroy()

