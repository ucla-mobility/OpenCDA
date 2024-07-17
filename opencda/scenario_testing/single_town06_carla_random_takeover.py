# -*- coding: utf-8 -*-
"""
Scenario testing: single vehicle behavior in intersection
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import random
import re
import sys
import json


import carla
import pygame
from carla import ColorConverter as cc
from pygame.locals import KMOD_CTRL
from pygame.locals import K_ESCAPE
from pygame.locals import K_q

from opencda.version import __version__
import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api

from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import \
    add_current_time

# pygame render
from opencda.core.common.pygame_render import World, HUD, KeyboardControl

# multi-processing
from multiprocessing import Process, Queue, get_context
import multiprocessing
# Set multiprocessing start method to 'spawn'
multiprocessing.set_start_method('spawn', force=True)

# ==============================================================================
# -------- PyGame Loop ---------------------------------------------------------
# ==============================================================================

def pygame_loop(input_queue, output_queue):
    """ Main loop for agent"""
    pygame.init()
    pygame.font.init()
    world = None
    tot_target_reached = 0
    num_min_waypoints = 21
    # get args
    args = input_queue.get()

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args)
        controller = KeyboardControl(world)

        # pygame clock 
        clock = pygame.time.Clock()

        connected_text = f'Connection established with OpenCDA vehicle...'
        world.hud.notification(connected_text, seconds=1.0)
        count = 0

        # render loop
        while True:
            count += 1
            clock.tick_busy_loop(60)
            if controller.parse_events():
                return

            # As soon as the server is ready continue!
            if not world.world.wait_for_tick(10.0):
                continue

                # as soon as the server is ready continue!
                world.world.wait_for_tick(10.0)
                world.tick(clock)
                world.render(display)
                pygame.display.flip()
                
            else:
                # agent.update_information(world)
                world.tick(clock)
                world.render(display)
                pygame.display.flip()

            # load output queue
            output_queue.put('Pygame loop start ticking ...')

    finally:
        if world is not None:
            world.destroy()

        pygame.quit()

def run_scenario(opt, scenario_params):
    try:
        # init simulation tick count 
        tick = 0
        scenario_params = add_current_time(scenario_params)

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)

        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town='Town06',
                                                   cav_world=cav_world)

        if opt.record:
            scenario_manager.client. \
                start_recorder("single_town06_carla.log", True)

        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'])

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='single_intersection_town06_carla',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()

        # multi-process
        ctx = get_context('spawn')
        input_queue = ctx.Queue(maxsize=1)
        output_queue = ctx.Queue(maxsize=1)
        pygame_process = ctx.Process(target=pygame_loop, 
                                     args=(input_queue,output_queue))
        # put opt to input queue
        input_queue.put(opt)

        # run steps
        while True:
            scenario_manager.tick()
            # increment simulation tick 
            tick += 1

            # pygame rendering 
            if not input_queue.empty() and \
                not pygame_process.is_alive() and\
                tick >= 2:
                pygame_process.start()
                print('start multi-processing!!')

            # catch output queue
            if not output_queue.empty():
                pygame_status = output_queue.get()

            # plan for human takeover
            # human_takeover_sec = random.uniform(1, 100) # random float from 1 to 100 with uniform distribution
            human_takeover_sec = 10 # hard code for debug purpose
            sim_dt = scenario_params['world']['fixed_delta_seconds']
            if tick*sim_dt == human_takeover_sec:
                print('Reduce collision time, human takeover !!!')
                # reduce safety distance 
                single_cav = single_cav_list[0].agent.reduce_following_dist()
                # check collision checker state 
                new_collision_time = single_cav_list[0].agent._collision_check.time_ahead
                print('New collision checker is enabled with: ' + \
                        str(new_collision_time) + 'second ahead time! ')

            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=50),
                carla.Rotation(
                    pitch=-
                    90)))

            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)

    finally:
        input_queue.put(None)  # Signal the GPU process to terminate
        pygame_process.join()
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in single_cav_list:
            v.destroy()
        for v in bg_veh_list:
            v.destroy()

