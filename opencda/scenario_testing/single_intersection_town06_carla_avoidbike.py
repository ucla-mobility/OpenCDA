# -*- coding: utf-8 -*-
"""
Scenario testing: single vehicle behavior in intersection
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os
import cv2
import carla
import numpy as np
from multiprocessing import Process, Queue, get_context
import multiprocessing
# Set multiprocessing start method to 'spawn'
multiprocessing.set_start_method('spawn', force=True)
import pygame
import sys
import socket
import time
import json

import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time
# import vision-language model
# from opencda.core.sensing.perception.vision_language_manager \
#     import VisionLanguageInterpreter

'''
Helper class for accessing arguments.
'''
class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

def vlm_gpu_task_handler(input_queue, output_queue, vlm_manager):
    # load LLaVA model for the rest of the simulation...
    model_path = vlm_manager.model_path
    # prompt_input =  vlm_manager.prompt_input
    model_name = vlm_manager.get_model_name_from_path(model_path)

    tokenizer, model, image_processor, context_len = vlm_manager.load_pretrained_model(
        model_path=model_path,
        model_base=None,
        model_name=model_name)

    # arguments 
    args = dotdict({
        "conv_mode": None,
        "sep": ",",
        "temperature": 0,
        "top_p": None,
        "num_beams": 1,
        "max_new_tokens": 128
    })
    
    while True:
        images, prompt_input = input_queue.get()  # Get data from the simulation 
        if images is None:  # Use None as a signal to stop the process
            break

        model_response = vlm_manager.get_llava_response(tokenizer,
                                                        model, 
                                                        model_name,
                                                        image_processor, 
                                                        context_len, 
                                                        args,
                                                        prompt_input, 
                                                        images)

        # Optionally send results back to the main process
        output_queue.put(model_response)

def run_scenario(opt, scenario_params):
    try:
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

        # # multi-process llava
        # model_path = "liuhaotian/llava-v1.5-7b"
        # # prompt_input = "Based on current traffic condition including traffic light, \
        # #             generate future driving plan in one short sentence.\
        # #             If there's no traffic light in pucture, just say it's not detected"
        # prompt_input = "Based on current traffic light,\
        #                 determine driving plan in less then 10 words.\
        #                 If no traffic light, say it's not detected. \
        #                 Do not report green light."


        # vlm_manager = VisionLanguageInterpreter(model_path)

        # # multi-processing with GPU
        # ctx = get_context('spawn')  # Get the context using 'spawn'
        # input_queue = ctx.Queue(maxsize=8)
        # output_queue = ctx.Queue(maxsize=8)
        # gpu_process = ctx.Process(target=vlm_gpu_task_handler,
        #                                   args=(input_queue, 
        #                                         output_queue,
        #                                         vlm_manager))

        # Carla spectator
        spectator = scenario_manager.world.get_spectator()
        step = 0
        vlm_ready = False
        idle_vehicle = True
        use_pygame = False

        # ------------- space key press event -------------
        world = scenario_manager.client.get_world()

        print("Press SPACE key to start the vehicle")
        running = False

        # Set up the Pygame window and clock
        pygame.init()

        screen = pygame.display.set_mode((700, 100))
        # Set the font and text for the message
        font = pygame.font.SysFont("monospace", 30)
        text = font.render("Press SPACE to start vehicle movement", \
                    True, (255, 255, 255))

        # Draw the message on the screen
        screen.blit(text, (10, 10))
        pygame.display.flip()

        clock = pygame.time.Clock()

        # -------------------------------------------------
        # connect to llm tcp 
        if use_pygame:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect(('localhost', 5000))

        # manual spectator 
        transform = single_cav_list[0].vehicle.get_transform()
        spectator.set_transform(carla.Transform(
            transform.location +
            carla.Location(
                z=50),
            carla.Rotation(
                pitch=-
                90)))

        # run steps
        while True:
            step += 1
            scenario_manager.tick()

            # pygame event 
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                    running = True

            # VLM GPU multi-processing 
            # if not input_queue.empty():
            #     if not gpu_process.is_alive(): 
            #         gpu_process.start()

            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()

                # simulation step 
                control, vlm_prompt = single_cav.run_step()

                # # off load camera feed
                # if step%10 == 0 and \
                #     single_cav.perception_manager.camera_img_buffer:
                #     vlm_image=[single_cav.perception_manager.camera_img_buffer[-1]]
                #     input_queue.put((vlm_image, vlm_prompt))
                
                # # VLM GPU process
                # if step%10 == 0 and not output_queue.empty():
                #     # start moving vehicle 
                #     idle_vehicle = False
                #     result = output_queue.get()
                #     # print(f"Received result: {result}")
                #     single_cav.update_vlm_info(result)

                #     # debug print, vlm response 
                #     print('*** vlm response from vehicle manager is : ' \
                #             + str(single_cav.perception_manager.vlm_response))

                # vehicle control
                if running == True:
                    # vlm response 
                    single_cav.vehicle.apply_control(control)
                    # vlm_response = single_cav.perception_manager.vlm_response
                    # default VLM response
                    vlm_response = 'Normal traffic, ego vehicle should proceed with current plan.'

                    # FSM info 
                    behavior_FSM = single_cav.agent.Behavior_FSM
                    current_superstate = str(behavior_FSM.current_superstate.name)
                    current_state = str(behavior_FSM.current_state.name)
                    next_superstate = str(single_cav.agent.best_superstate)
                    next_state = str(single_cav.agent.selected_nxt_state)

                    # print('----- FSM Debug stream @ scenario script -----')
                    # print('Current super state: ' + str(current_superstate))
                    # print('Current state: ' + str(current_state))
                    # print('Next super state: ' + str(next_superstate))
                    # print('Next state: ' + str(next_state))
                    # print('----------------------------------------------')
                    # print(' ')

                    # temporarly disable, debug without VLM
                    # manual adjustment 
                    if 'green' in vlm_response:
                        vlm_response = 'No traffic light detected, proceed with current plan.'
                    elif 'not possible' in vlm_response:
                        vlm_response = 'No traffic light detected, proceed with current plan.'
                    elif 'middle lane' in vlm_response:
                        vlm_response = 'Vehicle should stop at red traffic light and yield to other vehicles.'
                        next_state = 'STOP'
                    else: 
                        vlm_response = vlm_response

                    # intersection 
                    if single_cav.agent.near_target_intersection:
                        # red 
                        vlm_response = 'Traffic light is red,'+\
                              ' but turn on red is allowed, ego vehicle should stop on red before proceed.'
                        if single_cav.agent.stop_on_red_counter <= 50:
                            next_state = 'STOP'
                        
                    # bike avoidance
                    if 110 <= single_cav.agent._ego_pos.location.y <= 145:
                        # avoid bike
                        vlm_response = 'There is a cyclist in the current lane, ego vehicle ' + \
                            'should slow down or plan to go around with caution.'
                    elif 90 <= single_cav.agent._ego_pos.location.y < 110 and\
                            single_cav.agent._ego_pos.location.x < 9.8:
                        vlm_response = 'There is no obstacle in the current lane, ' +\
                            'ego vehicle should proceed with current plan.'


                    # construct dict
                    message_dict = {'vlm_response': vlm_response, 
                                    'current_superstate': current_superstate,
                                    'current_state': current_state,
                                    'next_superstate': next_superstate,
                                    'next_state': next_state}
                    # decode 
                    message = json.dumps(message_dict).encode('utf-8')

                    # send vlm 
                    if vlm_response and use_pygame:
                        client_socket.sendall(message)
                else: 
                    single_cav.vehicle.apply_control(
                                        carla.VehicleControl(brake=1.0))

    finally:
        # Clean up
        # input_queue.put(None)  # Signal the GPU process to terminate
        # gpu_process.join()
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in single_cav_list:
            v.destroy()
        for v in bg_veh_list:
            v.destroy()

