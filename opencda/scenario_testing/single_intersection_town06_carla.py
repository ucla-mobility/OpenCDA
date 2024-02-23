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
import sys

import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time
# import vision-language model
from opencda.core.sensing.perception.vision_language_manager \
    import VisionLanguageInterpreter

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
    prompt_input =  vlm_manager.prompt_input
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
        images = input_queue.get()  # Get data from the simulation 
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

        # multi-process llava
        model_path = "liuhaotian/llava-v1.5-7b"
        prompt_input = "Based on current traffic condition including traffic light, \
                    generate future driving plan in one short sentence.\
                    If there's no traffic light in pucture, just say it's not detected"

        vlm_manager = VisionLanguageInterpreter(model_path, prompt_input)

        # multi-processing with GPU
        ctx = get_context('spawn')  # Get the context using 'spawn'
        input_queue = ctx.Queue(maxsize=8)
        output_queue = ctx.Queue(maxsize=8)
        gpu_process = ctx.Process(target=vlm_gpu_task_handler,
                                          args=(input_queue, 
                                                output_queue,
                                                vlm_manager))
        # single_cav_list[0].start_vlm_render()

        # Carla spectator
        spectator = scenario_manager.world.get_spectator()
        step = 0
        vlm_ready = False
        idle_vehicle = True

        # run steps
        while True:
            step += 1
            scenario_manager.tick()
            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=50),
                carla.Rotation(
                    pitch=-
                    90)))

            # VLM GPU multi-processing 
            if not input_queue.empty():
                if not gpu_process.is_alive(): 
                    gpu_process.start()

            for i, single_cav in enumerate(single_cav_list):
                single_cav.update_info()

                # off load camera feed
                if single_cav.perception_manager.camera_img_buffer:
                    vlm_image=[single_cav.perception_manager.camera_img_buffer[-1]]
                    
                    # adjust llava frequency (delta_seconds = 0.05s)
                    if step%10 == 0 and step >= 50:
                        input_queue.put(vlm_image)
                        print('***debug stream: lenght of input queue is: ' \
                                + str(input_queue.qsize()))
                
                # VLM GPU process
                if not output_queue.empty():
                    # start moving vehicle 
                    idle_vehicle = False
                    result = output_queue.get()
                    # print(f"Received result: {result}")
                    single_cav.update_vlm_info(result)

                    # debug print, vlm response 
                    print('*** vlm response from vehicle manager is : ' \
                            + str(single_cav.perception_manager.vlm_response))

                # simulation step 
                control = single_cav.run_step()
                if idle_vehicle:
                    single_cav.vehicle.apply_control(
                                        carla.VehicleControl(brake=1.0))
                else:
                    single_cav.vehicle.apply_control(control)

    finally:
        # Clean up
        input_queue.put(None)  # Signal the GPU process to terminate
        gpu_process.join()
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for v in single_cav_list:
            v.destroy()
        for v in bg_veh_list:
            v.destroy()

