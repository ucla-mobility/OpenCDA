# -*- coding: utf-8 -*-
"""
Vision-language model integration.
"""
import os

import cv2
import open3d as o3d
import numpy as np

import argparse
import torch

from llava.constants import (
    IMAGE_TOKEN_INDEX,
    DEFAULT_IMAGE_TOKEN,
    DEFAULT_IM_START_TOKEN,
    DEFAULT_IM_END_TOKEN,
    IMAGE_PLACEHOLDER,
)
from llava.conversation import conv_templates, SeparatorStyle
from llava.model.builder import load_pretrained_model
from llava.utils import disable_torch_init
from llava.mm_utils import (
    process_images,
    tokenizer_image_token,
    get_model_name_from_path,
    KeywordsStoppingCriteria,
)

import requests
from PIL import Image
from io import BytesIO
import re

from llava.model.builder import load_pretrained_model
from llava.mm_utils import get_model_name_from_path
from llava.eval.run_llava import eval_model

'''
Helper class for accessing arguments.
'''
class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class VisionLanguageInterpreter(object):
    """
    Vision-Language interpreter class to interpret based on camera feed.
    (Note: The implementation is silimiar to data dumper.)

    Parameters
    ----------
    perception_manager : opencda object
        The perception manager contains rgb camera data and lidar data.

    vehicle_id : int
        The carla.Vehicle id.

    save_time : str
        The timestamp at the beginning of the simulation.

    Attributes
    ----------
    rgb_camera : list
        A list of opencda.CameraSensor that containing all rgb sensor data
        of the managed vehicle.

    lidar ; opencda object
        The lidar manager from perception manager.

    save_parent_folder : str
        The parent folder to save all data related to a specific vehicle.

    count : int
        Used to count how many steps have been executed. We dump data
        every 10 steps.

    """

    def __init__(self,
                 perception_manager,
                 vehicle_id,
                 save_time):

        self.rgb_camera = perception_manager.rgb_camera
        self.lidar = perception_manager.lidar

        self.save_time = save_time
        self.vehicle_id = vehicle_id

        current_path = os.path.dirname(os.path.realpath(__file__))
        # save the save path for debug purpose
        self.save_parent_folder = \
            os.path.join(current_path,
                         '../../../vision_language_interpret',
                         save_time,
                         str(self.vehicle_id))

        if not os.path.exists(self.save_parent_folder):
            os.makedirs(self.save_parent_folder)

        self.count = 0

        # load LLaVA model for the rest of the simulation...
        self.model_path = "liuhaotian/llava-v1.5-7b"
        self.model_name=get_model_name_from_path(self.model_path)

        self.tokenizer, self.model, \
        self.image_processor, self.context_len = load_pretrained_model(
            model_path=self.model_path,
            model_base=None,
            model_name=self.model_name
        )
        # # define GPU id
        # self.gpu_id = 1
        # model.cuda(device=gpu_id)

        # arguments 
        self.args = dotdict({
            "conv_mode": None,
            "sep": ",",
            "temperature": 0,
            "top_p": None,
            "num_beams": 1,
            "max_new_tokens": 128
        })

    def run_step(self):
        """
        Run vision-language model every 10 steps to interpret the scene
        based on camera input.

        """
        self.count += 1
        # warm-up: first 60 steps
        if self.count < 60:
            return

        # run vision-language model at 10hz
        if self.count % 2 != 0:
            return

        # run LLaVA  
        prompt_input = "Observe the traffic light condition and recommend driveing plans."

        # # get response for each camera 
        # for (i, camera) in enumerate(self.rgb_camera):
        #     camera_img = camera.image
        #     model_response = self.get_llava_response(self.tokenizer,
        #                                              self.model, 
        #                                              self.model_name,
        #                                              self.image_processor, 
        #                                              self.context_len, 
        #                                              self.args,
        #                                              prompt_input, 
        #                                              camera_img)

        # save results to folder
        # 1. save image feed 
        self.save_rgb_image(self.count)
        # 2. load images and get response 
        for (i, camera) in enumerate(self.rgb_camera):
            images=[]
            image_name = '%06d' % self.count + '_' + 'camera%d' % i + '.png'
            image_path = os.path.join(self.save_parent_folder, image_name)        
            image = Image.open(image_path).convert("RGB")
            images.append(image)

        # run llava
        model_response = self.get_llava_response(self.tokenizer,
                                                 self.model, 
                                                 self.model_name,
                                                 self.image_processor, 
                                                 self.context_len, 
                                                 self.args,
                                                 prompt_input, 
                                                 images)

        # 2. save LLaVA response 
        self.save_model_response(self.count, model_response)
        # 3. DEBUG: print step 
        print(model_response)
        print('Save vision-language interpretation to local directory... \n')

    def save_rgb_image(self, count):
        """
        Save camera rgb images to local directory.
        """
        for (i, camera) in enumerate(self.rgb_camera):

            frame = camera.frame
            image = camera.image

            image_name = '%06d' % count + '_' + 'camera%d' % i + '.png'

            cv2.imwrite(os.path.join(self.save_parent_folder, image_name),
                        image)

    def save_model_response(self, count, model_response):
        """
        Save model output text to local directory.
        """
        for (i, camera) in enumerate(self.rgb_camera):

            file_name = '%06d' % count + '_' + 'camera%d' % i + '.txt'
            file_path = os.path.join(self.save_parent_folder, file_name)
            
            with open(file_path, "w") as text_file:
                text_file.write("LLaVA model response for this image is: \n" \
                                + model_response)

    def get_llava_response(self, 
                           tokenizer, model, model_name, image_processor, context_len, args, 
                           prompt_input, images):
        '''
        Get image response from llava.
        '''
        disable_torch_init()
        qs = prompt_input
        image_token_se = DEFAULT_IM_START_TOKEN + DEFAULT_IMAGE_TOKEN + DEFAULT_IM_END_TOKEN

        # append img token with qs
        if model.config.mm_use_im_start_end:
            qs = image_token_se + "\n" + qs
        else:
            qs = DEFAULT_IMAGE_TOKEN + "\n" + qs

        # select model name
        if "llama-2" in model_name.lower():
            conv_mode = "llava_llama_2"
        elif "v1" in model_name.lower():
            conv_mode = "llava_v1"
        elif "mpt" in model_name.lower():
            conv_mode = "mpt"
        else:
            conv_mode = "llava_v0"

        # define conv mode
        if args.conv_mode is not None and conv_mode != args.conv_mode:
            print(
                "[WARNING] the auto inferred conversation mode is {}, while `--conv-mode` is {}, using {}".format(
                    conv_mode, args.conv_mode, args.conv_mode
                )
            )
        else:
            args.conv_mode = conv_mode

        conv = conv_templates[args.conv_mode].copy()
        conv.append_message(conv.roles[0], qs)
        conv.append_message(conv.roles[1], None)
        prompt = conv.get_prompt()

        # load image 
        images_tensor = process_images(
            images,
            image_processor,
            model.config
        ).to(model.device, dtype=torch.float16)

        # input id
        input_ids = (
            tokenizer_image_token(prompt, tokenizer, IMAGE_TOKEN_INDEX, return_tensors="pt")
            .unsqueeze(0)
            .cuda()
        )

        # stopping criteria 
        stop_str = conv.sep if conv.sep_style != SeparatorStyle.TWO else conv.sep2
        keywords = [stop_str]
        stopping_criteria = KeywordsStoppingCriteria(keywords, tokenizer, input_ids)

        # launch model
        with torch.inference_mode():
            output_ids = model.generate(
                input_ids,
                images=images_tensor,
                do_sample=True if args.temperature > 0 else False,
                temperature=args.temperature,
                top_p=args.top_p,
                num_beams=args.num_beams,
                max_new_tokens=args.max_new_tokens,
                use_cache=True,
                stopping_criteria=[stopping_criteria],
            )

        input_token_len = input_ids.shape[1]
        n_diff_input_output = (input_ids != output_ids[:, :input_token_len]).sum().item()
        if n_diff_input_output > 0:
            print(
                f"[Warning] {n_diff_input_output} output_ids are not the same as the input_ids"
            )
        outputs = tokenizer.batch_decode(
            output_ids[:, input_token_len:], skip_special_tokens=True
        )[0]
        outputs = outputs.strip()
        if outputs.endswith(stop_str):
            outputs = outputs[: -len(stop_str)]
        outputs = outputs.strip()

        # return results 
        return outputs