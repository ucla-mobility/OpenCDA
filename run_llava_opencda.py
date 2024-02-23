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
from collections import deque
import requests
from PIL import Image
from io import BytesIO
import re
import time 

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

from llava.model.builder import load_pretrained_model
from llava.mm_utils import get_model_name_from_path
from llava.eval.run_llava import eval_model

# define GPU 
os.environ['CUDA_VISIBLE_DEVICES'] ='1'

# some utils
class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

def image_parser(args):
    out = args.image_file.split(args.sep)
    return out

def load_image(image_file):
    if image_file.startswith("http") or image_file.startswith("https"):
        response = requests.get(image_file)
        image = Image.open(BytesIO(response.content)).convert("RGB")
    else:
        image = Image.open(image_file).convert("RGB")
    return image

def load_images(save_img_path):
    images=[]        
    image = Image.open(save_img_path).convert("RGB")
    images.append(image)
    return images

def get_llava_response(tokenizer, 
                       model, image_processor, context_len, 
                       prompt_input, args, images):
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

    # load image tensor
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

# ----- demo -----

# load model 
model_path = "liuhaotian/llava-v1.5-7b"
model_name=get_model_name_from_path(model_path)
is_8_bit = False
is_4_bit = True

tokenizer, model, image_processor, context_len = load_pretrained_model(
    model_path=model_path,
    model_base=None,
    model_name=model_name,
    load_8bit=is_8_bit, 
    load_4bit=is_4_bit
)

# arguments 
image_url = "https://scontent-sjc3-1.xx.fbcdn.net/v/t39.30808-6/420004364_2581402622019929_7825484103282604255_n.jpg?_nc_cat=107&ccb=1-7&_nc_sid=3635dc&_nc_ohc=j64ziDdZCoUAX9n1CMV&_nc_oc=AQlmA5prUgrQYb8PaE7iNNWfiESP0ZQSWhAV4DyLWICTNzIMHYq8YrEvYiXaO72niDCQxO_A8GRe2LfNZRzZlvZg&_nc_ht=scontent-sjc3-1.xx&oh=00_AfDp6N-bIH-AjV9RG1whl0BgtbhjMPlyC-j1h0elp1f_zA&oe=65AA211D"
args = dotdict({
    "conv_mode": None,
    "image_file": image_url,
    "sep": ",",
    "temperature": 0,
    "top_p": None,
    "num_beams": 1,
    "max_new_tokens": 128
})

# prompts 
prompt_input = "Observe the current traffic condition \
                and recommend driveing plan in one short sentence."

# define current folder 
current_path = os.path.dirname(os.path.realpath(__file__))
i = 0
# inference time 
inference_times = []

# run steps
try:
    while True:
        i += 1
        # img path 
        image_path = '/home/cav/OpenCDA/vision_language_interpret/seperate_test/'
        image_name = 'current_front_view_'+ str(int(i)) +'.png'
        save_img_path = os.path.join(image_path, image_name)
       
        # load image 
        images = load_images(save_img_path)

        if images is not None:
            # measure inference time 
            start = time.time()
            # get output 
            response = get_llava_response(tokenizer, model, image_processor, 
                                            context_len, prompt_input, args, images)
            end = time.time()
            # print LLaVa response 
            print('-----Current image response: -----')
            print(response)
            print('')
            # inference time 
            inference_time = end-start
            inference_times.append(inference_time)
        
        # save inference time 
        if len(inference_times) >= 50:
            print('Collected 50 inference times...')
            print('average Inference time is: ' + str(np.sum(inference_times)/50))
            print('saving inference time data...')

            # namings
            if is_4_bit:
                csv_name = "inference_time_4bit.csv"
            elif is_8_bit:
                csv_name = "inference_time_8bit.csv"
            else:
                csv_name = "inference_time_no_quanti.csv"
            # path
            inference_time_path = os.path.join(image_path, csv_name)
            # save
            np.savetxt(inference_time_path, np.asarray(inference_times), delimiter=",")

        else:
            print('Waiting for images...')
finally:
    print('Simulation done.')