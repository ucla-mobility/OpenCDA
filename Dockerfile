# MIT License
#
# Copyright (C) 2023 Goodarz Mehr
# Copyright (C) 2023 Virginia Tech ASIM Lab
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.

# OpenCDA Docker Configuration Script
#
# Performs all the necessary tasks regarding the generation of a generic
# OpenCDA Docker image, including installation of CARLA and its additional
# maps, perception components (PyTorch and YOLOv5), and SUMO.
#
# The base Docker image is Ubuntu 20.04 with CUDA 11.4.2 and Vulkan SDK
# 1.3.204.1. If you want to use a different base image, you may need to modify
# "ubuntu2004/x86_64" when fetching keys according to your Ubuntu release and
# system architecture.

# Build Arguments (Case Sensitive):
#
# USER:                 default username inside each container, set to
#                       "opencda" by default.
# CARLA_VERSION:        desired version of CARLA, set to "0.9.12" by default.
# ADDITIONAL_MAPS:      whether additional CARLA maps should be installed, set
#                       to "true" by default.
# PERCEPTION:           whether perception components (PyTorch and YOLOv5)
#                       should be installed, set to "true" by default.
# SUMO:                 whether SUMO should be installed, set to "true" by
#                       default.
# OPENCDA_FULL_INSTALL: whether OpenCDA should be fully installed and set up,
#                       or if only the required dependencies should be
#                       installed. In the latter OpenCDA can be mounted to the
#                       container at runtime, enabling faster development
#                       cycles. Set to "false" by default.

# Installation:
#
# 1. Install Docker on your system (https://docs.docker.com/engine/install/).
# 2. If you are using an Nvidia graphics card, install the Nvidia Container
# Toolkit (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit
# /install-guide.html#installation-guide). It exposes your Nvidia graphics
# card to Docker containers.
# 3. In the Dockerfile directory, run
#
# docker build --no-cache --rm --build-arg ARG -t opencda:develop .

# Usage:
#
# Launch a container by running
#
# docker run --privileged --gpus all --network=host -e DISPLAY=$DISPLAY
# -v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d
# -it opencda:develop /bin/bash
#
# Use "nvidia-smi" and "vulkaninfo --summary" to ensure your graphics card and
# Vulkan are both available inside the container. You may need to add some or
# all of the following when launching the container to ensure this.
#
# -e SDL_VIDEODRIVER=x11
# -e XAUTHORITY=$XAUTHORITY
# -v /tmp/.X11-unix:/tmp/.X11-unix:rw
# -v $XAUTHORITY:$XAUTHORITY

FROM nvidia/vulkan:1.3-470

# Define build arguments and environment variables.

ARG USER=opencda
ARG CARLA_VERSION=0.9.12
ARG ADDITIONAL_MAPS=true
ARG PERCEPTION=true
ARG SUMO=true
ARG OPENCDA_FULL_INSTALL=true

ENV TZ=America/New_York
ENV DEBIAN_FRONTEND=noninteractive
ENV CARLA_VERSION=$CARLA_VERSION
ENV CARLA_HOME=/home/carla
ENV SUMO_HOME=/usr/share/sumo

# Add new user and install prerequisite packages.

WORKDIR /home

RUN useradd -m ${USER}

RUN set -xue && apt-key del 7fa2af80 \
&& apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub \
&& apt-get update \
&& apt-get install -y build-essential cmake debhelper git wget xdg-user-dirs xserver-xorg libvulkan1 libsdl2-2.0-0 \
libsm6 libgl1-mesa-glx libomp5 pip unzip libjpeg8 libtiff5 software-properties-common nano fontconfig

# Install CARLA and its additional maps.

RUN mkdir carla

RUN wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_${CARLA_VERSION}.tar.gz -nv --show-progress \
--progress=bar:force:noscroll \
&& tar -zxvf CARLA_${CARLA_VERSION}.tar.gz --directory carla && rm CARLA_${CARLA_VERSION}.tar.gz \
&& if [ ${ADDITIONAL_MAPS} = true ] ; then \
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_${CARLA_VERSION}.tar.gz -nv \
--show-progress --progress=bar:force:noscroll && \
tar -zxvf AdditionalMaps_${CARLA_VERSION}.tar.gz --directory carla && rm AdditionalMaps_${CARLA_VERSION}.tar.gz ; \
elif [ ${ADDITIONAL_MAPS} != false ] ; then echo "Invalid ADDITIONAL_MAPS argument." ; \
else echo "Additional CARLA maps will not be installed." ; fi && chown -R ${USER}:${USER} /home/carla

# Install the perception components (PyTorch and YOLOv5).

RUN if [ ${PERCEPTION} = true ] ; then \
pip install torch torchvision torchaudio yolov5 ; \
elif [ ${PERCEPTION} != false ] ; then echo "Invalid PERCEPTION argument." ; \
else echo "Perception components (PyTorch and YOLOv5) will not be installed." ; fi

# Install SUMO.

RUN if [ ${SUMO} = true ] ; then \
add-apt-repository ppa:sumo/stable && apt-get update && apt-get install -y sumo sumo-tools sumo-doc \
&& pip install traci ; \
elif [ ${SUMO} != false ] ; then echo "Invalid SUMO argument." ; \
else echo "SUMO will not be installed." ; fi

# Install OpenCDA.

RUN if [ ${OPENCDA_FULL_INSTALL} = false ] ; then \
wget https://raw.githubusercontent.com/ucla-mobility/OpenCDA/main/requirements.txt \
&& pip install -r requirements.txt && rm requirements.txt ; \
elif [ ${OPENCDA_FULL_INSTALL} = true ] ; then \
git clone https://github.com/ucla-mobility/OpenCDA.git && pip install -r OpenCDA/requirements.txt \
&& chmod u+x OpenCDA/setup.sh && sed -i '/conda activate opencda/d' OpenCDA/setup.sh \
&& sed -i 's+${PWD}/+${PWD}/OpenCDA/+g' OpenCDA/setup.sh && ./OpenCDA/setup.sh \
&& chown -R ${USER}:${USER} /home/OpenCDA ; \
else echo "Invalid OPENCDA_FULL_INSTALL argument." ; fi

USER ${USER}
