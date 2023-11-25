# Installation

* [__System/Hardware Requirements__](#requirements)
* [__Local Installation__](#local-installation)
    * [__1. CARLA installation__](#1-carla-installation0911-required)
	    * [1.1 Package installation](#11-package-installation)  
	    * [1.2 Build from source](#12-build-from-source)  

    * [__2. Install OpenCDA__](#opencda-installation)
    * [__3. Install Pytorch and Yolov5 (Optional)__](#3-install-pytorch-and-yolov5optional)
    * [__4. Install Sumo (Optional)__](#4-install-sumooptional)

* [__Docker Installation__](#docker-installation)


---
## Requirements
To get started, the following requirements should be fulfilled.
* __System requirements.__ Any 64-bits OS should run OpenCDA. We highly recommends Ubuntu  16.04/18.04/20.04.

* __Adequate GPU.__ CARLA is a realistic simulation platform based on Unreal Engine 4, which requires at least a 3GB GPU for smooth scene rendering, though 8GB is recommended.
* __Disk Space.__ Estimate 100GB of space is recommended to install CARLA and Unreal Engine. 
* __Python__ Python3.7 or higher version is required for full functions.


---
## Local Installation
To get OpenCDA v0.1 running with complete functionality, you will need four things: CARLA, OpenCDA, and
Pytorch (optional). Pytorch is required only when you want to activate the perception module; otherwise OpenCDA
will retrieve all perception information from the simulation server directly.

###  1. CARLA Installation (>= 0.9.11 required)

There are two different recommended ways to install the CARLA simulator and either way is fine for using OpenCDA. <br>
Note: If you want to use the customized highway map with full assets (.fbx, .xml and .xodr) in OpenCDA, 
you have to build from source. Visit CARLA's tutorial [ADD a new map](https://carla.readthedocs.io/en/latest/tuto_A_add_map_overview/) for more information.


#### 1.1 Package Installation

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/releases" target="_blank" class="btn btn-neutral" title="Go to the latest CARLA release">
<span class="icon icon-github"></span> CARLA Released Package</a>
</p>
</div>

OpenCDA is tested both at 0.9.11 and 0.9.12.
To install CARLA as a precompiled package, download and extract the release file. It contains a precompiled version of the simulator, the Python API module and some scripts to be used as examples. <br>

<strong>Note: The  AdditionalMaps_0.9.1x.tar.gz also need to be downloaded and extract to the CARLA repo to support
scenario testings in Town06.</strong>

#### 1.2 Build From Source

For advanced CARLA usage that involves extensive customizations, [Build CARLA from Source](https://carla.readthedocs.io/en/0.9.11/build_linux/) is also supported by OpenCDA. Though source build in 
Windows OS is supported by CARLA, Ubuntu is the preferred OS as the OpenCDA was developoed in Ubuntu 18.04.  

<strong>Note: OpenCDA do not require CARLA source build. However, customized map with building/lane/traffic light/road surface materials assets  in CARLA  require source build. 
Visit CARLA's tutorial [ADD a new map](https://carla.readthedocs.io/en/latest/tuto_A_add_map_overview/) for more information. </strong>

---

### 2. OpenCDA Installation
First, download OpenCDA github to your local folder if you haven't done it yet.
```sh
git clone https://github.com/ucla-mobility/OpenCDA.git
cd OpenCDA
```
Make sure you are in the root dir of OpenCDA, and next let's install the dependencies. <strong>We highly
recommend use conda environment to install.</strong> 

```sh
conda env create -f environment.yml
conda activate opencda
python setup.py develop
```

If conda install failed,  install through pip
```sh
pip install -r requirements.txt
```

After dependencies are installed, we need to install the CARLA python library into opencda conda environment.
You can do this by running this script:
```sh
export CARLA_HOME=/path/to/your/CARLA_ROOT
export CARLA_VERSION=0.9.11 #or 0.9.12 depends on your CARLA
. setup.sh
```
If everything works correctly, you will see a cache folder is created in your OpenCDA root dir, and the terminal shows
"Successful Setup!". To double check the carla package is correctly installed, run the following command and 
there should be no error.
```sh
python -c "import carla" # check whether carla is installed correctly.
```
<strong>Note: If you are using Python other than 3.7 and CARLA < 0.9.11 then you have to change the setup.sh to your
carla version's egg file or manually installed carla to your conda environment.</strong>


### 3. Install Pytorch and Yolov5 (Optional)
This section is only needed for the users who want to test perception algorithms. By default, OpenCDA does not require
pytorch installed and it retrieves the object positions from the server directly. Once perception module is activated,
then OpenCDA will use yolov5 with pytorch to run object detection. <br>
To install pytorch based on your GPU and cuda version, go to the official pytorch website and install with conda command. Make
sure you install pytorch >= 1.7.0.  <strong>GPU Version highly recommended!</strong>
<div class="build-buttons">
<p>
<a href="https://pytorch.org/" target="_blank" class="btn btn-neutral" title="Pytorch">
<span class="icon icon-github"></span>Pytorch Official Website</a>
</p>
</div>

The command belows shows an example of installing pytorch v1.8.0 with cuda 11.1 in opencda
environment.

```sh

conda install pytorch==1.8.0 torchvision==0.9.0 torchaudio==0.8.0 cudatoolkit=11.1 -c pytorch -c conda-forge

```

After pytorch installation, install the requirements for Yolov5. <br>
```sh
pip install -qr https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt  # install dependencies
```

### 4. Install SUMO (Optional)
SUMO installation is only required for the users who require to conduct co-simulation testing and use future release of SUMO-only mode.

You can install SUMO directly by apt-get:
```sh
sudo add-apt-repository ppa:sumo/stable
sudo apt-get update
sudo apt-get install sumo sumo-tools sumo-doc
```
After that, install the traci python package.
```sh
pip install traci
```
Finally, add the following path to your ~/.bashrc:
```yaml
export SUMO_HOME=/usr/share/sumo
```
---

### 5. Install OpenScenario (Optional)
If you want to use OpenScenario to conduct scenario testing, e.g. `python opencda.py -t openscenario_carla -v 0.9.12`, you need to install OpenScenario first. <br>

Please follow the [openscenario installation](coperception/setup_scenario_runner.md) to install OpenScenario.

## Docker Installation
OpenCDA provides docker image for users to run directly.

Note: Openscenario is not supported in docker image yet.

### 1. Prerequisite
First, make sure that you have installed `docker` in your ubuntu system. If you have nvidia gpu, it is recommended to
install `nvidia-docker` as well.

### 2. Dockerfile Modification
Next, modify the Dockerfile based on your system. Some parameters you may want to pay attentions are:

- `CARLA_VERSION`: Change to the version you want. Notice after(including) OpenCDA 0.1.4, only CARLA >= 0.9.14 will be supported.
- `FROM nvidia/vulkan:1.3-470`: If you want the carla run with rendering in the docker, make sure the graphics driver is set to the same
as your local computer. For instance, mine is `FROM nvidia/cuda-11.4.2.0-ase-ubuntu2004`
- `OPENCDA_FULL_INSTALL`: If set true, it will run the `setup.sh` in OpenCDA automatically during building up the docker to install carla api into the python environment.
if set to false, then you need to go to your docker container after building to manually run `setup.sh`

### 3. Build up the docker
```shell
cd OpenCDA
docker build -t opencda_container .
```

### 4. Run the docker
The following command will allow you run Carla with rendering in your docker. If the rendering is the black,
then you probably build up the wrong nvidia graphics driver.
```shell
xhost +local: 

docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/root/.Xauthority \
  --name opencda_container opencda_docker /bin/bash
```

