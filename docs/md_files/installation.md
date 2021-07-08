# OpenCDA Installation

* __[System/Hardware Requirements](#requirements)__  
* __[Local Installation](#local-installation)__
    * [1,CARLA installation](#carla-installation) 
	    * [1.1 Debian CARLA installation](#1-debian-carla-installation)  
	    * [1.2 Package installation](#2-package-installation)  
	    * [1.3 Build from source](#3-build-from-source)  

    * [2. Install OpenCDA](#opencda-installation)
    * [3. Install Pytorch and Yolov5(Optional)](#)




---
## Requirements
To get started, the following requirements should be fulfilled.
* __System requirements.__ Any 64-bits OS should run OpenCDA. We highly recommends Ubuntu  16.04/18.04/20.04.

* __Adequate GPU.__ CARLA is a realistic simulation platform based on Unity, which requires at least a 3GB GPU for smooth scene rendering, though 8GB is recommended.
* __Disk Space.__ Estimate 30GB of space is recommended to install CARLA. 
* __Python__ Python3,7 or higher version is required for full functions.


---
## Local Installation
To get OpenCDA v0.1 running with complete functionality, you will need four things: CARLA, OpenCDA, and
Pytorch(optional). Pytorch is required only when you want to activate the perception module, otherwise OpenCDA
will retrieve all perception information from the simulation server directly.

###  1. CARLA Installation(0.9.11 required)

There are three different  ways to install the CARLA simulator and either way is fine for using OpenCDA. <br>
** Note: If you want to use the customized highway map with full assets(.fbx, .xml and .xodr) in OpenCDA, 
you have to build from source. Visit CARLA's tutorial [ADD a new map](https://carla.readthedocs.io/en/latest/tuto_A_add_map_overview/) for more information.

#### 1.1 Debian CARLA installation

Set up the Debian repository in the system.
```sh
   sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
   sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
```
Install CARLA and check for the installation in the `/opt/` folder.
```sh
   sudo apt-get update # Update the Debian package index
   sudo apt-get install carla-simulator # Install the latest CARLA version, or update the current installation
   cd /opt/carla-simulator # Open the folder where CARLA is installed
```

This repository contains CARLA 0.9.11 and later versions. To install a specific version add the version tag to the installation command.  
```sh
   sudo apt-get install carla-simulator=0.9.11
```

#### 1.2 Package installation

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/releases/tag/0.9.11" target="_blank" class="btn btn-neutral" title="Go to the latest CARLA release">
<span class="icon icon-github"></span> CARLA 0.9.11 Package</a>
</p>
</div>

OpenCDA is only tested at 0.9.11 and thus we highly recommend to use this version.
To insall CARLA as a precompiled package, download and extract the release file. It contains a precompiled version of the simulator, the Python API module and some scripts to be used as examples. <br>
** Note: The  AdditionalMaps_0.9.11.tar.gz also need to be downloaded and extract to the CARLA repo to support
scenario testings in Town06.

#### 1.3 Build From Source

For advanced CARLA usage that involves extensive customizations, [Build CARLA from Source](https://carla.readthedocs.io/en/latest/build_linux/) is also supported by OpenCDA. Though source build in 
Windows OS is supported by CARLA, Ubuntu is the preferred OS as the OpenCDA was developoed in Ubuntu 18.04.  
 
** Note: OpenCDA do not require CARLA source build. However, customized map with building/lane/traffic light/road surface materials assets  in CARLA  require source build. 
Visit CARLA's tutorial ["ADD a new map"](`CARLA Tutorials (assets). <https://carla.readthedocs.io/en/latest/tuto_A_add_map_overview/) for more information. 
 
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
pip install -r requirement.txt
```

After dependencies are installed, we need to install the CARLA PythonAPI package into the conda environment.
You can do this by running this script:
```sh
export CARLA_HOME=/path/to/your/CARLA_ROOT
# remember to set CARLA_HOME first to point to your carla repo
. setup.sh
```
If everything works correctly, you will see a cache folder is created in your OpenCDA root dir, and inside the cache folder
you will see an egg file and its unzipped folder. Then install this CARLA egg into your conda environment.
```sh
pip install -e cahce/carla-0.9.11-py3.7-linux-x86_64
python -c "import carla" # check whether carla is installed correctly.
```
**Note: If you are using Python other than 3.7 and CARLA rather than 0.9.11, then you have to change the setup.sh to your
carla version's egg file or manually installed carla to your conda environment.

### 3. Install Pytorch and Yolov5(Optional)
This section is only needed for the users who want to test perception algorithms. By default, OpenCDA does not require
pytorch installed and it retrieves the object positions from the server directly. Once perception module is activated,
then OpenCDA will use yolov5 with pytorch to run object detection. <br>
To install pytorch based on your GPU and cuda version, go to the official pytorch website and install with conda command. Make
sure you install pytorch >= 1.7.0
<div class="build-buttons">
<p>
<a href="https://pytorch.org/" target="_blank" class="btn btn-neutral" title="Pytorch">
<span class="icon icon-github"></span>Pytorch Official Website</a>
</p>
</div>

After pytorch installation, install the requirements for Yolov5. <br>
```sh
pip install -qr https://raw.githubusercontent.com/ultralytics/yolov5/master/requirements.txt  # install dependencies
```

