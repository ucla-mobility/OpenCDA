# OpenCDA Installation

* __[Installation summary](#installation-summary)__  
* __[Requirements](#requirements)__  
* __[CARLA installation](#carla-installation)__  
	* [1. Debian CARLA installation](#1-debian-carla-installation)  
	* [2. Package installation](#2-package-installation)  
	* [3. Build from source](#3-build-from-source)  
* __[SUMO installation](#sumo-installation)__  
	* [1. Download SUMO](#1-download-sumo)  
	* [2. Installation Python Package](#2-install-python-packages)  
* __[Install OpenCDA](#opencda-installation)__  
	* [1. Install CARLA as Python package](#1-install-carla-as-python-package)  
	* [2. Import CARLA for Conda Users](#2-import-carla-for-conda-users)  
	* [3. Install OpenCDA](#3-install-opencda)  
* __[Running CARLA](#running-carla)__  
* __[Follow-up](#follow-up)__  


---
## Installation summary

This installation guide shows how to download and running OpenCDA in your lcoal enviroment. OpenCDA is a generalized frame-work that encapsulated both CARLA and SUMO as simulation platform. With OpenCDA, users can conveniently replace any default module with customized algorithms or protocols and perform evaluations. A concrete example of platooning implementation will be used to illustrate the framework's utility for CDA research.

---
## Requirements
To get started, the following requirements should be fullfilled.
* __System requirements.__ Any 64-bits OS should run OpenCDA. We highly recommends Ubuntu 18.04.

* __Adequate GPU.__ CARLA is a realistic simulation platform based on Unity, which requires at least a 6GB GPU for smooth redering, though 8GB is recommended.
* __Disk Space.__ Estimate 30GB of space is recomended. 
* __Python 3.__ Python is the main script language in CARLA, SUMO and OpenCDA. OpenCDA provide easy scenario declaration through yaml, but Python3 is required for normal operation. 
* __Simulation platform.__ It is necessary to install [CARLA](https://carla.readthedocs.io/en/latest/start_quickstart/) and [SUMO](https://www.eclipse.org/sumo/>) as OpenCDA uses them as main simulation platform. 
* __Python Version Control.__ We recommend to use [Anaconda](https://www.anaconda.com/download) or [Miniconda](https://conda.io/miniconda.html) for handeling multiple Python versions in parallel enviroments.
* __Other requirements.__  Two Python modules: [Pygame](https://pypi.org/project/pygame/) to create graphics directly with Python, and [Numpy](https://pypi.org/project/numpy/) for great calculus.  

To install both modules using [pip](https://pip.pypa.io/en/stable/installing/), run the following commands. 
```sh
   pip install --user pygame numpy
```    

---
## CARLA Installation

The __Debian installation__ is the easiest way to get the latest release in Linux.  
__Download the GitHub repository__ to get either a specific release or the Windows version of CARLA.  

### 1. Debian CARLA installation

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

This repository contains CARLA 0.9.10 and later versions. To install a specific version add the version tag to the installation command.  
```sh
   sudo apt-get install carla-simulator=0.9.10-1 # In this case, "0.9.10" refers to a CARLA version, and "1" to the Debian revision
```

### 2. Package installation

<div class="build-buttons">
<p>
<a href="https://github.com/carla-simulator/carla/blob/master/Docs/download.md" target="_blank" class="btn btn-neutral" title="Go to the latest CARLA release">
<span class="icon icon-github"></span> CARLA repository</a>
</p>
</div>

The repository contains different versions of the CARLA simulator available. The OpenCDA is developed with CARLA 0.9.11, and we recommend this version for any OpenCDA usage. For more information about different versions or releases, please visit [CARLA installation guide](http://127.0.0.1:8000/OpenCDA_installation/#b-package-installation).  

To insall CARLA as a precompiled package, download and extract the release file. It contains a precompiled version of the simulator, the Python API module and some scripts to be used as examples. 

### 3. Build From Source

For advanced CARLA usage that involves extensive customizations, [Build CARLA from Source](https://carla.readthedocs.io/en/latest/build_linux/) is also supported by OpenCDA. Though source build in 
Windows OS is supported by CARLA, Ubuntu is the preferred OS as the OpenCDA was developoed in Ubuntu 18.04.  
 
** Note: OpenCDA do not require CARLA source build. However, custom map ingestion in CARLA may require source build or a docker image. Visit CARLA's tutorial ["ADD a new map"](`CARLA Tutorials (assets). <https://carla.readthedocs.io/en/latest/tuto_A_add_map_overview/) for more information. 
 
---
## SUMO Installation

To implement SUMO with OpenCDA alongside CARLA, it is required to install latest version of SUMO and the corresponding Python package.   

### 1. Download SUMO
Download the most recent sumo to your ubuntu.  

```sh
   sudo add-apt-repository ppa:sumo/stable
   sudo apt-get update
   sudo apt-get install sumo sumo-tools sumo-doc
```
For more information or earlier releases of SUMO, please refer to the [SUMO documentation](https://sumo.dlr.de/docs/Downloads.php).  

### 2. Install Python packages
Starting with SUMO 1.8.0 the installation is possible from [Python packaging index](https://pypi.org/project/eclipse-sumo/). You can use pip to install the full python package.  

```sh
   pip install eclipse-sumo
```
   
Alternatively, install an indiviaul package from the SUMO Python package stack(i.e., traci, libsumo, and sumolib) is also supported.  

```sh
   pip install <package-name-to-install>
```

For OpenCDA, we recommend install the full SUMO Python package.  

---
## OpenCDA Installation

Before installing OpenCDA, please verify the CARLA and SUMO installation status. OpenCDA uses CARLA as main simulation paltform, hence importing CARLA from its 
root directory is necessary.  

### 1. Install CARLA as Python package
 
To make CARLA importable, it is necessary to install CARLA as a individual python package.  

```sh
   cd <your-path-to-carla>/PythonAPI/carla/dist
   unzip carla-<your-carla-version>-py3.6-linux-x86_64.egg -d carla-<your-carla-version>-py3.6-linux-x86_64
   cd carla-<your-carla-version>-py3.6-linux-x86_64
```

Next, creat a python file with the name "setup.py", and copy the following contents.

```sh
  from distutils.core import setup
  setup(name='carla',
        version='0.9.10', 
        py_modules=['carla'],
        ) 
```

Once the setup file has been created, install CARLA using ** pip**.  

```sh
  pip3 install -e <your-path-to-carla>/PythonAPI/carla/dist/carla-<your-carla-version>-py3.6-linux-x86_64
```
This method is valid for all CARLA installation method including source build.
Alternatively, it is possible to declare the path to the ** egg** file at the beginning 
of the script before import CARLA. Please refer to the Python examples provided by CARLA 
for more details. 

### 2. Import CARLA for Conda Users
If you have a dedicated conda enviroment for running OpenCDA. Please refer to the previous sections to install CARLA inside the corresponding conda enviroment to make CARLA importable from any directory.  

```sh
  conda activate <your-env-name>
  pip3 install -e <your-path-to-carla>/PythonAPI/carla/dist/carla-<your-carla-version>-py3.6-linux-x86_64
```

### 3. Install OpenCDA
TO isntall OpenCDA, please download the latest release of OpenCDA from our github repository.  

```sh
  git clone https://github.com/ucla-mobility/OpenCDA
  cd OpenCDA
```


---
## Quickstart Examples


We provide quickstart examples to help users understanding how to execute a testing scenario. All pre-defined scenarios are located at directory **"OpenCDA/scenario_testing"**. To start a scenario, simple run the following command fron terminal:   

```sh
  cd <your-path-to-OpenCDA>/scenario_testing
  python3 <scenario-name>.py
```

To configure the parameters of pre-defined scebarios, such as vehicle spawned location, activated sensor suite, vehicle controller, vehicle behavior, and traffic manager settings, navigate to 
```sh
   cd <your-path-to-OpenCDA>/scenario_testing/config_yaml/<your-scenario-name>.yaml
```
and reconfigure the parameters in the ** yaml** file of the corresponding scenario.  

