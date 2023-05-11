# CARLA Log Replay Tool for Cooperative Perception Dataset
<p align="center">
<img src="../images/logreplay.gif" width="600" alt="" class="img-responsive">
</p>
This repository provides a toolbox to replay simulated cooperative perception dataset in CARLA. With 
this toolbox, you can 100% replay all the events in the offline dataset and add/change any sensors/groundtruth you
want to explore the tasks that the origin dataset do not support.

## Features
- Support replaying multiple cooperative perception dataset in CARLA
  - [x] [OPV2V [ICRA2022]](https://mobility-lab.seas.ucla.edu/opv2v/)
  - [ ] [V2XSet [ECCV2022]]()
- Help you add new sensors to the agents to have additional sensor data, including but not limited to:
  - Depth sensor
  - Event camera
  - Semantic LiDAR
  - Radar
  
- Dump new groundtruth that the origin dataset does not have, including but not limited to:
  - HDMap ([CoBEVT](https://arxiv.org/abs/2207.02202) (CoRL2022) used this groundtruth )
  - Trajectory for each vehicle
- Change the number of CAVs! The original OPV2V data has fixed CAVs, but with this tool, you can
assign any vehicle to a CAV equipped with sensors.

## How to use
### Step0: Install CARLA 0.9.12 and CARLA API to your python environment
Check the CARLA official website: https://carla.readthedocs.io/en/latest/start_quickstart/
### Step1: Yaml file configuration
Please change the parameters in `logreplay/hypes_yaml/replay.yaml` to configure your replay.
We have provided very detailed explanation for each line in the yaml. 
### Step2: add new sensors if necessary
We have by default provide Semantic Lidar and BEV semantic camera sensors in the yaml file and codebase.
If you want to add some new sensors, please add your sensor under `logreplay/sensors/` and refer to the `semantic_lidar`
or `bev_semantic_camera.py` to write your own sensor class.
### Step3: Run CARLA
Start your CARLA simulator first. Right now we only support version 0.9.12 
### Step4: run the log replay tool
Execute the following command:
```python
cd logreplay
python scenario/scenearios_manager.py
```


