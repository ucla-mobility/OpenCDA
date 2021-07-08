## Quick start 
OpenCDA provides  a default scenario database for users to get a quick start. A  scenario testing contains two components:
A python script and a yaml file. The yaml file will define CAVs' parameters, the traffic flow and the 
driving task of each CAV(where to start and where is the destination). The python script will load the yaml file 
and pass it to corresponding api to construct the scenario.<br>

Running a scenario testing can be done by just one line of command.

### 1. Single vehicle testing(w/o perception)
```sh
conda activate opencda
cd ~/OpenCDA
""" comments, do not input these to your terminal
single->single vehicle testing, 
2lanefree->testing map name, 
carla->only carla is involved(no sumo and ns3)
"""
python opencda.py -t single_2lanefree_carla
```
In this scenario testing, a single connected automated vehicle will be spawn at a 2-lane highway customized map.  This
CAV will try to reach the assigned destination with a desired speed of 100km/h and manage to safely and efficiently interact
with the surrounding traffic flow. The CAV's localization, planning, and control modules will be activated, and the perception module will be deactivated
by default, thus <strong> pytorch is NOT required in this testing </strong>. <br>

If you want to activate the perception module, please check [Yaml Defining Rules](Opencda_yaml.md) to see details.

![teaser](images/single_2lanefree_carla.gif)

** Note: The bounding boxes draw on the camera and lidar are projected from the true 3d poses from the server directly. 

### 2. Single vehicle testing(w/ perception)
```sh
python opencda.py -t single_town06_carla --apply_ml
```
The apply_ml flag will import the pytorch library and load Yolov5 model(<strong>Thus Pytorch is required</strong>) for object detection. Thus, in this
scenario, the <strong>perception</strong>, localization, planning and control modules will all be activated.

![teaser](images/single_town06_carla_2.gif)

### 3. Platooning(w/o perception)
```sh
python opencda.py -t platoon_stability_2lanefree_carla.py
```
In this scenario, a platoon with 4 members will be placed at the 2-lane highway map. The platoon leader will dramatically increases
and decreases its speed to test whether the members can still keep the desired time gap. In this way, the platoon
stability is verified.

### 4. Cooperative merge and join a platoon(w/o perception)
```sh
python opencda.py -t platoon_stability_2lanefree_carla.py
```
In this scenario, a platoon will drive on the mainline together with a mixed traffic flow. A single CAV will come from the 
merging lane, communicate with the platoon to cooperatively merge into the mainline, and simultaneously join the platoon.

![teaser](images/platoon_joining_2lanefree.gif)

### 5. Platoon back-join(w perception)
```sh
python opencda.py -t platoon_town06_carla.py
```
A single CAV will try to overtake several human-drive vehicles to join the platoon from the back side.
![teaser](images/platoon_joining_town06.gif)