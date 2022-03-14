## Release History of OpenCDA
This page demonstrates all the changes since the origin release v0.1.0 with more detailed imags.

--- 
### v0.1.2
##### Map manager
OpenCDA now adds a new component `map_manager` for each cav. It will dynamically load road topology, traffic light information, and dynamic
objects information around the ego vehicle and save them into rasterized map, which can be useful for RL planning, HDMap learning, scene understanding, etc.
Key elements in the rasterization map:
- Drivable space colored by black
- Lanes
  - Red lane: the lanes that are controlled by red traffic light
  - Green lane: the lanes that are controlled by green traffic light
  - Yellow lane: the lanes that are not effected by any traffic light
- Objects that are colored by white and represented as rectangle

![](images/map_manager.gif )

---
### v0.1.1
#### Cooperative Perception
OpenCDA now supports data dumping simultaneously for multiple CAVs to develop V2V perception 
algorithms offline. The dumped data includes: 
- LiDAR data
- RGB camera (4 for each CAV)
- GPS/IMU
- Velocity and future planned trajectory of the CAV
- Surrounding vehicles' bounding box position, velocity <br>

Besides the above dumped data, users can also generate the future trajectory for each 
vehicle for trajectory prediction purpose. Run `python root_of_opencda/scripts/generate_prediction_yaml.py`
to generate the prediction offline.

This new functionality has been proved helpful. The newest ICRA 2022 paper <strong>OPV2V: An Open Benchmark Dataset and Fusion Pipeline for Perception with Vehicle-to-Vehicle Communication</strong>
has utilized this new feature to collect cooperative data. Check https://mobility-lab.seas.ucla.edu/opv2v/ for more information

![](images/opv2v.png )

#### CARLA 0.9.12 Support
OpenCDA now supports both CARLA 0.9.12 and 0.9.11. Users needs to set CARLA_VERSION variable before
installing OpenCDA. When users run opencda.py, -v argument is required to classify the CARLA version for
OpenCDA to select the correct API.

#### Weather Parameters
To help estimate the influence of weather on cooperative driving automation, users now can 
define weather setting in the yaml file to control sunlight, fog, rain, wetness and other conditions.

#### Bug Fixes
Some minor bugs in the planning module are fixed.