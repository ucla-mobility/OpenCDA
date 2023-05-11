## Tutorial 2:  Data Annotation Introduction

---
We save all groundtruth annotations per agent per timestamp in the yaml files. For instance,
`2021_08_24_21_29_28/4805/000069.yaml` refers to the data annotations with the perspective of te
agent 4805 at timestamp 69 in the scenario database `2021_08_24_21_29_28`. Here we go through an example:

```yaml
camera0: # parameters for frontal camera
  cords: # the x,y,z,roll,yaw,pitch under CARLA map coordinate
  - 141.35067749023438
  - -388.642578125
  - 1.0410505533218384
  - 0.07589337974786758
  - 174.18048095703125
  - 0.20690691471099854
  extrinsic: # extrinsic matrix from camera to LiDAR
  - - 0.9999999999999999
    - -5.1230071481984265e-18
    - 9.322129061605055e-20
    - -2.999993025731527
  - - -2.5011383190939924e-18
    - 1.0
    - 1.1458579204685086e-19
    - -3.934422863949294e-06
  - - 2.7713237218713775e-20
    - 3.7310309839064755e-20
    - 1.0
    - 0.8999999040861146
  - - 0.0
    - 0.0
    - 0.0
    - 1.0
  intrinsic: # camera intrinsic matrix
  - - 335.639852470912
    - 0.0
    - 400.0
  - - 0.0
    - 335.639852470912
    - 300.0
  - - 0.0
    - 0.0
    - 1.0
camera1: ... # params of right rear camera
camera2: ... # params of left rear camera
canera3: ... # params of back camera
ego_speed: 18.13 # agent's current speed, km/h
lidar_pose: # LiDAR pose under CARLA map coordinate system
- 144.33
- -388.94
- 1.93
- 0.078
- 174.18
- 0.21
plan_trajectory: # agent's planning trajectory
- - 140.
  - -388
  - 87
predicted_ego_pos: # agent's localization (x,y,z,roll,yaw,pitch) gained from GPS
- 143.78
- -388.94
- 0.036
- 0.080
- -185.95
- 0.18
true_ego_pos: # agent's true localization
- 143.83
- -388.89
- 0.032
- 0.075
- 174.18
- 0.21
vehicles: # the surrounding vehicles that have at least one LiDAR point hit from the agent
  4796: # the id of the vehicle (i.e. object)
    angle: # roll, yaw, pitch under CARLA map coordinate system
    - 0.096
    - -177.86
    - 0.197
    center: # the relative position from bounding box center to the frontal axis of this vehicle
    - 0.0004
    - 0.0005
    - 0.71
    extent: # half length, width and height of the vehicle in meter
    - 2.45
    - 1.06
    - 0.75
    location: # x, y ,z position of the center in the frontal axis of the vehicle under CARLA map coordinate system
    - 158.55
    - -385.75
    - 0.032
    speed: 19.47 # vehicle's speed
  4880: ...
```

