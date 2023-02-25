# Data Preparation

---
OpenCOOD support both **OPV2V (ICRA2022)** and **V2XSet (ECCV2022)** dataset.

## OPV2V
All the data can be downloaded from [google drive](https://drive.google.com/drive/folders/1dkDeHlwOVbmgXcDazZvO6TFEZ6V_7WUu). If you have a good internet, you can directly
download the complete large zip file such as `train.zip`. In case you suffer from downloading large fiels, we also split each data set into small chunks, which can be found 
in the directory ending with `_chunks`, such as `train_chunks`. After downloading, please run the following command to each set to merge those chunks together:
```python
cat train.zip.part* > train.zip
unzip train.zip
```
After downloading is finished, please make the file structured as following:

```sh
OpenCOOD # root of your OpenCOOD
├── opv2v_data_dumping # the downloaded opv2v data
│   ├── train
│   ├── validate
│   ├── test
│   ├── test_culvercity
├── opencood # the core codebase

```


OPV2V data is structured as following:

```sh
opv2v_data_dumping
├── train # data for training
│   ├── 2021_08_22_21_41_24  # scenario folder
│     ├── data_protocol.yaml # the simulation parameters used to collect the data in Carla
│     └──  1732 # The connected automated vehicle's id 
│       └── 00000.pcd - 00700.pcd # the point clouds data from timestamp 0 to 700
│       ├── 00000.yaml - 00700.yaml # corresponding metadata for each timestamp
│       ├── 00000_camera0.png - 00700_camera0.png # frontal camera images
│       ├── 00000_camera1.png - 00700_camera1.png # right rear camera images
│       ├── 00000_camera2.png - 00700_camera2.png # left rear camera images
│       └── 00000_camera3.png - 00700_camera3.png # back camera images
├── validate  
├── test
├── test_culvercity
```

### 1. Data Split
OPV2V dataset can be divided into 4 different folders: `train`, `validation`, `test`, and `test_culvercity`
- `train`: contains all training data
- `validate`: used for validation during training
- `test`: test set for calrla default towns
- `test_culvercity`: test set for the digital town of Culver City, Los Angeles

### 2. Scenario Database
OPV2V has 73 scenarios in total, where each of them contains data stream from different agents across different timestamps.
Each scenario is named by the time it was gathered, e.g., `2021_08_22_21_41_24`.

### 3. Agent Contents
Under each scenario folder,  the data of every intelligent agent~(i.e. connected automated vehicle) appearing in the current scenario is saved in different folders. Each folder is named by the agent's unique id, e.g., 1732.

In each agent folder, data across different timestamps will be saved. Those timestamps are represented by five digits integers
as the prefix of the filenames (e.g., 00700.pcd). There are three types of files inside the agent folders: LiDAR point clouds (`.pcd` files), camera images (`.png` files), and metadata (`.yaml` files).

#### 3.1 Lidar point cloud
The LiDAR data is saved with Open3d package and has a postfix ".pcd" in the name. 

#### 3.2 Camera images
Each CAV is equipped with 4 RGB cameras (check https://mobility-lab.seas.ucla.edu/opv2v/ to see the mounted positions of these cameras) to capture the 360 degree of view of the surrounding scene.`camera0`, `camera1`, `camera2`, and `camera3` represent the front, right rear, left rear, and back cameras respectively.

#### 3.3  Data Annotation
All the metadata is saved in yaml files. It records the following important information at the current timestamp:
- **ego information**:  Current ego pose with and without GPS noise under Carla world coordinates, ego speed in km/h, the LiDAR pose, and future planning trajectories. 
- **calibration**: The intrinsic matrix and extrinsic matrix from each camera to the LiDAR sensor.
- **objects annotation**: The pose and velocity of each surrounding human driving vehicle that has at least one point hit by the agent's LiDAR sensor. See [data annotation section](data_annotation_tutorial.md) for more details. 

### 4. Data Collection Protocol
Besides agent contents, every scenario database also has a yaml file named `data_protocol.yaml`. 
This yaml file records the simulation configuration to collect the current scenario. It is used to log replay
the data and enable users to add/modify sensors for new tasks without changing the original events.

To help users collect customized data in CARLA (e.g., different sensor configurations and modalities) with similar data format and structure, we have released our OPV2V data collection code in the `feature/data_collection` branch of [OpenCDA](https://github.com/ucla-mobility/OpenCDA/tree/feature/data_collection). Users can refer to its [documentation](https://opencda-documentation.readthedocs.io/en/latest/md_files/introduction.html) for detailed instructions.

---
## V2XSet
The data can be found from [google url](https://drive.google.com/drive/folders/1r5sPiBEvo8Xby-nMaWUTnJIPK6WhY1B6?usp=sharing).  Since the data for train/validate/test
is very large, we  split each data set into small chunks, which can be found in the directory ending with `_chunks`, such as `train_chunks`. After downloading, please run the following command to each set to merge those chunks together:
```
cat train.zip.part* > train.zip
unzip train.zip
```

### 1. Structure
After downloading is finished, please make the file structured as following:

```sh
OpenCOOD # root of opencood
├── v2xset # the downloaded v2xset data
│   ├── train
│   ├── validate
│   ├── test
├── opencood # the core codebase
```

### 2. Label format
V2XSet's data label format is nearly the same with OPV2V, except:
- OPV2V only has vehicles while V2XSet has infrastructure sensors
- All the infrastructure sensors' folder is named with negative integer, e.g. `-1`

---
## Adding Noise to OPV2V and V2XSet

Cooperative perception faces the challenge of GPS error and communication delay. Our OpenCOOD allows users 
to add realistic GPS error and communication delay to the dataset.

To add noise to both OPV2V and V2XSet, just add the following parameters to the model yaml file:
```
wild_setting: # setting related to noise
  async: true
  async_mode: 'sim'
  async_overhead: 100
  backbone_delay: 10
  data_size: 1.06
  loc_err: true
  ryp_std: 0.2
  seed: 25
  transmission_speed: 27
  xyz_std: 0.2
```
`async`: whether add communication delay. <br>
`aysnc_mode`:  sim or real mode. In sim mode, the delay is a constant while in real mode, the delay has a uniform distribution.
The major experiment in the paper used sim mode whereas the 'Effects of transmission size' study used real
 mode. <br>
`async_overhead`: the communication delay in ms. In sim mode, it represents a constant number. In real mode,
the systematic async will be a random number from 0 to `aysnc_overhead`. <br>
`backbone_delay`: an estimate of backbone computation time. Only useful in real mode. <br>
`data_size`: transmission data size in Mb. Only used in real mode. <br>
`transmission_speed`: data transmitting speed during communication. By default 27 Mb/s. Only used in real mode. <br>
`loc_err`: whether to add localization error. <br>
`xyz_std`: the standard deviation of positional GPS error. <br>
`ryp_std`: the standard deviation of angular GPS error. <br>
`seed`: random seed for noise simulation. <strong>please keep it as 25 during testing </strong>.

To see a complete example, refer to [configuration tutorial section](config_tutorial.md)