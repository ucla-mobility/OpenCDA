# OpenCDA
[![Build Status](https://travis-ci.com/ucla-mobility/OpenCDA.svg?branch=develop)](https://travis-ci.com/ucla-mobility/OpenCDA)
[![Coverage Status](https://coveralls.io/repos/github/ucla-mobility/OpenCDA/badge.svg?branch=feature/readme_revise)](https://coveralls.io/github/ucla-mobility/OpenCDA?branch=feature/readme_revise)
[![Documentation Status](https://readthedocs.org/projects/opencda-documentation/badge/?version=latest)](https://opencda-documentation.readthedocs.io/en/latest/?badge=latest)


OpenCDA is a <strong>SIMULATION</strong> tool integrated with a prototype cooperative driving automation (CDA; see [SAE J3216](https://www.sae.org/standards/content/j3216_202005/)) pipeline as
well as regular automated driving components (e.g., perception, localization, planning, control). The tool integrates automated driving simulation (CARLA), traffic simulation (SUMO), and Co-simulation (CARLA + SUMO). 

OpenCDA is <strong>all in Python</strong>. The purpose is to enable researchers to fast-prototype, simulate, and test CDA algorithms and functions.  By applying our simulation tool, users can conveniently conduct both task-specific evaluation (e.g. object detection accuracy) and pipeline-level assessment (e.g. traffic safety) on their customized algorithms.

Inspired by the [USDOT CDA CARMA program](https://its.dot.gov/cda/), OpenCDA, as an open-source project, makes a unique contribution from the perspective of initial-stage development and testing using simulation. 
Users can consider using OpenCDA for <strong>initial algorithmic testing</strong>, and should use the [CARMA everything-in-the-loop](https://github.com/usdot-fhwa-stol/carma-simulation) evaluation tool and [CARMA platform](https://github.com/usdot-fhwa-stol/carma-platform) for software platform development and field testing.

The key features of OpenCDA are:
* <strong>Integration</strong>: OpenCDA utilizes CARLA and SUMO separately, as well as integrates them together for realistic scene rendering, vehicle modeling, and traffic simulation.
* <strong> Full-stack</strong> prototype CDA Platform in Simulation: OpenCDA provides a simple prototype automated driving and cooperative driving platform, all in Python, that contains perception, localization, planning, control, and V2X communication modules.
* <strong>Modularity</strong>: OpenCDA is highly modularized, enabling users to conveniently replace any default algorithms or protocols with their own customized design. 
* <strong>Benchmark</strong>: OpenCDA offers benchmark testing scenarios, benchmark baseline maps, state-of-the-art benchmark algorithms for ADS and Cooperative ADS functions, and benchmark evaluation metrics.
* <strong>Connectivity and Cooperation</strong>: OpenCDA supports various levels and categories of cooperation between CAVs in simulation. This differentiates OpenCDA from other single vehicle simulation tools.

 
Users could refer to [OpenCDA documentation](https://opencda-documentation.readthedocs.io/en/latest/) for more details.

## Major Components
![teaser](docs/md_files/images/OpenCDA_diagrams.png )

OpenCDA  consists of three major component: <strong>Cooperative Driving System</strong>,  <strong>Co-Simulation Tools</strong>,
and  <strong>Scenario Manager</strong>.

Check the [OpenCDA Introduction](https://opencda-documentation.readthedocs.io/en/latest/md_files/introduction.html) for more details.


 ## Citation
 If you are using our OpenCDA framework or codes for your development, please cite the following paper:
 ```bibtex
@inproceedings{xu2021opencda,
title={OpenCDA:  An  Open  Cooperative  Driving  Automation  Framework
Integrated  with  Co-Simulation},
author={Runsheng Xu, Yi Guo, Xu Han, Xin Xia, Hao Xiang, Jiaqi Ma},
booktitle={2021 IEEE Intelligent Transportation Systems Conference (ITSC)},
year={2021}
}
```
The arxiv link to the paper:  https://arxiv.org/abs/2107.06260

Also, under this LICENSE, OpenCDA is for non-commercial research only. Researchers can modify the source code for their own research only. Contracted work that generates corporate revenues and other general commercial use are prohibited under this LICENSE. See the LICENSE file for details and possible opportunities for commercial use.
## Get Started

 ![teaser](docs/md_files/images/platoon_joining_2lanefree_complete.gif)


### Users Guide
* [Overview](https://opencda-documentation.readthedocs.io/en/latest/md_files/introduction.html)
* [Installation](https://opencda-documentation.readthedocs.io/en/latest/md_files/installation.html)
* [Quick Start](https://opencda-documentation.readthedocs.io/en/latest/md_files/getstarted.html)
* [Logic Flow](https://opencda-documentation.readthedocs.io/en/latest/md_files/logic_flow.html)
* [Traffic Generation](https://opencda-documentation.readthedocs.io/en/latest/md_files/traffic_generation.html)


Note: We continuously improve the performance of OpenCDA. Currently, it is mainly tested in our customized maps and
 Carla town06 map; therefore, we <strong>DO NOT </strong> guarantee the same level of  robustness in other maps.

### Developer Guide

*  [Class Design](https://opencda-documentation.readthedocs.io/en/latest/md_files/developer_tutorial.html)
*  [Customize Your Algorithms](https://opencda-documentation.readthedocs.io/en/latest/md_files/customization.html)
*  [API Reference](https://opencda-documentation.readthedocs.io/en/latest/modules.html) <br>


### Contribution Rule
We welcome your contributions.
- Please report bugs and improvements by submitting issues.
- Submit your contributions using [pull requests](https://github.com/ucla-mobility/OpenCDA/pulls).
 Please use [this template](.github/PR_TEMPLATE.md) for your pull requests.
 
## In OpenCDA v0.1.0 Release
The current version features the following:
* OpenCDA v0.1.0 software stack (basic ADS and cooperative ADS platform, benchmark algorithms for platooning, cooperative lane change, merge, and other freeway maneuvers)
* CARLA only simulation
* Co-Simulation function with CARLA + SUMO
* Scenario manager and scenario database for CDA freeway applications


## In Future Releases
Future versions are expected to include the following:
* OpenCDA v0.2.0 and above software stack, including signalized intersection and corridor applications, cooperative perception and localization, enhanced scenario generation/manager and scenario database for newly added CDA applications)
* SUMO only simulation which includes SUMO implementation of all cooperative driving applications using behavior based approach (consistent with CARLA implementation)
* Software-in-the-loop interfaces with two open-source ADS platforms, i.e., Autoware and CARMA
* hardware-in-the-loop interfaces and example projects with a real automated driving vehicle platform and a driving simulator

<!-- ## 2021 RoadMap
![teaser](docs/md_files/images/roadmap.PNG)
-->

## Contributors
OpenCDA is supported by the [UCLA Mobility Lab](https://mobility-lab.seas.ucla.edu/). <br>

### Lab Principal Investigator:
- Dr. Jiaqi Ma ([linkedin](https://www.linkedin.com/in/jiaqi-ma-17037838/),
               [UCLA Samueli](https://samueli.ucla.edu/people/jiaqi-ma/))

### Project Lead: <br>
 - Runsheng Xu ([linkedin](https://www.linkedin.com/in/runsheng-xu/), [github](https://github.com/DerrickXuNu))  <br>
 
### Team Members: 
 - Xu Han ([linkedin](https://linkedin.com/in/xu-han-12851a64), [github](https://github.com/xuhan417))
 - Hao Xiang ([linkedin](https://www.linkedin.com/in/hao-xiang-42bb5a1b2/), [github](https://github.com/XHwind))
 - Dr. Yi Guo ([linkedin](https://www.linkedin.com/in/yi-guo-4008baaa/))
 - Dr. Xin Xia ([linkedin](https://www.linkedin.com/in/yi-guo-4008baaa/))
 

