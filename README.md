# OpenCDA
[![Build Status](https://travis-ci.com/ucla-mobility/OpenCDA.svg?branch=develop)](https://travis-ci.com/ucla-mobility/OpenCDA)
[![Coverage Status](https://coveralls.io/repos/github/ucla-mobility/OpenCDA/badge.svg?branch=feature/readme_revise)](https://coveralls.io/github/ucla-mobility/OpenCDA?branch=feature/readme_revise)
[![Documentation Status](https://readthedocs.org/projects/opencda-documentation/badge/?version=latest)](https://opencda-documentation.readthedocs.io/en/latest/?badge=latest)



OpenCDA is a generalized framework for fast prototyping <strong>cooperative driving automation 
applications</strong>(e.g., cooperative perception, platooning) as well as <strong>autonomous vehicle components</strong>(e.g., 
perception, localization, planning, control) under <strong>Co-simulation</strong>(CARLA and SUMO).

The key features of OpenCDA can be summarized as <strong>CIFMB</strong>:
* <strong>C</strong>onnectivity and <strong>C</strong>ooperation: OpenCDA supports various levels and categories of cooperation between CAVs in simulation.
  This differentiates OpenCDA with other single vehicle automation tools.
* <strong>I</strong>ntegration: OpenCDA integrates CARLA and SUMO together for realistic scene rendering, vehicle modeling and traffic simulation.
* <strong>F</strong>ull-stack System: OpenCDA provides a full-stack software system that contains perception, localization, planning, control, and V2X communication modules.
* <strong>M</strong>odularity: OpenCDA is highly modularized, enabling users to conveniently replace any default algorithms or protocols with their own customzied design. 
* <strong>B</strong>enchmark: OpenCDA offers benchmark testing scenarios, state-of-the-art benchmark algorithms, and benchmark evaluation metrics.

![teaser](docs/md_files/images/platoon_joining_2lanefree_complete.gif)
 
Users could refer to [OpenCDA documentation](https://opencda-documentation.readthedocs.io/en/latest/) to see more details.

## Major Components
![teaser](docs/md_files/images/OpenCDA_diagrams.png )

OpenCDA  is composed of three major component: <strong>Cooperative Driving System</strong>,  <strong>Co-Simulation Tools</strong>,
and  <strong>Scenario Manager</strong>.

Check the [OpenCDA Introduction](https://opencda-documentation.readthedocs.io/en/latest/md_files/introduction.html) for more details.

## Get Started
* [Overview](https://opencda-documentation.readthedocs.io/en/latest/md_files/introduction.html)
* [Installation](https://opencda-documentation.readthedocs.io/en/latest/md_files/installation.html)
* [Quick Start](https://opencda-documentation.readthedocs.io/en/latest/md_files/getstarted.html)

### Users Guide
* [Logic Flow](https://opencda-documentation.readthedocs.io/en/latest/md_files/logic_flow.html)
* [Traffic Generation](https://opencda-documentation.readthedocs.io/en/latest/md_files/traffic_generation.html)


Note: We are keeping improving the performance of OpenCDA. Currently, it is mainly tested in our 2-lane highway customized map and
 Carla town06 map, therefore, we <strong>DO NOT </strong> guarantee the same level of  robustness in other maps.

### Developer Guide

*  [Class Design](https://opencda-documentation.readthedocs.io/en/latest/developer_tutorial.html)
*  [Customize Your Algorithms](https://opencda-documentation.readthedocs.io/en/latest/customization.html)
*  [API Reference](https://opencda-documentation.readthedocs.io/en/latest/modules.html) <br>


### Contribution Rule
We welcome your contributions.
- Please report bugs and improvements by submitting issues.
- Submit your contributions using [pull requests](https://github.com/ucla-mobility/OpenCDA/pulls).
 Please use [this template](.github/PR_TEMPLATE.md) for your pull requests.
 
 ## Citation
 If you are using our framework for your development, please cite the following paper:
 ```bibtex
@inproceedings{xu2021opencda,
title={OpenCDA:  An  Open  Cooperative  Driving  Automation  Framework
Integrated  with  Co-Simulation},
author={Runsheng Xu, Yi Guo, Xu Han, Xin Xia, Hao Xiang, Jiaqi Ma},
booktitle={2021 IEEE Intelligent Transportation Systems Conference (ITSC)},
year={2021}
}
```
And the arxiv link to the paper:  https://arxiv.org/abs/2107.06260

## 2021 RoadMap
![teaser](docs/md_files/images/roadmap.PNG)

## Contributors
OpenCDA is supported by the [UCLA Mobility Lab](https://mobility-lab.seas.ucla.edu/). <br>


### Project Leader: <br>
 Runsheng Xu ([linkedin](https://www.linkedin.com/in/runsheng-xu/), [github](https://github.com/DerrickXuNu))  <br>
 
### Major developers: 
 - Runsheng Xu([linkedin](https://www.linkedin.com/in/runsheng-xu/), [github](https://github.com/DerrickXuNu))
 - Xu Han([linkedin](https://linkedin.com/in/xu-han-12851a64), [github](https://github.com/xuhan417))
 - Hao Xiang([linkedin](https://www.linkedin.com/in/hao-xiang-42bb5a1b2/), [github](https://github.com/XHwind))

### Theoretical supports:
 - Dr. Yi Guo([linkedin](https://www.linkedin.com/in/yi-guo-4008baaa/))
 - Dr. Xin Xia([linkedin](https://www.linkedin.com/in/yi-guo-4008baaa/))
 
### Lab Supervisor
- Dr. Jiaqi Ma([linkedin](https://www.linkedin.com/in/jiaqi-ma-17037838/),
               [google scholar](https://scholar.google.com/citations?user=S3cQz1AAAAAJ&hl=en))
