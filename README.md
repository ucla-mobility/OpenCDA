# OpenCDA
[![Build Status](https://travis-ci.com/ucla-mobility/OpenCDA.svg?token=z7KWtezdxQBPWn78zjmz&branch=develop)
[![Coverage Status](https://coveralls.io/repos/github/ucla-mobility/OpenCDA/badge.svg?branch=feature/readme_revise)](https://coveralls.io/github/ucla-mobility/OpenCDA?branch=feature/readme_revise)
[![Documentation Status](https://readthedocs.org/projects/opencda-documentation/badge/?version=latest)](https://opencda-documentation.readthedocs.io/en/latest/?badge=latest)


OpenCDA is a generalized framework for fast developing and testing <strong>cooperative driving automation 
applications</strong>(e.g., cooperative perception, platooning) as well as <strong>autonomous vehicle components</strong>(e.g., 
perception, localization, planning, control) on <strong>multi-resolution simulators</strong>(e.g., CARLA, SUMO, NS3).

OpenCDA is still under development, and many features are still in the future roadmap. 
We welcome your contributions!

Users could refer to [OpenCDA documentation](https://opencda-documentation.readthedocs.io/en/latest/) to see more details.

## Major Components
![teaser](docs/md_files/images/OpenCDA_diagrams.png )

OpenCDA  is composed of three major component: <strong>Cooperative Driving System</strong>,  <strong>Co-Simulation Tools</strong>,
and  <strong>Scenario Manager</strong>.

Check the [OpenCDA Introduction](https://opencda-documentation.readthedocs.io/en/latest/OpenCDA_introduction/) for more details.

## Get Started
![teaser](docs/md_files/images/platoon_joining_2lanefree_complete.gif)
### Users Guide
1. [Installation](https://opencda-documentation.readthedocs.io/en/latest/OpenCDA_installation)
1. [Quick Start](https://opencda-documentation.readthedocs.io/en/latest/OpenCDA_getstarted)
1. [Tutorials](https://opencda-documentation.readthedocs.io/en/latest/OpenCDA_tutorial/)
1. [Python API](https://opencda-documentation.readthedocs.io/en/latest/PythonAPI/)

### Developer Guide
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
author={Runsheng Xu, Yi Guo, Xu Han, Xin Xia, Jiaqi Ma},
booktitle={2021 IEEE Intelligent Transportation Systems Conference (ITSC)},
year={2021}
}
```

## Future Plans(v0.2)
- [x] Sumo code prototyping
- [ ] Clean Sumo code
- [ ] ns3 integration
- [ ] Cooperative Perception application
- [ ] Cooperative Localization application

## Contributors
OpenCDA is supported by the [UCLA Mobility Lab](https://mobility-lab.seas.ucla.edu/). <br>
 Major developers: 
 - Runsheng Xu([linkedin](https://www.linkedin.com/in/runsheng-xu/), [github](https://github.com/DerrickXuNu))
 - Xu Han([linkedin](linkedin.com/in/xu-han-12851a64), [github](https://github.com/xuhan417))
