.. OpenCDA documentation master file, created by
   sphinx-quickstart on Fri Jul  2 11:48:53 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to OpenCDA's documentation!
===================================
OpenCDA is a **SIMULATION** tool integrated with a prototype cooperative driving automation (CDA; see SAE J3216) pipeline as
well as regular automated driving components (e.g., perception, localization, planning, control). The tool integrates automated driving simulation (CARLA), traffic simulation (SUMO), and Co-simulation (CARLA + SUMO).
OpenCDA builds upon standard automated driving system (ADS) platforms and focuses on various classes of data exchanges and cooperation between vehicles, infrastructure, and other road users (e.g., pedestrians).

OpenCDA is **all in Python**. The purpose is to enable researchers to fast-prototype, simulate, and test CDA algorithms and functions.  By applying our simulation tool, users can conveniently conduct both task-specific evaluation (e.g. object detection accuracy) and pipeline-level assessment (e.g. traffic safety) on their customized algorithms.

In collaboration with `U.S.DOT CDA Research <https://its.dot.gov/cda/>`_ and the `FHWA CARMA Program <https://highways.dot.gov/research/operations/CARMA>`_, OpenCDA, as an open-source project, makes a unique contribution from the perspective of initial-stage development and testing using simulation. OpenCDA is designed and built to support **initial algorithmic testing** for CDA Features. Through collaboration with CARMA Collaborative, this tool provides a unique capability to the CDA research community and will interface with the `CARMA XiL tools <https://github.com/usdot-fhwa-stol/carma-simulation>`_ being developed by U.S.DOT to support more advanced simulation testing of CDA Features.

OpenCDA is a work in progress. Many features on the roadmap are being continuously developed. We welcome your contribution and please visit our Github repo
for the latest release.

    `OpenCDA source code on  Github <https://github.com/ucla-mobility/OpenCDA>`_

.. toctree::
   :maxdepth: 1
   :caption: Getting Started

   md_files/introduction.md
   md_files/installation.md
   md_files/getstarted.md

.. toctree::
   :maxdepth: 2
   :caption: User Tutorial

   md_files/logic_flow.md
   md_files/traffic_generation.md

.. toctree::
   :maxdepth: 1
   :caption: Developer Tutorial

   md_files/developer_tutorial.md
   md_files/customization.md

.. toctree::
   :maxdepth: 1
   :caption: Additional Information

   md_files/contributor.md
   md_files/release_history.md
   modules



**Citing OpenCDA**\ :

If you are using our OpenCDA framework or codes for your development, please cite the following paper::

    @inproceedings{xu2021opencda,
      author = {Runsheng Xu, Yi Guo, Xu Han, Xin Xia, Hao Xiang, Jiaqi Ma},
      title = {OpenCDA:  An  Open  Cooperative  Driving  Automation
      Framework Integrated  with  Co-Simulation},
      booktitle = {2021 IEEE Intelligent Transportation Systems Conference (ITSC)},
      year = {2021}}

Our paper can be accessed by arxiv: https://arxiv.org/abs/2107.06260

Also, under this LICENSE, OpenCDA is for non-commercial research only. Researchers can modify the source code for their own research only. Contracted work that generates corporate revenues and other general commercial use are prohibited under this LICENSE. See the LICENSE file for details and possible opportunities for commercial use.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
