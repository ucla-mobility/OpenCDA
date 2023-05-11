.. OpenCDA documentation master file, created by
   sphinx-quickstart on Fri Jul  2 11:48:53 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to OpenCOOD's documentation!
===================================
OpenCOOD is an open-source cooperative detection framework for autonomous driving. It provides SOTA cooperative detection algorithms,
convenient APIs for the large-scale simulated V2V perception dataset `OPV2V <https://mobility-lab.seas.ucla.edu/opv2v/>`_, and a set of useful tools for log replay.

In collaboration with `OpenCDA <https://github.com/ucla-mobility/OpenCDA>`_ , OpenCOOD is mainly focus on offline cooperative perception training and testing. If you are
interested in online cooperative perception and the corresponding closed-loop simulation test, OpenCDA will be the best tool.

OpenCOOD is a work in progress. Many features on the roadmap are being continuously developed. We welcome your contribution and please visit our Github repo
for the latest release.

    `OpenCOOD source code on  Github <https://github.com/DerrickXuNu/OpenCOOD>`_

.. toctree::
   :maxdepth: 1
   :caption: Getting Started

   md_files/data_intro.md
   md_files/installation.md

.. toctree::
   :maxdepth: 1
   :caption: Tutorials

   md_files/config_tutorial.md
   md_files/data_annotation_tutorial.md
   md_files/logic_flow.md

.. toctree::
   :maxdepth: 1
   :caption: Additional Information

   md_files/contributor.md

**Citing OpenCOOD**\ :

If you are using our OpenCOOD framework or codes for your development, please cite the following paper::

    @inproceedings{xu2022opencood,
      author = {Runsheng Xu, Hao Xiang, Xin Xia, Xu Han, Jinlong Li, Jiaqi Ma},
      title = {OPV2V: An Open Benchmark Dataset and Fusion Pipeline for Perception with Vehicle-to-Vehicle Communication},
      booktitle = {2022 IEEE International Conference on Robotics and Automation (ICRA)},
      year = {2022}}

Also, under this LICENSE, OpenCOOD is for non-commercial research only. Researchers can modify the source code for their own research only.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
