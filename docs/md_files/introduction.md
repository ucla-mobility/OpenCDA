## OpenCDA Overview
Current autonomous driving simulation platforms that support scene rendering and traffic simulation mainly concentrate
on single-vehicle intelligence, therefore, developing and testing Cooperative Driving Automation applications (e.g., cooperative perception,
cooperative merge) under a realistic simulated environment becomes difficult.

OpenCDA is created to fill such gap. 

<div class="build-buttons">
<p>
<a href="https://github.com/ucla-mobility/OpenCDA" target="_blank" class="btn btn-neutral" title="Go to the latest OpenCDA page">
<span class="icon icon-github"></span> OpenCDA 0.1.0</a>
</p>
</div>

### Major Components
![](images/OpenCDA_diagrams.png )
 1. <em>Cooperative Driving System</em>: <br /> OpenCDA  provides  a  <strong>full-stack software </strong>  that  contains  the  common  self-driving  modules including
    <strong>sensing,   planning and actuation  layers </strong>,  and  it  is  developed  purely  in  Python for fast prototyping.
    Built  upon these basic modules, OpenCDA supports a range of  <strong>common cooperative  driving  applications</strong>. <br />
 2. <em>Co-Simulation Tools</em>: <br />OpenCDA provides interfaces to integrate multiple open-source simulation tools
 with the cooperative driving system. Through the interfaces, OpenCDA is able to take advantage of the high-quality scene rendering
  and realistic dynamic modelling from <strong>CARLA</strong> , and the realistic traffic simulation from <strong>SUMO</strong>.

 3. <em>Scenario Manager</em>:  <br />By defining the parameters in the Yaml file,  OpenCDA is able to <strong>construct the simulation scenario</strong>,
<strong> creating the traffic flows</strong>, and <strong>assigning various dynamic driving tasks </strong> to different connected automated vehicles. 
 Through such lightweight configuration, researchers can conveniently test and evaluate their algorithms under different scenarios. In the next verision v0.2, 
 OpenScenario will be supported to trigger special events.

### Key Features
The key features of OpenCDA can be summarized as <strong>CIFMB</strong>:
* <strong>C</strong>onnectivity and <strong>C</strong>ooperation: OpenCDA supports various levels and categories of cooperation between CAVs in simulation.
  This differentiates OpenCDA with other single vehicle automation tools.
* <strong>I</strong>ntegration: OpenCDA integrates CARLA and SUMO together for realistic scene rendering, vehicle modeling and traffic simulation.
* <strong>F</strong>ull-stack System: OpenCDA provides a full-stack software system that contains perception, localization, planning, control, and V2X communication modules.
* <strong>M</strong>odularity: OpenCDA is highly modularized, enabling users to conveniently replace any default algorithms or protocols with their own customzied design. 
* <strong>B</strong>enchmark: OpenCDA offers benchmark testing scenarios, state-of-the-art benchmark algorithms for all modules, benchmark testing road maps, and benchmark evaluation metrics.

