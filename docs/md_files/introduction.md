## OpenCDA Overview
Current autonomous driving simulation platforms that support scene rendering and traffic simulation mainly concentrate
on single-vehicle intelligence; therefore, developing and testing Cooperative Driving Automation applications (e.g., cooperative perception, platooning, 
signalized intersection appriach and departure) under a realistic simulated environment becomes difficult.

OpenCDA is created to fill such gaps. 

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
  and realistic dynamic modelling from <strong>CARLA</strong> , and the realistic traffic simulation from <strong>SUMO</strong>. Also, CARLA only and SUMO only mode of OpenCDA also offers researchers the flexibility to test vehicle-level and traffic-level performance, respectively.

 3. <em>Scenario Manager</em>:  <br />By defining the parameters in the Yaml file,  OpenCDA is able to <strong>construct the simulation scenario</strong>,
<strong> creating the traffic flows</strong>, and <strong>assigning various dynamic driving tasks </strong> to different connected automated vehicles. 
 Through such lightweight configuration, researchers can conveniently test and evaluate their algorithms under different scenarios. In the next verision v0.2, 
 OpenScenario will be supported to trigger special events.

### Key Features
The key features of OpenCDA are:
* <strong>Full-stack CDA System</strong>: OpenCDA provides a full-stack automated driving and cooperative driving software system that contains perception, localization, planning, control, and V2X communication modules.
* <strong>Connectivity and Cooperation</strong>: OpenCDA supports various levels and categories of cooperation between CAVs in simulation. This differentiates OpenCDA with other single vehicle automation tools.
  This differentiates OpenCDA with other single vehicle automation tools.
* <strong>Integration</strong>: OpenCDA utilizes CARLA and SUMO separately, as well as integrates them together for realistic scene rendering, vehicle modeling, and traffic simulation. It is also efficient and flexible enough to be applied for software-in-the-loop and hardware-in-the-loop testing.
* <strong>Modularity</strong>: OpenCDA is highly modularized, enabling users to conveniently replace any default algorithms or protocols with their own customzied design. 
* <strong>Benchmark</strong>: OOpenCDA offers benchmark testing scenarios, benchmark baseline maps, state-of-the-art benchmark algorithms for ADS and Cooperative ADS functions, and benchmark evaluation metrics.

