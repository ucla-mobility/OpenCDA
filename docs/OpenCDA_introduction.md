# OpenCDA 

OpenCDA provides a full-stacksoftware that contains the common self-driving modulescomposed of sensing, computation, and actuation capabili-ties, and it is developed purely in Python. Built ponthese basic modules, OpenCDA supports a range of commoncooperative driving applications, such as platooning, co-operative sensing, cooperative merge, speed harmonization,and so on. More importantly, OpenCDA offers a scenario database that includes various standard scenarios for different cooperative driving applications as benchmarks. Userscan easily replace any predefined guidance algorithms orprotocols in any default module in OpenCDA with theirown designs and test them in the supplied scenarios.

---
## Simulator Platforms
We selected CARLA and SUMO as our multi-resolutionsimulation platforms. CARLA offers realistic scene rendering, vehicle dynamics, and sensor modeling, while SUMO's scope lies in generating large-scale practical background traffic and providing macroscopic level evaluation.

## Customize Scenarios
OpenCDA offers a scenario database that includes various standard scenarios for different cooperative driving applications as benchmarks. Users can easily replace any predefined guidance algorithms or protocols in any default module in OpenCDA with their own designs and test them in the supplied scenarios.

## Modularity 
If users desire to make their own scenarios, OpenCDA also provides a simple API to support such customization. The existing API provide customization in sensor suites, driving automation, vehicle information, simulation information, platoon information, and custome intellogent controller. 
