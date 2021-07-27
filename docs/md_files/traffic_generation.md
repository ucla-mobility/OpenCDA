## Traffic Generation

OpenCDA supports two different ways to generate the background traffic flow: CARLA traffic manager and SUMO traffic simulation. The traffic and other scenario generation function are currently being enhanced and will be released in the future.

---
### CARLA Traffic Manager
OpenCDA support researchers with the CARLA built-in traffic manager to control
the traffic flow. All the test scripts that utilize CARLA traffic manager should
have names ending with 'carla', e.g. single_2lanefree_carla.py, single_town06_carla.py.

To generate the CARLA traffic flow, users need to <strong>define the corresponding parameters
in yaml file </strong> and call the APIs in `opencda.scenario_testing.utils.sim_api.ScenarioManager`:
* Check the carla_traffic_manager section in [yaml rule](yaml_define.html#carla_traffic_manager) to
see how to define parameters related to carla traffic in the yaml file.
* Utilizing `ScenarioManager` to generate CARLA traffic flow is easy. It just takes 3 lines of codes. `scenario_manager.tick()`
will keep the traffic manager keep running during the simulation loop.
```python
import opencda.scenario_testing.utils.sim_api as sim_api
# scenario_params are defined in the yaml file
scenario_manager = sim_api.ScenarioManager(scenario_params,
                                           opt.apply_ml,
                                           xodr_path=xodr_path,
                                           cav_world=cav_world)
# create background traffic in carla
traffic_manager, bg_veh_list = scenario_manager.create_traffic_carla()

while True:
    scenario_manager.tick()

```
---
### Sumo Traffic Management (Co-Simulation)
OpenCDA provides the interface to enable users <strong>control the CAVs in CARLA </strong>
and <strong>manage the traffic flow in Sumo</strong>. All the test scripts that utilize Sumo traffic manager should
have names ending with 'cosim', e.g. single_2lanefree_cosim.py, single_town06_cosim.py.

To generate the Sumo traffic flow, three things are needed:
* Define the Sumo server settings in yaml file. Check our [yaml rule sumo part](yaml_define.html#sumo-optional) to see more details.
* Three files that defines Sumo road network and traffic route: a `xxx.sumocfg` file, 
  a `xxx.net.xml` file, and a `xxx.rou.xml` file. xxx is the name of the testing map, e,g. Town06. This
  name should keep consistent acroos the three files.
  
    *   `.sumocfg` : used to give the Sumo server the path of the network and route xml file. Check
        [.sumocfg file extension](https://fileinfo.com/extension/sumocfg) to see more explanations.
    * `.net.xml` : defines the road graph. Check [Sumo Road Networks](https://sumo.dlr.de/docs/Networks/SUMO_Road_Networks.html)
      to see more details. This xml file can be converted from your xodr file:
  ```bash
  cd root/of/OpenCDA
  python scripts/netconvert_carla.py your_map.xodr -o your_map.net.xml
  ```
   * `rou.xml` : defines the traffic flow. As the example presents below, `vType` is used to define
     the vehicle types and `flow` defines the route. In this example, the server will generate continuous traffic flow
     in the first and second lanes of the road `-63`. Both lanes' vehicles will try to reach road `-62` via road `-61` with
     a speed of 15 m/s. To know more details about the parameters' meaning in route file, check [Sumo Vehicle and Routes](https://sumo.dlr.de/docs/Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.html).
  ```xml
  <routes>
  <vType id="vType_0" minGap="2.00" speedFactor="normc(1.00,0.00)" vClass="passenger" carFollowModel="IDMM" tau="0.6"/>
  <vType id="DEFAULT_VEHTYPE" minGap="2.50" tau="1.0"  color="255,255,255" Class="passenger" accel="0.5"/>
  <flow id="flow_0" begin="0.00" departLane="1" departSpeed="15" departPos="-320" from="-63.0.00" to="-62.0.00" via="-60.0.00" end="4800.00" vehsPerHour="1000.00" type="DEFAULT_VEHTYPE"/>
  <flow id="flow_1" begin="0.2" departLane="0" departSpeed="15" departPos="-320" from="-63.0.00" to="-62.0.00" via="-60.0.00" end="4800.00" vehsPerHour="1000.00" type="DEFAULT_VEHTYPE"/>
  
  </routes>
  
  ```
* Use `CoScenarioManager` to load sumo files and create sumo traffic flow. During initialization, 
  `CoscenarioManager` will setup the Sumo server. During the `tick()` function, `CoScenarioManager` 
  will keep spawning Sumo vechiles as traffic flow.
  
    ```python
  import opencda.scenario_testing.utils.cosim_api as sim_api
  
  # there should be a Town06.sumocfg, a Town06.net.xml, and a Town06.rou.xml in
  # Town06 folder
  sumo_cfg = 'Town06'
  
  # create co-simulation scenario manager
  scenario_manager = \
  sim_api.CoScenarioManager(scenario_params,
                            opt.apply_ml,
                            town='Town06',
                            cav_world=cav_world,
                            sumo_file_parent_path=sumo_cfg)
  
  while True:
        scenario_manager.tick()
    ```

