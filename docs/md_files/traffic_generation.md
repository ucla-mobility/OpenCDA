## Traffic Generation in OpenCDA

OpenCDA supports two different ways to generate the traffic flow: CARLA
traffic manager and Sumo traffic simulation.

---
### CARLA Traffic Manager
OpenCDA support researchers use the CARLA built-in traffic manager to control
the traffic flow. All the test scripts that utilize CARLA traffic manager should
have names ending with 'carla', e.g., single_2lanefree_carla.py, single_town06_carla.py.

To generate the CARLA traffic flow, users need to <strong>define the corresponding parameters
in yaml file </strong> and call the APIs in `opencda.scenario_testing.utils.sim_api.ScenarioManager`:
* Check [the carla_traffic_manager in yaml rule section](yaml_define.md#carla_traffic_manager) to
see how to define parameters related to carla traffic in the yaml file.
* Utilize `ScenarioManager` to generate CARLA traffic flow is easy. It just takes 3 lines of codes. `scenario_manager.tick()`
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
OpenCDA provides the interface to enable users <strong> control the CAVs in CARLA </strong>
and <strong>manage the traffic flow in Sumo </strong>. All the test scripts that utilize CARLA traffic manager should
have names ending with 'cosim', e.g., single_2lanefree_cosim.py, single_town06_cosim.py.

