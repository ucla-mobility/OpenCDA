## Scenario Runner

To enable the scenario runner, follow the official docs: 
https://carla-scenariorunner.readthedocs.io/en/latest/ to set up the environments.

### Details for each step:
1. Clone the Scenario Runner repo. Find the match version with your installed Carla. We'll use `0.9.12` as the following example
2. The `SCENARIO_RUNNER_ROOT="/to/scenario_runner/installation/path"`. In our example, we use `${HOME}/scenario_runner-0.9.12`
3. Set up the environment in `.bashrc` like
```
export SCENARIO_RUNNER_ROOT=${HOME}/scenario_runner-0.9.12
export PYTHONPATH=$SCENARIO_RUNNER_ROOT:$PYTHONPATH
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
```
4. source your environment file as `source .bashrc`
5. create an empty `__init__.py` under the scenario runner repo so that the folder is treated as a package

### Verify the installation
1. verify with the import
```
python -c 'import scenario_runner'
```
2. Verify with python prompt
```
>> import scenario_runner as sr
>> dir(sr)
['CarlaDataProvider', 'LooseVersion', 'OpenScenario', 'OpenScenarioConfiguration', 'RawTextHelpFormatter', 'RouteParser', 'RouteScenario', 'ScenarioConfigurationParser', 'ScenarioManager', 'ScenarioRunner', 'VERSION', '__builtins__', '__cached__', '__doc__', '__file__', '__loader__', '__name__', '__package__', '__spec__', 'argparse', 'carla', 'datetime', 'glob', 'importlib', 'inspect', 'json', 'main', 'os', 'pkg_resources', 'print_function', 'signal', 'sys', 'time', 'traceback']
```

