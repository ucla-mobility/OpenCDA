from gym.envs.registration import register, registry
from opencda.core.ml_libs.rl import SIMULATORS

envs = []
env_map = {}

if 'carla' in SIMULATORS:
    from opencda.core.ml_libs.rl.envs.simple_carla_env_scenario_manager import CarlaRLEnv

    env_map.update({
        "CarlaScenarioManger-v1": 'core.ml_libs.rl.envs.simple_carla_env_scenario_manager'
    })

for k, v in env_map.items():
    if k not in registry.env_specs:
        envs.append(k)
        register(id=k, entry_point=v)

if len(envs) > 0:
    print("[ENV] Register environments: {}.".format(envs))
