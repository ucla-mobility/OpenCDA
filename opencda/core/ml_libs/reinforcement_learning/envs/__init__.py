from gym.envs.registration import register, registry
from opencda.core.ml_libs.reinforcement_learning import SIMULATORS

envs = []
env_map = {}

if 'carla' in SIMULATORS:
    from .simple_carla_env import SimpleCarlaEnv
    # from .scenario_carla_env import ScenarioCarlaEnv
    env_map.update({
        "SimpleCarla-v1": 'core.envs.simple_carla_env.SimpleCarlaEnv'
        # "ScenarioCarla-v1": 'core.envs.scenario_carla_env.ScenarioCarlaEnv'
    })

for k, v in env_map.items():
    if k not in registry.env_specs:
        envs.append(k)
        register(id=k, entry_point=v)

if len(envs) > 0:
    print("[ENV] Register environments: {}.".format(envs))
