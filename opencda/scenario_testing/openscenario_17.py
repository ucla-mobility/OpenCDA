# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla, time
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.constants import Profile, suffix, headline_str
from omegaconf import OmegaConf
from multiprocessing import Process

import scenario_runner as sr


def exec_scenario_runner(scenario_params):
    """
    Execute the ScenarioRunner process

    Parameters
    ----------
    scenario_params: Parameters of ScenarioRunner

    Returns
    -------
    """
    scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
    scenario_runner.run()
    scenario_runner.destroy()


def run_scenario(opt, scenario_params, experiment_params):
    scenario_runner = None
    cav_world = None
    scenario_manager = None
    experiment_profile = Profile.PREDICTION_OPENCOOD_CAV

    # iterate through the profiles
    for profile in experiment_profile.profiles():
        scenario_params = OmegaConf.merge(scenario_params, experiment_params[profile])

    try:
        # Create CAV world
        if experiment_profile in [Profile.PREDICTION_OPENCOOD_SINGLE,
                                  Profile.PREDICTION_OPENCOOD_CAV]:
            cav_world = CavWorld(apply_ml=True,
                                 apply_coperception=True,
                                 coperception_params=scenario_params['coperception'])
            if experiment_profile == Profile.PREDICTION_OPENCOOD_CAV:
                print("Enabling CAV ....")
                config_file = scenario_params.scenario_runner.configFile.split("/")
                filename = config_file[-1][:-len(suffix)]
                prefix_name = "/".join(config_file[0:-1])
                scenario_params.scenario_runner.configFile = f"{prefix_name}/{filename}_cav{suffix}"
        else:
            if experiment_profile == Profile.PREDICTION_YOLO:
                cav_world = CavWorld(True)
            else:
                cav_world = CavWorld(False)
        # Create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town=scenario_params.scenario_runner.town,
                                                   cav_world=cav_world)

        # Create a background process to init and execute scenario runner
        sr_process = Process(target=exec_scenario_runner,
                             args=(scenario_params,))
        sr_process.start()

        world = scenario_manager.world
        ego_vehicle = None
        num_actors = 0
        other_cav_list = []

        while ego_vehicle is None or num_actors < scenario_params.scenario_runner.num_actors:
            print("Waiting for the actors")
            time.sleep(2)
            vehicles = world.get_actors().filter('vehicle.*')
            walkers = world.get_actors().filter('walker.*')
            for vehicle in vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    ego_vehicle = vehicle
                elif vehicle.attributes['role_name'].startswith('cav'):
                    print("CAV found with name: ", vehicle.attributes['role_name'])
                    other_cav_list.append(vehicle)
            num_actors = len(vehicles) + len(walkers)
        print(f'Found all {num_actors} actors')

        if experiment_profile == Profile.PREDICTION_OPENCOOD_CAV:
            single_cav_list = scenario_manager.create_vehicle_manager_openscenario(
                application=['single', 'cooperative'], vehicles=[ego_vehicle] + other_cav_list
            )
        else:
            single_cav_list = scenario_manager.create_vehicle_manager_from_scenario_runner(
                vehicle=ego_vehicle,
            )

        print(headline_str.format(scenario_params.scenario_runner['scenario'], experiment_profile,
                                  scenario_params.scenario_runner['town'], len(other_cav_list)))

        spectator = ego_vehicle.get_world().get_spectator()
        # Bird view following
        spectator_altitude = 50
        spectator_bird_pitch = -90

        while True:
            scenario_manager.tick()
            ego_cav = single_cav_list[0].vehicle

            # Bird view following
            view_transform = carla.Transform()
            view_transform.location = ego_cav.get_transform().location
            view_transform.location.z = view_transform.location.z + spectator_altitude
            view_transform.rotation.pitch = spectator_bird_pitch
            spectator.set_transform(view_transform)

            # Apply the control to the ego vehicle
            for _, single_cav in enumerate(single_cav_list):
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)
            time.sleep(0.01)

    finally:
        if cav_world is not None:
            cav_world.destroy()
        print("Destroyed cav_world")
        if scenario_manager is not None:
            scenario_manager.close()
        print("Destroyed scenario_manager")
        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")
