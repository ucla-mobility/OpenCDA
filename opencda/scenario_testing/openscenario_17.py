# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
# from opencda.scenario_testing.utils.keyboard_listener import KeyListener

import time
from multiprocessing import Process
import psutil

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


def run_scenario(opt, scenario_params):
    scenario_runner = None
    cav_world = None
    scenario_manager = None

    try:
        # Create CAV world
        cav_world = CavWorld(opt.apply_ml)
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

        # key_listener = KeyListener()
        # key_listener.start()

        world = scenario_manager.world
        ego_vehicle = None
        num_actors = 0

        while ego_vehicle is None or num_actors < scenario_params.scenario_runner.num_actors:
            print("Waiting for the actors")
            time.sleep(2)
            vehicles = world.get_actors().filter('vehicle.*')
            walkers = world.get_actors().filter('walker.*')
            for vehicle in vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    ego_vehicle = vehicle
            num_actors = len(vehicles) + len(walkers)
        print(f'Found all {num_actors} actors')

        single_cav_list = scenario_manager.create_vehicle_manager_from_scenario_runner(
            vehicle=ego_vehicle,
        )

        spectator = ego_vehicle.get_world().get_spectator()
        # Bird view following
        spectator_altitude = 60
        spectator_bird_pitch = -90

        while True:
            # if key_listener.keys['esc']:
            #     sr_process.kill()
            #     # Terminate the main process
            #     return
            # if key_listener.keys['p']:
            #     psutil.Process(sr_process.pid).suspend()
            #     continue
            # if not key_listener.keys['p']:
            #     psutil.Process(sr_process.pid).resume()

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

