# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import carla
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld

import time
from multiprocessing import Process

import scenario_runner as sr

def exec_scenario_runner(scenario_params):
    """
    Execute the SenarioRunner process

    Parameters
    ----------
    scenario_params: Parameters of ScenarioRunner

    Returns
    -------
    """
    scenario_runner = sr.ScenarioRunner(scenario_params.scenario)
    scenario_runner.run()
    scenario_runner.destroy()


def exec_ego_vehicle_runner(scenario_params):
    """
    Execute the ego vehicle runner

    Parameters
    ----------
    scenario_params: Parameters of ScenarioRunner

    Returns
    -------
    """
    scenario_runner = None
    cav_world = None
    scenario_manager = None
    client = None

    try:
        client_host = scenario_params.world.client_host
        client_port = scenario_params.world.client_port
        client = carla.Client(client_host, client_port)
        world = client.get_world()

        player = None
        leading = None

        while player is None:
            print("Waiting for the ego vehicle...")
            time.sleep(2)
            possible_vehicles = world.get_actors().filter('vehicle.*')
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    player = vehicle
                if vehicle.attributes['role_name'] == 'scenario':
                    print("Leading vehicle found")
                    leading = vehicle
                
        spectator = player.get_world().get_spectator()
        # Bird view following
        spectator_altitude = 80
        spectator_bird_pitch = -90
        
        while True:
            world.tick()
            
            # Get the control of the ego vehicle
            control = leading.get_control()
            # Get the distance between the ego and leading vehicles
            distance = leading.get_location().distance(player.get_location())
            # Get the velocity of the ego vehicle on the x-y plane
            ego_velocity = math.sqrt(player.get_velocity().x ** 2 + player.get_velocity().y ** 2)
            leading_velocity = math.sqrt(leading.get_velocity().x ** 2 + leading.get_velocity().y ** 2)
            
            # Wait for the leading vehicle to start
            if leading_velocity < 0.1: 
                control = carla.VehicleControl(throttle=0.50000, steer=0, brake=0.000000, hand_brake=False,
                                            reverse=False, manual_gear_shift=False, gear=0)
            # If the leading vehicle is too close, stop the ego vehicle
            elif distance < 1.7 * ego_velocity or distance < 10:
                control = carla.VehicleControl(throttle=0, steer=0, brake=1.0, hand_brake=True,
                                            reverse=False, manual_gear_shift=False, gear=0)
            
            print("Distance: {}, Ego velocity: {}, Leading velocity: {}".format(distance, ego_velocity, leading_velocity))
            # Bird view following
            view_transform = carla.Transform()
            view_transform.location = player.get_transform().location
            view_transform.location.z = view_transform.location.z + spectator_altitude
            view_transform.rotation.pitch = spectator_bird_pitch
            spectator.set_transform(view_transform)

            # Apply the control to the ego vehicle
            player.apply_control(control)
            time.sleep(0.05)
    finally:
        if cav_world is not None:
            cav_world.destroy()
        print("Destroyed cav_world")
        if scenario_manager is not None:
            scenario_manager.destroyActors()
            scenario_manager.close()
        if scenario_runner is not None:
            scenario_runner.destroy()
            del scenario_runner
        print("Destroyed scenario_runner...")


def run_scenario(opt, scenario_params):
    try:
        # Create CAV world
        cav_world = CavWorld(opt.apply_ml)
        # Create scenario manager
        sim_api.ScenarioManager(scenario_params,
                                opt.apply_ml,
                                opt.version,
                                town=scenario_params.scenario.town,
                                cav_world=cav_world)

        # Create a background process to init and execute scenario runner
        sr_process = Process(target=exec_scenario_runner, args=(scenario_params, ))
        # Create a parallel process to init and run EI-Drive
        ei_process = Process(target=exec_ego_vehicle_runner, args=(scenario_params, ))

        sr_process.start()
        ei_process.start()
        
        sr_process.join()
        ei_process.join()
        


    except Exception as e:
        print("Error: {}".format(e))
