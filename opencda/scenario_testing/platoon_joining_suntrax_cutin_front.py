# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the
customized 2-lane freeway simplified map sorely with carla
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os
import carla

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import \
    load_yaml


def run_scenario(opt, config_yaml):
    try:
        # first define the path of the yaml file and map file
        scenario_params = load_yaml(config_yaml)
        current_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(
            current_path,
            # /home/carma/OpenCDA/opencda/assets/Suntrax/suntrax_oval_map.xodr
            '../assets/Suntrax/suntrax_oval_map.xodr')

        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   xodr_path=xodr_path)
        if opt.record:
            scenario_manager.client. \
                start_recorder("platoon_joining_suntrax_front.log", True)

        # create platoon members
        platoon_list = \
            scenario_manager.create_platoon_manager(
                map_helper=map_api.spawn_helper_2lanefree,
                data_dump=False)

        # create single cavs
        single_cav_list = \
            scenario_manager.create_vehicle_manager(['platooning'],
                                                    map_api.spawn_helper_2lanefree)

        # create background traffic in carla
        back_traffic = scenario_params['carla_traffic_manager']['background_traffic']
        
        if back_traffic: 
            traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='platoon_joining_suntrax_front',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = platoon_list[0].vehicle_manager_list[1].vehicle

        # run steps
        # headlight blink timer to indicate platooning status
        blink_timer = 0
        while True:
            scenario_manager.tick()
            transform = spectator_vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location +
                    carla.Location(
                        z=80),
                    carla.Rotation(
                        pitch=-
                        90)))
            for platoon in platoon_list:
                platoon.update_information()
                platoon.run_step()
                # flash headlight briefly to indicate joining the platoon
                for cav in platoon.vehicle_manager_list:
                    # check light state
                    light_state = cav.vehicle.get_light_state()
                    if light_state != carla.VehicleLightState.NONE:
                        blink_timer += 1
                    # turn of headlight after 25 steps
                    if blink_timer >= 25:
                        cav.vehicle.set_light_state(carla.VehicleLightState.NONE)
                        # reset
                        blink_timer = 0

            for i, single_cav in enumerate(single_cav_list):
                # this function should be added in wrapper
                if single_cav.v2x_manager.in_platoon():
                    # notify the change of status
                    single_cav.vehicle.set_light_state(carla.VehicleLightState(
                                                       carla.VehicleLightState.RightBlinker | 
                                                       carla.VehicleLightState.LeftBlinker |
                                                       carla.VehicleLightState.LowBeam | 
                                                       carla.VehicleLightState.HighBeam | 
                                                       carla.VehicleLightState.Fog | 
                                                       carla.VehicleLightState.Brake))

                    single_cav_list.pop(i)
                else:
                    single_cav.update_info()
                    control = single_cav.run_step()
                    single_cav.vehicle.apply_control(control)

    finally:
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()

        for platoon in platoon_list:
            platoon.destroy()
        for cav in single_cav_list:
            cav.destroy()
        for v in bg_veh_list:
            v.destroy()
