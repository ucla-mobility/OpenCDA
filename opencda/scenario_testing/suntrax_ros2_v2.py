# -*- coding: utf-8 -*-
"""
Scenario testing: single vehicle behavior in intersection
"""
# Author: XH
# License: TDG-Attribution-NonCommercial-NoDistrib
import os
import carla
import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus, \
                           CarlaWorldInfo, CarlaStatus, CarlaEgoVehicleInfo, \
                           CarlaActorInfo

import opencda.scenario_testing.utils.sim_api as sim_api
import opencda.scenario_testing.utils.customized_map_api as map_api
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import \
    load_yaml
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.utils.suntrax_carla_ros2_nodes import \
    VehicleInfoSubscriber, CarlaDataSubscriber, ROS2ControlPublisher,\
    ROSInfoManager
from opencda.core.sensing.localization.coordinate_transform import \
    geo_to_transform


def run_scenario(opt, config_yaml):
    try:
        # client = carla.Client('localhost', 2000)
        # client.get_world().tick()
        
        scenario_params = load_yaml(config_yaml)
        cav_world = CavWorld(opt.apply_ml)

        # create scenario manager to read current world
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   cav_world=cav_world)

        if opt.record:
            scenario_manager.client. \
                start_recorder("suntrax_ros2.log", True)

        # single CAV 
        single_cav_list = \
            scenario_manager.create_ros_vehicle_manager(application=['single'])

        # platoon CAV
        # todo: implement platoon manager here for the mainline vehicle

        # ROS2 subs
        carla_ros2_world = scenario_manager.world
        geo_ref = carla_ros2_world.get_map().transform_to_geolocation(carla.Location(x=0, y=0, z=0))

        # CAV role names
        single_cav_config = scenario_params['scenario']['single_cav_list'][0]
        single_cav_role_name = single_cav_config['vehicle_name']
        print('single cav role name is: ' + str(single_cav_role_name))

        # init subscriber nodes
        rclpy.init()
        carla_data_subscriber = CarlaDataSubscriber()
        single_cav_subscriber = VehicleInfoSubscriber(single_cav_role_name)
        command_pubisher = ROS2ControlPublisher(single_cav_role_name)

        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = single_cav_list[0].vehicle

        # init subscriber nodes
        rclpy.init()
        executor = MultiThreadedExecutor()
        executor.add_node(carla_data_subscriber)
        executor.add_node(single_cav_subscriber)
        executor.add_node(command_pubisher)
        executor.spin()

        # run steps
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

            single_cav = single_cav_list[0]
            single_cav.update_info()
            control = single_cav.run_step()

            e_brake = carla.VehicleControl()
            e_brake.throttle = 0.0
            e_brake.brake = 1.0
            e_brake.steer = 0.0
            single_cav.vehicle.apply_control(e_brake)


    finally:
        # eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        executor.shutdown()
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

        # scenario_manager.close()

        # for v in single_cav_list:
        #     v.destroy()

