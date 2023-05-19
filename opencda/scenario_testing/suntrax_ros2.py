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

        # init ROS2
        rclpy.init()
        carla_data_subscriber = CarlaDataSubscriber()
        single_cav_subscriber = VehicleInfoSubscriber(single_cav_role_name)
        main_cav1_subscriber = VehicleInfoSubscriber('mainline_ADS_vehicle_1')
        main_cav2_subscriber = VehicleInfoSubscriber('mainline_ADS_vehicle_2')

        single_cav_command_pubisher = ROS2ControlPublisher(single_cav_role_name)
        main_cav1_command_pubisher = ROS2ControlPublisher('mainline_ADS_vehicle_1')
        main_cav2_command_pubisher = ROS2ControlPublisher('mainline_ADS_vehicle_2')

        print("ROS init done, Press Enter to continue...")
        input()
        print("Continuing...")

        if opt.record:
            scenario_manager.client. \
                start_recorder("suntrax_ros2.log", True)

        # create platoon members
        platoon_list = \
            scenario_manager.create_ros_platoon_manager(map_helper=None,
                                                        data_dump=False)


        # single CAV 
        single_cav_list = \
            scenario_manager.create_ros_vehicle_manager(['platooning'])

        # platoon CAV
        # todo: implement platoon manager here for the mainline vehicle

        # ROS2 subs
        carla_ros2_world = scenario_manager.world
        geo_ref = carla_ros2_world.get_map().transform_to_geolocation(carla.Location(x=0, y=0, z=0))

        # CAV role names
        single_cav_config = scenario_params['scenario']['single_cav_list'][0]
        single_cav_role_name = single_cav_config['vehicle_name']
        print('single cav role name is: ' + str(single_cav_role_name))

        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = single_cav_list[0].vehicle


        try:
            # merge simulation loop and ROS2 loop 
            while rclpy.ok():
                # tick from opencda side
                # scenario_manager.tick()

                # spectator
                transform = spectator_vehicle.get_transform()
                spectator.set_transform(
                    carla.Transform(
                        transform.location +
                        carla.Location(
                            z=80),
                        carla.Rotation(
                            pitch=-
                            90)))

                # spin ROS2 nodes
                rclpy.spin_once(carla_data_subscriber, timeout_sec=0.005)
                rclpy.spin_once(single_cav_subscriber, timeout_sec=0.005)
                rclpy.spin_once(single_cav_command_pubisher, timeout_sec=0.005)
                
                # get GNSS 
                single_cav_gnss = single_cav_subscriber.get_gnss_data()
                single_cav_IMU = single_cav_subscriber.get_imu_data()
                single_cav_speed = single_cav_subscriber.get_speed_data()
                
                # transfer GNSS to x,y,z location
                if single_cav_gnss is not None: # and\
                    # single_cav_speed is not None and\
                    # single_cav_IMU is not None: 
                    x, y, z = geo_to_transform(single_cav_gnss.latitude,
                                               single_cav_gnss.longitude,
                                               single_cav_gnss.altitude,
                                               geo_ref.latitude,
                                               geo_ref.longitude, 0.0)
                    single_cav_location = carla.Location(x,y,z)


                    # debug stream
                    # print('======== Debug stream ========')
                    # print("map: " + str(carla_data_subscriber.map))
                    # print("world_info: " + str(carla_data_subscriber.world_info))
                    # print("carla_status: " + str(carla_data_subscriber.carla_status))
                    # print("----- Vehicle data -----")
                    # print("single CAV GNSS: " + str(single_cav_gnss))
                    # print('------------------------')
                    # print("single CAV IMU: " + str(single_cav_IMU))
                    # print('------------------------')
                    # print("single CAV Speed: " + str(single_cav_speed.data))
                    # print('------------------------')
                    # print("single CAV coordinates: " + str(single_cav_location))
                    # print("==============================")
                    # print("")

                    # assign ROS2 info manager 
                    # single_cav_ros_manager = ROSInfoManager(single_cav_role_name,
                    #                                         single_cav_location,
                    #                                         single_cav_speed)

                # platoon control
                # control_list = []
                # platoon = platoon_list[0]
                for platoon in platoon_list:
                    platoon.update_information()
                    control_list = platoon.run_ros_step()
                    # print(control_list)
                    # publish control
                    '''
                    Note:
                     !!! need to update this. after joning, the platoon list order will change!!
                     Try use only the role name to publish control !!!
                    '''
                    if len(control_list) <= 2:
                        main_cav1_command_pubisher.publish_control_command(control_list[0])
                        main_cav2_command_pubisher.publish_control_command(control_list[1])
                    elif len(control_list) > 2:
                        # adjust leader speed
                        single_cav_command_pubisher.publish_control_command(control_list[0], speed_reduce=True)
                        main_cav1_command_pubisher.publish_control_command(control_list[1])
                        main_cav2_command_pubisher.publish_control_command(control_list[2])


                # single CAV
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
                        # single_cav.update_ros_info(single_cav_location, 
                        #                             single_cav_speed, 
                        #                             use_ros_data=True)
                        control = single_cav.run_step()
                        single_cav_command_pubisher.publish_control_command(control)


                        
                # # single vehicle testing
                # single_cav = single_cav_list[0]       
                # single_cav.update_ros_info(single_cav_location, 
                #                            single_cav_speed.data, 
                #                            use_ros_data=False)
                # control = single_cav.run_step()
                # e_brake = carla.VehicleControl()
                # e_brake.throttle = 0.0
                # e_brake.brake = 1.0
                # e_brake.steer = 0.0
                # # single_cav.vehicle.apply_control(control)
                # single_cav_command_pubisher.publish_control_command(control)


        except KeyboardInterrupt:
            pass 

        carla_data_subscriber.destroy_node()
        single_cav_subscriber.destroy_node()
        rclpy.shutdown()


    finally:
        # eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        # scenario_manager.close()

        # for v in single_cav_list:
        #     v.destroy()

