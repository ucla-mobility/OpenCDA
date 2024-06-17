# -*- coding: utf-8 -*-
"""
ROS2 node to subscribe ADS vehicle data.
"""
# Author: Xu Han
# License: MIT

import uuid
import carla
import rclpy
import math
from collections import deque

import numpy as np
from rclpy.node import Node

from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus, \
                           CarlaWorldInfo, CarlaStatus, CarlaEgoVehicleInfo, \
                           CarlaActorInfo, CarlaEgoVehicleControl


# vehicle sensor subscriber
class VehicleInfoSubscriber(Node):
    def __init__(self, vehicle_role_name: str):
        super(VehicleInfoSubscriber, self).__init__('vehicle_info_subscriber_' + vehicle_role_name)
        
        self.gnss_data = NavSatFix()
        self.imu_data = Imu()
        self.speed_data = None
        self.current_speed = None

        self.gnss_subscription = self.create_subscription(
            NavSatFix,
            f'/carla/{vehicle_role_name}/gnss',
            self.gnss_callback,
            200
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            f'/carla/{vehicle_role_name}/imu',
            self.imu_callback,
            200
        )
        self.speedo_subscription = self.create_subscription(
            Float32,
            f'/carla/{vehicle_role_name}/speedometer',
            self.speedo_callback,
            200
        )
        self.odometry__subscription = self.create_subscription(
            Odometry,
            f'/carla/{vehicle_role_name}/odometry',
            self.odometry_cb,
            200
        )

    def gnss_callback(self, msg: NavSatFix):
        self.gnss_data = msg

    def imu_callback(self, msg: Imu):
        self.imu_data = msg

    def speedo_callback(self, msg: Float32):
        self.speed_data = msg.data

    def odometry_cb(self, odometry_msg):
        self._current_pose = odometry_msg.pose.pose
        self.current_speed = math.sqrt(odometry_msg.twist.twist.linear.x ** 2 +
                                            odometry_msg.twist.twist.linear.y ** 2 +
                                            odometry_msg.twist.twist.linear.z ** 2) * 3.6

    def get_gnss_data(self):
        return self.gnss_data

    def get_imu_data(self):
        return self.imu_data

    def get_speed_data(self):
        return self.current_speed

# carla world subscriber
class CarlaDataSubscriber(Node):
    def __init__(self):
        super().__init__('carla_data_subscriber')

        # vehicle role name to subscribe to corresponding topic
        self.ADS_vehicle_data = {}

        # simulation info
        self.map = None
        self.world_info = None
        self.carla_status = None

        # tf for all sensors
        self.tf_meta = TFMessage()

        # ROS subscription world
        self.map_subscription = self.create_subscription(
            String,
            '/carla/map',
            self.carla_map_callback,
            200
        )

        self.carla_status_subscription = self.create_subscription(
            CarlaStatus,
            '/carla/status',
            self.carla_status_callback,
            200
        )

        self.world_info_subscription = self.create_subscription(
            CarlaWorldInfo,
            '/carla/world_info',
            self.world_info_callback,
            200
        )

        self.TF_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            200
        )

    def carla_map_callback(self, msg: String):
        self.map = msg
        # self.get_logger().info(f'The subscribed map info is: {msg}')

    def carla_status_callback(self, msg: CarlaStatus):
        self.carla_status = msg
        # self.get_logger().info(f'The subscribed CARLA status is: {msg}')

    def world_info_callback(self, msg: CarlaWorldInfo):
        self.world_info = msg
        # self.get_logger().info(f'The subscribed World Info is: {msg}')

    def tf_callback(self, msg: TFMessage):
        self.tf_meta = msg

# control command publisher
class ROS2ControlPublisher(Node):
    def __init__(self, vehicle_name):
        super().__init__('ros2_control_publisher')
        vehicle_name = vehicle_name
        self.publisher_ = self.create_publisher(CarlaEgoVehicleControl, 
                                                '/carla/' + vehicle_name + '/vehicle_control_cmd', 
                                                200)
        # Create a new queue with a maximum size
        self.throttle = deque(maxlen=3)


    def publish_control_command(self, control, speed_reduce=False):
        msg = CarlaEgoVehicleControl()
        if control.throttle:
            msg.throttle = control.throttle
            self.throttle.append(control.throttle)
        else:
            msg.throttle = self.throttle[2]

        if control.steer:     
            msg.steer = np.clip(control.steer, -1.0, 1.0)  
        else: 
            msg.steer = 0.0

        if control.brake:
            msg.brake = np.clip(control.brake, 0.0, 1.0)  
        else:
            msg.brake = 0.0
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: \
        #                         throttle="{msg.throttle}", \
        #                         steer="{msg.steer}", \
        #                         brake="{msg.brake}"')

# ROS2 information manager
class ROSInfoManager(object):
    """
    A manager class to regulate all information from ROS topics.
    """

    def __init__(
            self,
            vehicle_role_name,
            ros_location,
            ros_speed):

        self.vehicle_role_name = vehicle_role_name
        self.ros_location = ros_location
        self.ros_speed = ros_speed


    def update_info(self,
                    vehicle_role_name,
                    ros_location,
                    ros_speed):
        self.vehicle_role_name = vehicle_role_name
        self.ros_location = ros_location
        self.ros_speed = ros_speed

    def get_ros_location(self):
        return  self.ros_location

    def get_speed(self):
        return self.ros_speed