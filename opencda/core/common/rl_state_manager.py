# -*- coding: utf-8 -*-

""" Manager for calculating navigation status of RL ego vehicle
function to add: set total distance
"""
# Author: Xu Han
# License: TDG-Attribution-NonCommercial-NoDistrib

from collections import deque
import weakref

import carla
import numpy as np

from opencda.core.application.platooning.platooning_plugin \
    import PlatooningPlugin
from opencda.core.common.misc import compute_distance


class RLStateManager(object):
    def __init__(
            self,
            vehicle_manager_list,
            rl_config):

        # init interface
        self._rl_config = rl_config
        self.vehicle_manager_list = vehicle_manager_list
        self.ego_agent = vehicle_manager_list[0].agent
        self.ego_vehicle = vehicle_manager_list[0].vehicle
        self.ego_localizer = vehicle_manager_list[0].localizer
        self.ego_off_road = False
        self._is_v2v = self._rl_config.env.activate_v2v
        self._waypoint_num = self._rl_config.env.simulator.waypoint_num

        # init params
        self._ego_end_distance = float('inf')
        self._ego_end_timeout = float('inf')
        self._ego_total_distance = float('inf')
        self._ego_wrong_direction = False

    # ----- get parameters -----
    def get_end_distance(self):
        """
        Get the distance to target (along current route) for host vehicle in the current frame.
        Returns
        -------
        : float
            The distance to target value along the current planned route.
        """
        return self._ego_end_distance

    def get_ego_end_timeout(self):
        """
        Get the current timeout value from planner/agent.
        Returns
        -------
        :float
            The current timeout value.
        """
        return self._ego_end_timeout

    def get_total_distance(self):
        """
        Get the total distance to navigation goal along current route.
        Returns
        -------
        :float
            The calculated total distance to goal along current route.
        """
        return self._ego_total_distance

    def is_vehicle_wrong_direction(self):
        """
        Get the wrong direction indicator result.
        Returns
        -------
        :bool
            Whether the current host vehicle is driving in the wrong direction.
        """
        return self._ego_wrong_direction

    def get_route(self):
        """
        Get the current route of the behavioral agent.
        Returns
        -------
        :list
            The current route of the vehicle as a list of waypoints.
        """
        return self.ego_agent.get_route()

    def get_ego_speed(self):
        """
        Get the true speed of the ego vehicle.
        Returns
        -------
        :float
            Get the true speed of the ego vehicle, in km/h.
        """
        return self.ego_localizer.get_ego_spd()

    def get_ego_transform(self):
        """
        Get the transform of the ego vehicle.
        Returns
        -------
        :carla.transform
            The transform of the ego vehicle.
        """
        return self.ego_localizer.get_ego_pos()

    def get_ego_location(self):
        """
        Return the location (x,y,z) of the ego vehicle.
        Returns
        -------
        :carla.location
            The location of the ego vehicle.
        """
        return self.ego_localizer.get_ego_pos().location

    def get_ego_forward_vector(self):
        """
        Return the forward vector (x,y,z) of the ego vehicle.
        Returns
        -------
        :carla.vector
            The foward vector of the ego vehicle.s
        """
        return self.ego_localizer.get_ego_pos().rotation.get_forward_vector()

    def get_ego_acceleration(self):
        """
        Get the true acceleration of the ego vehicle.
        Returns
        -------
        :carla.vector
            The acceleration of the ego vehicle in x,y,z direction.
            Note: IMU sensor returns a rectified (0~1) acceleration value (accelerometer).
        """
        return self.ego_vehicle.get_acceleration()
        # return get_acc(self.ego_vehicle, meters=True)

    def get_ego_angular_velocity(self):
        """
        Get the angular velocity of the ego vehicle.
        Returns
        -------
        :carla.vector
            The angular velocity of the ego vehicle in deg/s.
        """
        return self.ego_vehicle.get_angular_velocity()

    def get_speed_vector(self):
        """
        Get the x,y,z velocity vector of the ego vehicle.
        Returns
        -------
        :carla.vector
            The current ego vehicle speed vector in m/s on x,y,z direction.
        """
        return self.ego_vehicle.get_velocity()

    # ----- set parameters -----
    def set_total_distance(self):
        """
        Update the total distance to navigation goal.
        """
        self._ego_total_distance = self.ego_agent.distance_to_goal

    def set_off_road(self, off_road):
        """
        Update off-road indicator based on carla env condition.
        Parameters
        ----------
        off_road:bool
            The indicator of off-road status.
        """
        self.ego_off_road = off_road

    # ----- rl functions -----
    def get_vehicle_state(self):
        """
        Return the current vehicle state information.

        Returns
        -------
        vehicle_state: tuple
            The detailed vehicle state in a tuple.
            Keys include: speed, location, forward vector,
                          acceleration, velocity, angular velocity, and rotation.
        """
        # vehicle state info
        speed = self.get_ego_speed()  # speed in km/h
        transform = self.get_ego_transform()  # ego transform
        location = self.get_ego_location()  # ego location
        forward_vector = self.get_ego_forward_vector()  # ego forward vector
        acceleration = self.get_ego_acceleration()  # ego acceleration vector
        angular_velocity = self.get_ego_angular_velocity()  # ego angular velocity
        velocity = self.get_speed_vector()  # ego speed in x,y,z direction

        # formalize vehicle state
        vehicle_state = {
            'speed': speed,
            'location': np.array([location.x, location.y, location.z]),
            'forward_vector': np.array([forward_vector.x, forward_vector.y]),
            'acceleration': np.array([acceleration.x, acceleration.y, acceleration.z]),
            'velocity': np.array([velocity.x, velocity.y, velocity.z]),
            'angular_velocity': np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z]),
            'rotation': np.array([transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll]),
        }
        return vehicle_state

    def get_navigation_state(self):
        """
        Get navigation info in current world, including agent state, command, navigation node,
        navigation target, and speed limit and speed list.

        Returns
        -------
        navigation:dict
            The current navigation information summarized in a dictionary.

        """
        command = self.ego_agent.node_road_option
        node_location = self.ego_agent.node_waypoint.transform.location
        node_forward = self.ego_agent.node_waypoint.transform.rotation.get_forward_vector()
        target_location = self.ego_agent.target_waypoint.transform.location
        target_forward = self.ego_agent.target_waypoint.transform.rotation.get_forward_vector()
        waypoint_list = self.ego_agent.get_waypoints_list(self._waypoint_num)
        direction_list = self.ego_agent.get_direction_list(self._waypoint_num)
        agent_state = self.ego_agent.agent_state
        speed_limit = self.ego_agent.speed_limit
        self._ego_end_distance = self.ego_agent.distance_to_goal
        self._ego_end_timeout = self.ego_agent.timeout

        waypoint_location_list = []
        for wp in waypoint_list:
            wp_loc = wp.transform.location
            wp_vec = wp.transform.rotation.get_forward_vector()
            waypoint_location_list.append([wp_loc.x, wp_loc.y, wp_vec.x, wp_vec.y])

        if not self.ego_off_road:
            current_waypoint = self.ego_agent.current_waypoint
            node_waypoint = self.ego_agent.node_waypoint

            # Lanes and roads are too chaotic at junctions
            if current_waypoint.is_junction or node_waypoint.is_junction:
                self._ego_wrong_direction = False
            else:
                node_yaw = node_waypoint.transform.rotation.yaw % 360
                cur_yaw = current_waypoint.transform.rotation.yaw % 360

                wp_angle = (node_yaw - cur_yaw) % 360

                if 150 <= wp_angle <= (360 - 150):
                    self._ego_wrong_direction = True
                else:
                    # Changing to a lane with the same direction
                    self._ego_wrong_direction = False

        navigation = {
            'agent_state': agent_state.value,
            'command': command.value,
            'node': np.array([node_location.x, node_location.y]),
            'node_forward': np.array([node_forward.x, node_forward.y]),
            'target': np.array([target_location.x, target_location.y]),
            'target_forward': np.array([target_forward.x, target_forward.y]),
            'waypoint_list': np.array(waypoint_location_list),
            'speed_limit': np.array(speed_limit),
            'direction_list': np.array(direction_list)
        }
        return navigation, waypoint_list

    def get_lidar_frames(self):
        lidar_frames = {}
        for iter, v_manager in enumerate(self.vehicle_manager_list):
            point_cloud = v_manager.data_dumper.lidar.data
            point_xyz = point_cloud[:, :-1]
            # point_intensity = point_cloud[:, -1]   todo: intensity may not be necessary
            if iter == 0:
                lidar_frames['ego_lidar_frame'] = point_xyz
            else:
                lidar_frames['lidar_frame_%d' % iter] = point_xyz
        return lidar_frames
