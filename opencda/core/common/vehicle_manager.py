# -*- coding: utf-8 -*-
"""
Basic class of CAV
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import uuid

from opencda.core.actuation.control_manager \
    import ControlManager
from opencda.core.application.platooning.platoon_behavior_agent \
    import PlatooningBehaviorAgent
from opencda.core.common.v2x_manager \
    import V2XManager
from opencda.core.sensing.localization.localization_manager \
    import LocalizationManager
from opencda.core.sensing.perception.perception_manager \
    import PerceptionManager
from opencda.core.plan.behavior_agent \
    import BehaviorAgent
from opencda.core.map.map_manager import MapManager
from opencda.core.common.data_dumper import DataDumper


class VehicleManager(object):
    """
    A class manager to embed different modules with vehicle together.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    config_yaml : dict
        The configuration dictionary of this CAV.

    application : list
        The application category, currently support:['single','platoon'].

    carla_map : carla.Map
        The CARLA simulation map.

    cav_world : opencda object
        CAV World. This is used for V2X communication simulation.

    current_time : str
        Timestamp of the simulation beginning, used for data dumping.

    data_dumping : bool
        Indicates whether to dump sensor data during simulation.

    Attributes
    ----------
    v2x_manager : opencda object
        The current V2X manager.

    localizer : opencda object
        The current localization manager.

    perception_manager : opencda object
        The current V2X perception manager.

    agent : opencda object
        The current carla agent that handles the basic behavior
         planning of ego vehicle.

    controller : opencda object
        The current control manager.

    data_dumper : opencda object
        Used for dumping sensor data.
    """

    def __init__(
            self,
            vehicle,
            config_yaml,
            application,
            carla_map,
            cav_world,
            current_time='',
            data_dumping=False):

        # an unique uuid for this vehicle
        self.vid = str(uuid.uuid1())
        self.vehicle = vehicle
        self.carla_map = carla_map

        # retrieve the configure for different modules
        sensing_config = config_yaml['sensing']
        map_config = config_yaml['map_manager']
        behavior_config = config_yaml['behavior']
        control_config = config_yaml['controller']
        v2x_config = config_yaml['v2x']

        # v2x module
        self.v2x_manager = V2XManager(cav_world, v2x_config, self.vid)
        # localization module
        self.localizer = LocalizationManager(
            vehicle, sensing_config['localization'], carla_map)
        # perception module
        self.perception_manager = PerceptionManager(
            vehicle, sensing_config['perception'], cav_world,
            data_dumping)
        # map manager
        self.map_manager = MapManager(vehicle,
                                      carla_map,
                                      map_config)

        # behavior agent
        self.agent = None
        if 'platooning' in application:
            platoon_config = config_yaml['platoon']
            self.agent = PlatooningBehaviorAgent(
                vehicle,
                self,
                self.v2x_manager,
                behavior_config,
                platoon_config,
                carla_map)
        else:
            self.agent = BehaviorAgent(vehicle, carla_map, behavior_config)

        # Control module
        self.controller = ControlManager(control_config)

        if data_dumping:
            self.data_dumper = DataDumper(self.perception_manager,
                                          vehicle.id,
                                          save_time=current_time)
        else:
            self.data_dumper = None

        # RL related attributes
        self._end_distance = float('inf')
        self._end_timeout = float('inf')
        self._total_distance = float('inf')
        self._wrong_direction = False

        cav_world.update_vehicle_manager(self)

    def set_destination(
            self,
            start_location,
            end_location,
            clean=False,
            end_reset=True):
        """
        Set global route.

        Parameters
        ----------
        start_location : carla.location
            The CAV start location.

        end_location : carla.location
            The CAV destination.

        clean : bool
             Indicator of whether clean waypoint queue.

        end_reset : bool
            Indicator of whether reset the end location.

        Returns
        -------
        """

        self.agent.set_destination(
            start_location, end_location, clean, end_reset)
        # first time set destination, calculate total distance to goal here.
        self._total_distance = self._agent.distance_to_goal

    def update_info(self):
        """
        Call perception and localization module to
        retrieve surrounding info an ego position.
        """
        # localization
        self.localizer.localize()

        ego_pos = self.localizer.get_ego_pos()
        ego_spd = self.localizer.get_ego_spd()

        # object detection
        objects = self.perception_manager.detect(ego_pos)

        # update the ego pose for map manager
        self.map_manager.update_information(ego_pos)

        # update ego position and speed to v2x manager,
        # and then v2x manager will search the nearby cavs
        self.v2x_manager.update_info(ego_pos, ego_spd)

        self.agent.update_information(ego_pos, ego_spd, objects)
        # pass position and speed info to controller
        self.controller.update_info(ego_pos, ego_spd)

    def run_step(self, target_speed=None):
        """
        Execute one step of navigation.
        """
        # visualize the bev map if needed
        self.map_manager.run_step()
        target_speed, target_pos = self.agent.run_step(target_speed)
        control = self.controller.run_step(target_speed, target_pos)

        # dump data
        if self.data_dumper:
            self.data_dumper.run_step(self.perception_manager,
                                      self.localizer,
                                      self.agent)

        return control

    def destroy(self):
        """
        Destroy the actor vehicle
        """
        self.perception_manager.destroy()
        self.localizer.destroy()
        self.vehicle.destroy()
        self.map_manager.destroy()

    def get_navigation_state(self):
        """
        Get navigation info in current world, including agent state, command, navigation node,
        navigation target, and speed limit and speed list.

        Returns
        -------
        navigation:dict
            The current navigation information summarized in a dictionary.

        """
        command = self.agent.node_road_option
        node_location = self.agent.node_waypoint.transform.location
        node_forward = self.agent.node_waypoint.transform.rotation.get_forward_vector()
        target_location = self.agent.target_waypoint.transform.location
        target_forward = self.agent.target_waypoint.transform.rotation.get_forward_vector()
        waypoint_list = self.agent.get_waypoints_list(self._waypoint_num)
        direction_list = self.agent.get_direction_list(self._waypoint_num)
        agent_state = self.agent.agent_state
        speed_limit = self.agent.speed_limit
        self._end_distance = self.agent.distance_to_goal
        self._end_timeout = self.agent.timeout

        if self._bev_wrapper is not None:
            self._bev_wrapper.update_waypoints(waypoint_list)

        waypoint_location_list = []
        for wp in waypoint_list:
            wp_loc = wp.transform.location
            wp_vec = wp.transform.rotation.get_forward_vector()
            waypoint_location_list.append([wp_loc.x, wp_loc.y, wp_vec.x, wp_vec.y])

        if not self._off_road:
            current_waypoint = self.agent.current_waypoint
            node_waypoint = self.agent.node_waypoint

            # Lanes and roads are too chaotic at junctions
            if current_waypoint.is_junction or node_waypoint.is_junction:
                self._wrong_direction = False
            else:
                node_yaw = node_waypoint.transform.rotation.yaw % 360
                cur_yaw = current_waypoint.transform.rotation.yaw % 360

                wp_angle = (node_yaw - cur_yaw) % 360

                if 150 <= wp_angle <= (360 - 150):
                    self._wrong_direction = True
                else:
                    # Changing to a lane with the same direction
                    self._wrong_direction = False

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
        return navigation

    def is_vehicle_wrong_direction(self):
        """
        Get the wrong direction indicator result.
        Returns
        -------
        :bool
            Whether the current host vehicle is driving in the wrong direction.

        """
        return self._wrong_direction

    def get_end_distance(self):
        """
        Get the distance to target (along current route) for host vehicle in the current frame.
        Returns
        -------
        : float
            The distance to target value along the current planned route.
        """
        return self._end_distance

    def get_total_distance(self):
        """
        Get the total distance to navigation goal along current route.
        Returns
        -------
        :float
            The calculated total distance to goal along current route.
        """
        return self._total_distance

    def get_total_timeout(self):
        """
        Get the current timeout value.
        Returns
        -------
        :flaot
            The current timeout value.
        """
        return self.get_total_timeout()
