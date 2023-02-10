"""
Sensors related to safety status check
"""
import math
import numpy as np
import carla
import weakref
from collections import deque


class CollisionSensor(object):
    """
    Collision detection sensor.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle, this is for cav.
    params : dict
        The dictionary containing sensor configurations.

    Attributes
    ----------
    image : np.ndarray
        Current received rgb image.
    sensor : carla.sensor
        The carla sensor that mounts at the vehicle.
    """

    def __init__(self, vehicle, params):
        world = vehicle.get_world()

        blueprint = world.get_blueprint_library().find('ensor.other.collision')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(),
                                        attach_to=vehicle)

        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: CollisionSensor._on_collision(weak_self, event))

        self.collided = False
        self.collided_frame = -1
        self._history = deque(maxlen=params['history_size'])
        self._threshold = params['col_thresh']

    @staticmethod
    def _on_collision(weak_self, event) -> None:
        self = weak_self()
        if not self:
            return
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self._history.append((event.frame, intensity))
        if intensity > self._threshold:
            self.collided = True
            self.collided_frame = event.frame

    def return_status(self):
        return {'collision': self.collided}

    def destroy(self) -> None:
        """
        Clear collision sensor in Carla world.
        """
        self._history.clear()
        if self.sensor.is_alive:
            self.sensor.stop()
            self.sensor.destroy()


class StuckDetector(object):
    """
    Stuck detector used to detect vehicle stuck in simulator.
    It takes speed as input in each tick.

    Parameters
    ----------
    params : dict
        The dictionary containing sensor configurations.
    """

    def __init__(self, params):
        self._speed_queue = deque(maxlen=params['len_thresh'])
        self._len_thresh = params['len_thresh']
        self._speed_thresh = params['speed_thresh']

        self.stuck = False

    def tick(self, data_dict) -> None:
        """
        Update one tick

        Parameters
        ----------
        data_dict : dict
            The data dictionary provided by the upsteam modules.
        """
        speed = data_dict['ego_speed']
        self._speed_queue.append(speed)
        if len(self._speed_queue) >= self._len_thresh:
            if np.average(self._speed_queue) < self._speed_thresh:
                self.stuck = True
                return
        self.stuck = False

    def return_status(self):
        return {'stuck': self.stuck}

    def destroy(self):
        """
        Clear speed history
        """
        self._speed_queue.clear()


class OffRoadDetector(object):
    """
    A detector to monitor whether

    Parameters
    ----------
    params : dict
        The dictionary containing sensor configurations.
    """
    def __init__(self, params):
        self.off_road = False

    def tick(self, data_dict) -> None:
        """
        Update one tick

        Parameters
        ----------
        data_dict : dict
            The data dictionary provided by the upsteam modules.
        """
        # static bev map that indicate where is the road
        static_map = data_dict['static_bev_map']
        h, w = static_map.shape[0], static_map.shape[1]
        # the ego is always at the center of the bev map. If the pixel is
        # black, that means the vehicle is off road.
        if np.mean(static_map[h//2, w//2]) == 255:
            self.off_road = True
        else:
            self.off_road = False

    def destroy(self):
        pass


class TrafficLightHelper(object):
    """
    Interface of traffic light detector and recorder. It detects next traffic light state,
    calculates distance from hero vehicle to the end of this road, and if hero vehicle crosses
    this line when correlated light is red, it will record running a red light
    """
    def __init__(self, params):
        self.light_dist_thresh = params['light_dist_thresh']

        self.total_lights_ran = 0
        self.total_lights = 0
        self.ran_light = False

    def tick(self, data_dict) -> None:
        # todo: implement later
        pass
