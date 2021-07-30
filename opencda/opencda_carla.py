# -*- coding: utf-8 -*-
"""
Used to reduce the dependency on CARLA api by mocking
them in the same structure.
"""
# Author: Credits to Pylot team <https://github.com/erdos-project/pylot>
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


class Vector3D(object):
    """Represents a 3D vector and provides useful helper functions.
    Args:
        x: The value of the first axis.
        y: The value of the second axis.
        z: The value of the third axis.
    Attributes:
        x: The value of the first axis.
        y: The value of the second axis.
        z: The value of the third axis.
    """

    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    @classmethod
    def from_simulator_vector(cls, vector):
        """Creates a pylot Vector3D from a simulator 3D vector.

        Args:
            vector: An instance of a simulator 3D vector.

        Returns:
            :py:class:`.Vector3D`: A pylot 3D vector.
        """
        from carla import Vector3D
        if not isinstance(vector, Vector3D):
            raise ValueError('The vector must be a Vector3D')
        return cls(vector.x, vector.y, vector.z)


class Location(Vector3D):
    """Stores a 3D location, and provides useful helper methods.

    Args:
        x: The value of the x-axis.
        y: The value of the y-axis.
        z: The value of the z-axis.

    Attributes:
        x: The value of the x-axis.
        y: The value of the y-axis.
        z: The value of the z-axis.
    """

    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        super(Location, self).__init__(x, y, z)

    @classmethod
    def from_simulator_location(cls, location):
        """Creates a pylot Location from a simulator location.

        Args:
            location: An instance of a simulator location.

        Returns:
            :py:class:`.Location`: A pylot location.
        """
        from carla import Location, Vector3D
        if not (isinstance(location, Location)
                or isinstance(location, Vector3D)):
            raise ValueError('The location must be a Location or Vector3D')
        return cls(location.x, location.y, location.z)


class Rotation(object):
    """Used to represent the rotation of an actor or obstacle.

    Rotations are applied in the order: Roll (X), Pitch (Y), Yaw (Z).
    A 90-degree "Roll" maps the positive Z-axis to the positive Y-axis.
    A 90-degree "Pitch" maps the positive X-axis to the positive Z-axis.
    A 90-degree "Yaw" maps the positive X-axis to the positive Y-axis.

    Args:
        -pitch: Rotation about Y-axis.
        -yaw:   Rotation about Z-axis.
        -roll:  Rotation about X-axis.

    Attributes:
        -pitch: Rotation about Y-axis.
        -yaw:   Rotation about Z-axis.
        -roll:  Rotation about X-axis.
    """

    def __init__(self, pitch: float = 0, yaw: float = 0, roll: float = 0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll

    @classmethod
    def from_simulator_rotation(cls, rotation):
        """Creates a pylot Rotation from a simulator rotation.

        Args:
            -rotation: An instance of a simulator rotation.

        Returns:
            :py:class:`.Rotation`: A pylot rotation.
        """
        from carla import Rotation
        if not isinstance(rotation, Rotation):
            raise ValueError('rotation should be of type Rotation')
        return cls(rotation.pitch, rotation.yaw, rotation.roll)


class Transform(object):
    """A class that stores the location and rotation of an obstacle.

    It can be created from a simulator transform, defines helper functions
    needed in Pylot, and makes the simulator transform serializable.

    A transform object is instantiated with either a location and a rotation,
    or using a matrix.

    Args:
        -location (:py:class:`.Location`, optional): The location of the object
            represented by the transform.
        -rotation (:py:class:`.Rotation`, optional): The rotation  (in degrees)
            of the object represented by the transform.

    Attributes:
        -location (:py:class:`.Location`): The location of the object
            represented by the transform.
        -rotation (:py:class:`.Rotation`): The rotation (in degrees) of the
            object represented by the transform.
    """

    def __init__(self,
                 location: Location = None,
                 rotation: Rotation = None):
        self.location = location
        self.rotation = Rotation(0, 0, 0) if not rotation else rotation

    @classmethod
    def from_simulator_transform(cls, transform):
        """Creates a pylot transform from a simulator transform.

        Args:
            -transform: A simulator transform.

        Returns:
            :py:class:`.Transform`: An instance of a pylot transform.
        """
        from carla import Transform
        if not isinstance(transform, Transform):
            raise ValueError('transform should be of type Transform')
        return cls(Location.from_simulator_location(transform.location),
                   Rotation.from_simulator_rotation(transform.rotation))
