# -*- coding: utf-8 -*-
# (C) 2016 Jean Nassar
# Released under BSD
"""
Helper functions and classes for general use.

"""
from __future__ import division
from functools import partial, update_wrapper
from time import localtime, strftime

import numpy as np
from numpy.linalg import norm

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import tf


d2r = np.deg2rad
r2d = np.rad2deg
EULER_CONVENTION = "rxyz"


def unit_vector(v):
    """
    Change the length of the vector to unity in the same direction.

    Parameters
    ----------
    v : array-like
        A vector to be normalized.

    Returns
    -------
    np.ndarray
        The normalized vector, or the original vector if it has a length of 0.

    """
    norm = np.linalg.norm(v)
    if norm:
        return v / norm
    else:
        return np.asarray(v)


# noinspection PyPep8Naming
class memoize(object):
    def __init__(self, func):
        self.func = func
        update_wrapper(self, func)

    def __get__(self, instance, owner):
        if instance is None:
            return self.func
        return partial(self, instance)

    def __call__(self, *args, **kwargs):
        obj = args[0]
        try:
            cache = obj.__cache__
        except AttributeError:
            cache = obj.__cache__ = {}
        key = (self.func, args[1:], frozenset(kwargs.items()))
        try:
            res = cache[key]
        except KeyError:
            res = cache[key] = self.func(*args, **kwargs)
        return res


class Pose(object):
    """
    Convenience wrapper for PoseStamped.

    Parameters
    ----------
    pose_stamped : PoseStamped
        The pose message.

    Attributes
    ----------
    pose_stamped : PoseStamped
        The pose message.
    position : np.ndarray
        The x, y, and z coordinates contained in the pose.
    orientation : np.ndarray
        The x, y, z, and w quaternion contained in the pose.
    header : Header
        The header from the pose message

    """
    def __init__(self, pose_stamped):
        self.pose_stamped = pose_stamped
        self.position, self.orientation = self._components(self.pose_stamped)
        self.header = self.pose_stamped.header

    def rel_position(self, pose, rotation_matrix=None):
        """
        Calculate the relative position with another pose, with local reference.

        Parameters
        ----------
        pose : Pose
            The target pose.
        rotation_matrix : Optional[np.ndarray]
            The rotation matrix to use. If not provided, the rotation matrix of
            the current pose is used.

        Returns
        -------
        np.ndarray
            The x, y, z relative positions.

        """
        if rotation_matrix is None:
            rotation_matrix = Quat.rotation_matrix(self.orientation)
        return rotation_matrix.dot(pose.position - self.position)

    def rel_euler(self, pose):
        """
        Calculate the relative angle with another pose.

        Parameters
        ----------
        pose : Pose
            The target pose.

        Returns
        -------
        np.ndarray
            The relative angle as Euler, in the order of pitch, roll, yaw.

        """
        return Quat.to_euler(Quat.rel_rotation(pose.orientation,
                                               self.orientation))

    def distance(self, pose):
        """
        Calculate the distance to another pose.

        Parameters
        ----------
        pose : Pose
            The target pose.

        Returns
        -------
        float
            The distance to the target pose.

        """
        return norm(pose.position - self.position)

    @staticmethod
    def _components(pose_stamped):
        """
        Return the position and orientation of a PoseStamped as numpy arrays.

        Parameters
        ----------
        pose_stamped : Pose(WithCovariance)?(Stamped)?
            The pose to be decomposed.

        Returns
        -------
        position : np.ndarray
            The x, y, and z coordinates contained in the pose.
        orientation : np.ndarray
            The x, y, z, and w quaternion contained in the pose.

        """
        position = np.array([pose_stamped.pose.position.x,
                             pose_stamped.pose.position.y,
                             pose_stamped.pose.position.z])

        orientation = np.array([pose_stamped.pose.orientation.x,
                                pose_stamped.pose.orientation.y,
                                pose_stamped.pose.orientation.z,
                                pose_stamped.pose.orientation.w])

        return position, orientation

    @classmethod
    def from_components(cls, position, orientation, sequence=0):
        """
        Generate a Pose from its components.

        Parameters
        ----------
        position : Sequence[float]
            The x, y, and z coordinates of the pose.
        orientation : Sequence[float]
            The x, y, z, and w quaternion of the pose.
        sequence : Optional[int]
            The sequence number of the pose.

        Returns
        -------
        Pose
            The generated pose.

        """
        return cls(cls.generate_stamped(position, orientation, sequence))

    @staticmethod
    def generate_stamped(position, orientation, sequence=0):
        """
        Generate a PoseStamped from its components.

        Parameters
        ----------
        position : Sequence[float]
            The x, y, and z coordinates of the pose.
        orientation : Sequence[float]
            The x, y, z, and w quaternion of the pose.
        sequence : Optional[int]
            The sequence number of the pose.

        Returns
        -------
        PoseStamped
            The generated pose.

        """
        pose_stamped = PoseStamped()
        pose_stamped.header.seq = sequence
        try:
            pose_stamped.header.stamp = rospy.Time.now()
        except rospy.exceptions.ROSInitException:
            pass

        pose_stamped.pose.position = Point(*position)
        pose_stamped.pose.orientation = Quaternion(*orientation)
        return pose_stamped

    def __repr__(self):
        return "<Pose ({position}, {orientation})>".format(
            position=self.position.tolist(),
            orientation=self.orientation.tolist(),
            time=self.header.stamp)

    def __str__(self):
        return "<Pose ({position}, {orientation}): {time}>".format(
            position=self.position.tolist(),
            orientation=self.orientation.tolist(),
            time=self.header.stamp)


class Frame(object):
    """
    Encapsulate an image and the pose it was taken in.

    Parameters
    ----------
    pose_stamped : PoseStamped
        The pose of the drone when the image was taken.
    image : Image
        The image that was taken.

    Attributes
    ----------
    pose_stamped : PoseStamped
        The raw pose message of the drone at which the image was taken.
    pose : Pose
        The pose of the drone at which the image was taken.
    rotation_matrix : np.ndarray
        The rotation matrix of the frame orientation.
    image : Image
        The image that was taken.
    stamp : rospy.rostime.Time
        The timestamp of the pose.
    stamp_str : str
        The timestamp of the pose, in human readable format.

    """
    def __init__(self, pose_stamped, image):
        self.pose_stamped = pose_stamped
        self.pose = Pose(pose_stamped)
        self.rotation_matrix = Quat.rotation_matrix(self.pose.orientation)
        self.image = image
        self.stamp = self.pose.header.stamp
        self.stamp_str = strftime("%Y-%m-%d %H:%M:%S",
                                  localtime(self.stamp.to_time()))

    @memoize
    def rel_position(self, pose):
        """
        Calculate the relative position with another pose, with local reference.

        Parameters
        ----------
        pose : Pose
            The target pose.

        Returns
        -------
        np.ndarray
            The x, y, z relative positions.

        """
        return self.pose.rel_position(pose,
                                      rotation_matrix=self.rotation_matrix)

    @memoize
    def rel_euler(self, pose):
        """
        Calculate the relative angle with another pose.

        Parameters
        ----------
        pose : Pose
            The target pose.

        Returns
        -------
        np.ndarray
            The relative angle as Euler, in the order of pitch, roll, yaw.

        """
        return self.pose.rel_euler(pose)

    @memoize
    def distance(self, pose):
        """
        Calculate the distance to another pose.

        Parameters
        ----------
        pose : Pose
            The target pose.

        Returns
        -------
        float
            The distance to the target pose.

        """
        return self.pose.distance(pose)

    def __repr__(self):
        return "<Frame({pose})>".format(pose=self.pose)


class Fov(object):
    """
    Field of view methods.

    """
    @staticmethod
    def d2v(fov_diagonal, aspect_ratio=4 / 3):
        """
        Convert a diagonal field of view to vertical.

        Parameters
        ----------
        fov_diagonal : float
            The diagonal field of view.
        aspect_ratio: Optional[float]
            The aspect ratio of the display. Default is 4:3.

        Returns
        -------
        float
            The vertical field of view.

        """
        ratio_diagonal = np.sqrt(1 + aspect_ratio**2)
        return 2 * r2d(np.arctan(np.tan(d2r(fov_diagonal) / 2)
                                 / ratio_diagonal))

    @staticmethod
    def v2h(fov_vertical, aspect_ratio=4 / 3):
        """
        Convert a vertical field of view to horizontal.

        Parameters
        ----------
        fov_vertical : float
            The vertical field of view.
        aspect_ratio: Optional[float]
            The aspect ratio of the display. Default is 4:3.

        Returns
        -------
        float
            The horizontal field of view.

        """
        return 2 * r2d(np.arctan(np.tan(d2r(fov_vertical) / 2) * aspect_ratio))


class Quat(object):
    """
    Quaternion methods.

    """
    @staticmethod
    def to_euler(quaternion):
        """
        Change a quaternion to an Euler angle representation.

        Parameters
        ----------
        quaternion : np.ndarray
            A quaternion in the order of x, y, z, w.

        Returns
        -------
        np.ndarray
            The Euler angle, in the order of pitch, roll, yaw.

        """
        # noinspection PyUnresolvedReferences
        return tf.transformations.euler_from_quaternion(quaternion,
                                                        EULER_CONVENTION)

    @staticmethod
    def to_axis(quaternion):
        """
        Change a quaternion to an axis-angle representation.

        Parameters
        ----------
        quaternion : np.ndarray
            A quaternion in the order of x, y, z, w.

        Notes
        -----
        θ is in degrees rather than radians, for ease of integration in OpenGL.

        Returns
        -------
        tuple
            The angle in axis-angle representation, with the order of θ, x, y,
            z. θ is in degrees.

        """
        x, y, z, w = unit_vector(quaternion)
        angle = r2d(2 * np.arccos(w))

        if angle == 0:
            axis_x = 1
            axis_y = axis_z = 0
        elif angle % 180 == 0:
            axis_x, axis_y, axis_z = x, y, z
        else:
            axis_x = x / np.sqrt(1 - w**2)
            axis_y = y / np.sqrt(1 - w**2)
            axis_z = z / np.sqrt(1 - w**2)

        return angle, axis_x, axis_y, axis_z

    @staticmethod
    def product(a, b):
        """
        Find the product of two quaternions.

        Parameters
        ----------
        a : Sequence[float]
            A quaternion, in the order of x, y, z, w
        b : Sequence[float]
            A quaternion, in the order of x, y, z, w

        Returns
        -------
        np.ndarray
            A quaternion, in the order of x, y, z, w

        """
        imaginary_part = a[3] * b[:3] + b[3] * a[:3] + np.cross(a[:3], b[:3])
        real_part = a[3] * b[3] - np.dot(a[:3], b[:3])
        return np.append(imaginary_part, real_part)

    @staticmethod
    def inverse(quaternion):
        """
        Return the inverse of a quaternion

        Parameters
        ----------
        quaternion : Sequence[float]
            A quaternion, in the order of x, y, z, w

        Returns
        -------
        np.ndarray
            The inverse of the quaternion.

        """
        return (quaternion * np.array([-1, -1, -1, 1])
                / np.linalg.norm(quaternion)**2)

    @staticmethod
    def rel_rotation(a, b):
        """
        Find the quaternion which produces a rotation from `a` to `b`.

        Parameters
        ----------
        a : Sequence[float]
            A quaternion, in the order of x, y, z, w
        b : Sequence[float]
            A quaternion, in the order of x, y, z, w

        Returns
        -------
        np.ndarray
            A quaternion, in the order of x, y, z, w

        """
        return Quat.product(unit_vector(a), Quat.inverse(unit_vector(b)))

    @staticmethod
    def rotation_matrix(quaternion):
        """
        Create the rotation matrix of a quaternion.

        Parameters
        ----------
        quaternion : np.ndarray
            A quaternion in the order of x, y, z, w.

        Returns
        -------
        np.ndarray
            A 3x3 rotation matrix representing the quaternion.

        References
        ----------
        .. [1] Wikipedia, Rotation Matrix.
               https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion

        """
        x, y, z, w = quaternion
        n = sum(quaternion**2)

        if n == 0:
            return np.identity(3)

        s = 2 / n

        wx = s * w * x
        wy = s * w * y
        wz = s * w * z

        xx = s * x * x
        xy = s * x * y
        xz = s * x * z

        yy = s * y * y
        yz = s * y * z
        zz = s * z * z

        return np.array([[1 - (yy + zz), xy - wz, wy],
                         [wz, 1 - (xx + zz), yz - wx],
                         [xz - wy, yz + wx, 1 - (xx + yy)]])
