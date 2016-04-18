# -*- coding: utf-8 -*-
"""
Helper functions for vector, pose, and tf shenanigans.

"""
from __future__ import division
from collections import namedtuple

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf


d2r = np.deg2rad
r2d = np.rad2deg
Euler = namedtuple("Euler", ("roll", "pitch", "yaw"))


def normalize(v):
    """
    Change the length of the vector to unity in the same direction.

    Parameters
    ----------
    v : np.ndarray
        A vector to be normalized.

    Returns
    -------
    np.ndarray
        The normalized vector.

    """
    norm = np.linalg.norm(v)
    return (v / norm) if norm else v


def get_pose_components(pose):
    """
    Return the coordinates and orientation of a pose as a numpy array.

    Parameters
    ----------
    pose : Pose | PoseWithCovariance | PoseStamped | PoseWithCovarianceStamped
        The pose to be decomposed.

    Returns
    -------
    coords : np.ndarray
        The x, y, and z coordinates contained in the pose.
    orientation : np.ndarray
        The x, y, z, and w quaternion contained in the pose.

    """
    coords = np.array([pose.pose.position.x,
                       pose.pose.position.y,
                       pose.pose.position.z])

    orientation = np.array([pose.pose.orientation.x,
                            pose.pose.orientation.y,
                            pose.pose.orientation.z,
                            pose.pose.orientation.w])

    return coords, orientation


def pose_from_components(coords, orientation, sequence=0):
    """
    Generate a pose from its components.

    Parameters
    ----------
    coords : Sequence[float]
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
    pose = PoseStamped()
    pose.header.seq = sequence
    try:
        pose.header.stamp = rospy.Time.now()
    except rospy.exceptions.ROSInitException:
        pass

    pose.pose.position.x = coords[0]
    pose.pose.position.y = coords[1]
    pose.pose.position.z = coords[2]

    pose.pose.orientation.x = orientation[0]
    pose.pose.orientation.y = orientation[1]
    pose.pose.orientation.z = orientation[2]
    pose.pose.orientation.w = orientation[3]

    return pose


def pose_from_tf(transform):
    """
    Generate a pose from a transform.

    Parameters
    ----------
    transform : TransformStamped
        The transform to be translated.

    Returns
    -------
    PoseStamped
        The pose.

    """
    pose = PoseStamped()
    try:
        pose.header.stamp = rospy.Time.now()
    except rospy.exceptions.ROSInitException:
        pass
    pose.header.frame_id = transform.header.frame_id

    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z

    pose.pose.orientation.x = transform.transform.rotation.x
    pose.pose.orientation.y = transform.transform.rotation.y
    pose.pose.orientation.z = transform.transform.rotation.z
    pose.pose.orientation.w = transform.transform.rotation.w

    return pose


def tf_from_pose(pose, parent="world", child="robot"):
    """
    Generate a transform from a pose.

    Parameters
    ----------
    pose : Pose(WithCovariance)?(Stamped)?
        The pose to be translated.
    parent : Optional[str]
        The frame_id of the transform. Default is "world"
    child : Optional[str]
        The child_frame_id of the transform. Default is "robot"

    Returns
    -------
    TransformStamped
        The transform.

    """
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent
    transform.child_frame_id = child

    transform.transform.translation.x = pose.pose.position.x
    transform.transform.translation.y = pose.pose.position.y
    transform.transform.translation.z = pose.pose.position.z

    transform.transform.rotation.x = pose.pose.orientation.x
    transform.transform.rotation.y = pose.pose.orientation.y
    transform.transform.rotation.z = pose.pose.orientation.z
    transform.transform.rotation.w = pose.pose.orientation.w

    return transform


def quat2axis(quaternion):
    """
    Change a quaternion to an axis-angle representation.

    Parameters
    ----------
    quaternion : np.ndarray
        A quaternion in the order of x, y, z, w.

    Returns
    -------
    tuple
        The angle in axis-angle representation, with the order of θ, x, y, z

    """
    x, y, z, w = normalize(quaternion)
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


def quat2euler(quaternion):
    """
    Change a quaternion to an axis-angle representation.

    Parameters
    ----------
    quaternion : np.ndarray
        A quaternion in the order of x, y, z, w.

    Returns
    -------
    Euler
        The euler angle, as roll, pitch, and yaw.

    """
    return Euler(tf.transformations.euler_from_quaternion(quaternion))


def angle_between_quaternions(a, b):
    """
    Find the angle between two quaternions.

    Parameters
    ----------
    a : Sequence[float]
        A quaternion in the order of x, y, z, w.
    b : Sequence[float]
        A quaternion in the order of x, y, z, w.

    Returns
    -------
    float
        The angle between the quaternions, in radians.

    Notes
    -----
    The angle is multiplied by 2 because of its effect in 3D.

    Examples
    --------
    >>> np.rad2deg(angle_between_quaternions([0, 0, 0, 1], [0, 0, 0, 1]))
    0.0
    >>> np.rad2deg(angle_between_quaternions([0, 0, 0, 1], [0, -1, 0, 1]))
    90.0

    """
    return 2 * np.arccos(np.dot(normalize(a), normalize(b)))


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


def fov_diagonal2vertical(fov_diagonal, aspect_ratio=4 / 3):
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
    return 2 * r2d(np.arctan(np.tan(d2r(fov_diagonal) / 2) / ratio_diagonal))


def fov_vertical2horizontal(fov_vertical, aspect_ratio=4 / 3):
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
