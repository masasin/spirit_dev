# -*- coding: utf-8 -*-
"""
Helper functions for vector, pose, and tf shenanigans.

"""
from __future__ import division

import numpy as np

import rospy
from geometry_msgs.msg import Point, Quaternion, PoseStamped, TransformStamped
import tf


d2r = np.deg2rad
r2d = np.rad2deg


def unit_vector(v):
    """
    Change the length of the vector to unity in the same direction.

    Parameters
    ----------
    v : np.ndarray
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


def get_pose_components(pose):
    """
    Return the coordinates and orientation of a pose as a numpy array.

    Parameters
    ----------
    pose : Pose(WithCovariance)?(Stamped)?
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

    pose.pose.position = Point(*coords)
    pose.pose.orientation = Quaternion(*orientation)
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

    pose.pose.position = transform.transform.translation
    pose.pose.orientation = transform.transform.rotation

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

    transform.transform.translation = pose.pose.position
    transform.transform.rotation = pose.pose.orientation

    return transform


def quat2axis(quaternion):
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
        The angle in axis-angle representation, with the order of θ, x, y, z. θ
        is in degrees.

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


def quat2euler(quaternion):
    """
    Change a quaternion to an Euler angle representation.

    Parameters
    ----------
    quaternion : np.ndarray
        A quaternion in the order of x, y, z, w.

    Returns
    -------
    np.ndarray
        The Euler angle, in the order of roll, pitch, yaw.

    """
    return tf.transformations.euler_from_quaternion(quaternion)


def angle_between(a, b):
    """
    Find the angle between two vectors.

    Parameters
    ----------
    a : Sequence[float]
        A vector of length n.
    b : Sequence[float]
        A vector of length n.

    Returns
    -------
    float
        The angle between the quaternions, in radians.

    Raises
    ------
    ValueError
        If the lengths of the vectors are different.

    Examples
    --------
    >>> angle_between([1, 0, 0], [0, 1, 0])
    1.5707963267948966
    >>> angle_between([1, 0, 0], [1, 0, 0])
    0.0
    >>> angle_between([1, 0, 0], [-1, 0, 0])
    3.1415926535897931

    """
    return np.arccos(np.clip(np.dot(unit_vector(a), unit_vector(b)), -1, 1))


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
    >>> angle_between_quaternions([0, 0, 0, 1], [0, 0, 0, 1])
    0.0
    >>> angle_between_quaternions([0, 0, 0, 1], [0, -1, 0, 1])
    1.5707963267948968

    """
    return 2 * angle_between(a, b)


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
