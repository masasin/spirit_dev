# -*- coding: utf-8 -*-
"""
Helper functions for vector, pose, and tf shenanigans.

"""
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped


def normalize_vector(v):
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
    x, y, z, w = normalize_vector(quaternion)
    angle = np.rad2deg(2 * np.arccos(w))

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
    n = sum(i**2 for i in quaternion)
    s = 0 if n == 0 else 2 / n

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
