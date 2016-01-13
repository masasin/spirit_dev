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
    pose.header.stamp = rospy.Time.now()

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
    pose.header.stamp = rospy.Time.now()
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
