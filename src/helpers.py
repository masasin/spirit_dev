import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped


def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    else:
        return v / norm


def get_pose_components(pose):
    coords = np.array([pose.pose.position.x,
                       pose.pose.position.y,
                       pose.pose.position.z])

    orientation = np.array([pose.pose.orientation.x,
                            pose.pose.orientation.y,
                            pose.pose.orientation.z,
                            pose.pose.orientation.w])

    return coords, orientation


def pose_from_components(coords, orientation, sequence=0):
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


def tf_from_pose(pose, parent="world", child="robot"):
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
