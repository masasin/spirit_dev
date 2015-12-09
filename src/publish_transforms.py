#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Publishes random similar poses to /ardrone/pose.

"""
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

from helpers import tf_from_pose


class TransformGenerator(object):
    """
    A pose generator.

    """
    def __init__(self):
        self.pose_pub = rospy.Publisher("/ardrone/pose",
                                        PoseStamped, queue_size=1)
        rospy.Subscriber("/ardrone/pose", PoseStamped, self.pose_callback)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(200)

    def pose_callback(self, pose):
        self.tf_pub.sendTransform(tf_from_pose(pose, child="ardrone/body"))


def main():
    """
    Main entry point for script.

    """
    rospy.init_node("generate_tf", anonymous=True)
    rospy.loginfo("Generatign transforms")
    TransformGenerator()
    rospy.spin()


if __name__ == "__main__":
    main()
