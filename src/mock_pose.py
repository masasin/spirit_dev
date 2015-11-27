#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Publish random similar poses to /ardrone/pose.

"""
from __future__ import division

import numpy as np

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

from helpers import pose_from_components, tf_from_pose


class PoseGenerator(object):
    """
    A pose generator.

    """
    def __init__(self):
        self.pose_pub = rospy.Publisher("/ardrone/pose",
                                        PoseStamped, queue_size=1)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(200)

    def stream(self):
        """
        Send random poses.

        """
        sequence = 0

        while not rospy.is_shutdown():
            sequence += 1
            pose = self.generate_random_pose(sequence)

            self.pose_pub.publish(pose)
            self.tf_pub.sendTransform(tf_from_pose(pose, child="ardrone/body"))
            self.rate.sleep()

    @staticmethod
    def generate_random_pose(sequence=0):
        return pose_from_components(coords=np.random.rand(3) / 100,
                                    orientation=(0.1, 0, 0.1, 0.1),
                                    sequence=sequence)


def main():
    rospy.init_node("mock_pose", anonymous=True)
    rospy.loginfo("Streaming mock poses")
    PoseGenerator().stream()


if __name__ == "__main__":
    main()
