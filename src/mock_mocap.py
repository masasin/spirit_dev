#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Publish random similar poses to /ardrone/pose.

"""
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped


class PoseGenerator(object):
    """
    A pose generator.

    """
    def __init__(self):
        self.pose_pub = rospy.Publisher("/ardrone/pose",
                                        PoseStamped, queue_size=1)
        self.rate = rospy.Rate(30)

    def stream(self):
        """
        Send random poses.

        """
        sequence = 0

        while not rospy.is_shutdown():
            sequence += 1
            pose = PoseStamped()
            pose.header.seq = sequence
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = np.random.rand()
            pose.pose.position.y = np.random.rand()
            pose.pose.position.z = np.random.rand()

            pose.pose.orientation.x = np.random.rand()
            pose.pose.orientation.y = np.random.rand()
            pose.pose.orientation.z = np.random.rand()
            pose.pose.orientation.w = np.random.rand()

            self.pose_pub.publish(pose)
            self.rate.sleep()

        self.shutdown()

    def shutdown(self):
        self.camera.release()


def main():
    rospy.init_node("mock_pose", anonymous=True)
    rospy.loginfo("Streaming mock poses.")
    PoseGenerator().stream()


if __name__ == "__main__":
    main()
