#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Publishes random similar poses to /ardrone/pose.

"""
from __future__ import division

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

from helpers import tf_from_pose


class PoseGenerator(object):
    """
    A pose generator.

    """
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber("/ardrone/pose",
                                                PoseStamped,
                                                self.stream,
                                                queue_size=1)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.rate = rospy.Rate(200)

    def stream(self, pose):
        """
        Send random poses.

        """
        self.tf_pub.sendTransform(tf_from_pose(pose, child="ardrone/body"))
        self.rate.sleep()


def main():
    """
    Main entry point for script.

    """
    rospy.init_node("mocap_pose_tf", anonymous=True)
    rospy.loginfo("Streaming tf")
    PoseGenerator()
    rospy.loginfo("Obtaining tf")
    rospy.spin()


if __name__ == "__main__":
    main()
