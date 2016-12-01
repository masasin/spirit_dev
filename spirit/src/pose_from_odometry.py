#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
Generate a pose using AR.Drone odometry readings.

Note that this would only work if the AR.Drone is active with the motors on.

"""
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class PoseObtainer(object):
    """
    Generates a pose using AR.Drone odometry readings.

    """
    def __init__(self):
        self.odom_subscriber = rospy.Subscriber("/ardrone/odometry",
                                                Odometry,
                                                self.odometry_callback,
                                                queue_size=1)
        self.pose_publisher = rospy.Publisher("/ardrone/pose",
                                              PoseStamped, queue_size=1)
        rospy.logdebug("Subscribed to /ardrone/odometry")

    def odometry_callback(self, odom):
        """
        Publish pose.

        Parameters
        ----------
        odom : Odometry
            Odometry message.

        """
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_publisher.publish(pose)


def main():
    """
    Main entry point for script.

    """
    rospy.init_node("pose_from_odometry", anonymous=True)
    PoseObtainer()
    rospy.loginfo("Obtaining pose")
    rospy.spin()


if __name__ == "__main__":
    main()
