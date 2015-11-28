#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Publishes random similar poses to /ardrone/pose.

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
        self.n_items = 100000
        x = np.arange(self.n_items)
        self.x_pos = np.sin(0.1*x*np.pi/180)
        self.y_pos = np.sin(0.1*x*np.pi/180)
        self.z_pos = np.sin(0.1*x*np.pi/180)

    def stream(self):
        """
        Send random poses.

        """
        sequence = 0

        while not rospy.is_shutdown():
            sequence += 1
            pose = self.generate_sine_pose(sequence)

            self.pose_pub.publish(pose)
            self.tf_pub.sendTransform(tf_from_pose(pose, child="ardrone/body"))
            self.rate.sleep()

    def generate_sine_pose(self, sequence=0):
        """
        Generate a sinusoidal pose.

        Parameters
        ----------
        sequence : int, optional
            The sequence of the pose. Default is zero.

        Returns
        -------
        The generated pose.

        """
        idx = sequence % self.n_items
        return pose_from_components(coords=(30*self.x_pos[idx],
                                            0,
                                            0),  # self.z_pos[idx]),
                                    orientation=(0, 1, 0, -0.6),
                                    sequence=sequence)

    @staticmethod
    def generate_random_pose(sequence=0):
        """
        Generate a random pose.

        Parameters
        ----------
        sequence : int, optional
            The sequence of the pose. Default is zero.

        Returns
        -------
        The generated pose.

        """
        return pose_from_components(coords=np.random.rand(3) / 100,
                                    orientation=(0.1, 0, 0.1, 0.1),
                                    sequence=sequence)


def main():
    """
    Main entry point for script.

    """
    rospy.init_node("mock_pose", anonymous=True)
    rospy.loginfo("Streaming mock poses")
    PoseGenerator().stream()


if __name__ == "__main__":
    main()
