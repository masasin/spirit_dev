#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Manages the octree.

Adds all frame and position data to the octree, and selects the best image for
SPIRIT based on an evaluation function. The data for that image is published.

"""
from time import localtime, strftime

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

from octree import Octree


class State(object):
    def __init__(self, pose, image):
        self.pose = pose
        self._position_precise = np.array([self.pose.position.x,
                                           self.pose.position.y,
                                           self.pose.position.z])
        self.position = self._position_precise // 10 * 10  # Round to 1 cm
        self.image = image
        self.stamp = strftime("%Y-%m-%d %H:%M:%S",
                              localtime(self.pose.header.stamp.to_time()))

    def __repr__(self):
        return "Frame ({}): {}".format(self._position_precise.tolist(),
                                       self.stamp)


class Manager(object):
    """
    Data container.

    """
    def __init__(self):
        self.clear()
        self.octree = Octree((0, 0, 0), 10000)  # 10 m
        rospy.Subscriber("/output/slow_image_raw", Image, self.image_callback)
        rospy.Subscriber("/ardrone/pose", PoseStamped, self.pose_callback)

    def image_callback(self, image):
        self.image = image
        if self.is_ready:
            rospy.logdebug("Adding data")
            self.octree.insert(State(self.pose, self.image))
            self.clear()

    def pose_callback(self, pose):
        self.pose = pose
        best = self.select_best_past_image()

    def select_best_past_image(self):
        """
        Use the current pose to select the best image.

        """

    @property
    def is_ready(self):
        return self.image and self.pose

    def clear(self):
        self.image = None
        self.pose = None


def main():
    """Initialize ROS node."""
    rospy.init_node("manager")
    Manager()
    rospy.loginfo("Started octree manager")
    rospy.spin()


if __name__ == "__main__":
    main()
