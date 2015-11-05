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


class StateData(object):
    """
    Data container.

    """
    def __init__(self):
        self.clear()

    def image_callback(self, image):
        self.image = image

    def pose_callback(self, pose):
        self.pose = pose

    @property
    def is_ready(self):
        return self.image and self.pose

    def clear(self):
        self.image = None
        self.pose = None


class Node(object):
    def __init__(self, state):
        self.pose = state.pose
        self._position_precise = np.array([self.pose.position.x,
                                           self.pose.position.y,
                                           self.pose.position.z])
        self.position = self._position_precise // 10 * 10  # Round to 1 cm
        self.image = state.image
        self.stamp = strftime("%Y-%m-%d %H:%M:%S",
                              localtime(self.pose.header.stamp.to_time()))

    def __repr__(self):
        return "Frame ({}): {}".format(self._position_precise.tolist(),
                                       self.stamp)


def registrar():
    state = StateData()
    octree = Octree((0, 0, 0), 10000)

    rospy.init_node("registrar")
    rate = rospy.rate(200)  # Hz

    rospy.Subscriber("/output/slow_image_raw", Image, state.image_callback)
    rospy.Subscriber("/ardrone/pose", PoseStamped, state.pose_callback)

    while not rospy.is_shutdown():
        if state.is_ready:
            rospy.logdebug("Adding data")
            node = Node(state)
            octree.insert(node)
            state.clear()

        rate.sleep()


if __name__ == "__main__":
    try:
        registrar()
    except rospy.ROSInterruptException:
        pass
