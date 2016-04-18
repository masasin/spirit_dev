#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
Reduce /ardrone/image_color framerate from 30 Hz to 2 Hz.

"""
from __future__ import division

import rospy
from sensor_msgs.msg import Image


class FramerateReducer(object):
    """
    Reduces the framerate of a video feed to one fifteenth of the original.

    """
    def __init__(self):
        self.image_subscriber = rospy.Subscriber("/ardrone/image_color",
                                                 Image, self.frame_callback,
                                                 queue_size=1)
        self.image_publisher = rospy.Publisher("/ardrone/slow_image_raw",
                                               Image, queue_size=1)
        rospy.logdebug("Subscribed to /ardrone/image_color")
        self.period = rospy.get_param("slowdown", 15)  # times
        self.count = 0

    def frame_callback(self, frame):
        """
        Publish at a reduced rate.

        Parameters
        ----------
        frame : Image
            A newly arrived image.

        """
        # Publish at the reduced rate.
        if not self.count % self.period:
            self.image_publisher.publish(frame)
        self.count += 1


def main():
    """
    Main entry point for script.

    """
    rospy.init_node("framerate_reducer", anonymous=True)
    FramerateReducer()
    rospy.loginfo("Reducing framerate")
    rospy.spin()


if __name__ == "__main__":
    main()
