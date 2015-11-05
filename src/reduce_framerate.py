#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Reduce /ardrone/image_raw framerate from 30 Hz to 2 Hz.

"""
import rospy
from sensor_msgs.msg import Image


class ImageFeature(object):
    """
    A ROS image Publisher/Subscriber.

    """
    def __init__(self):
        self.image_subscriber = rospy.Subscriber("/ardrone/image_raw",
                                                 Image, self.image_callback,
                                                 queue_size=1)
        self.image_publisher = rospy.Publisher("/output/slow_image_raw",
                                               Image, queue_size=1)
        rospy.logdebug("Subscribed to /ardrone/image_raw")
        self.count = 0

    def frame_callback(self, frame):
        """
        Callback function of subscribed topic.

        """
        # Publish every fifteenth frame
        if not self.count % 15:
            self.image_publisher.publish(frame)
        self.count += 1


def main():
    """Initialize ROS node."""
    rospy.init_node("framerate_reducer", anonymous=True)
    ImageFeature()
    rospy.loginfo("Reducing framerate")
    rospy.spin()


if __name__ == "__main__":
    main()
