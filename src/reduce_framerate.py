#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Reduce /ardrone_camera framerate to 2 Hz.

"""
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageFeature(object):
    """
    A ROS image Publisher/Subscriber.

    """
    def __init__(self):
        self.subscriber = rospy.Subscriber("/ardrone/image_raw",
                                           Image, self.callback, queue_size=1)
        self.image_pub = rospy.Publisher("/output/slow_image_raw",
                                         Image, queue_size=1)
        self.bridge = CvBridge()
        rospy.logdebug("Subscribed to /ardrone_camera/image_raw.")
        self.count = 0

    def callback(self, ros_data):
        """
        Callback function of subscribed topic.

        """
        # Publish every fifteenth frame
        if not self.count % 15:
            try:
                image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr(e)

        self.count += 1


def main():
    """Initialize and cleanup ROS node."""
    rospy.init_node("image_feature", anonymous=True)
    ImageFeature()
    rospy.loginfo("Starting feature detection.")
    rospy.spin()


if __name__ == "__main__":
    main()
