#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Publish the stream from /dev/video0 to /ardrone_camera/image_raw at 30 Hz.

"""
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Webcam(object):
    """
    A webcam publisher.

    """
    def __init__(self):
        self.cam_pub = rospy.Publisher("/ardrone_camera/image_raw",
                                       Image, queue_size=1)
        self.camera = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)

    def stream(self):
        """
        Send camera stream.

        """
        while not rospy.is_shutdown():
            ret, frame = self.camera.read()

            try:
                self.cam_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr(e)

            self.rate.sleep()
        self.shutdown()

    def shutdown(self):
        self.camera.release()


def shutdown_hook():
    cv2.destroyAllWindows()


def main():
    rospy.init_node("webcam", anonymous=True)
    rospy.on_shutdown(shutdown_hook)
    rospy.loginfo("Streaming camera.")
    Webcam().stream()


if __name__ == "__main__":
    main()
