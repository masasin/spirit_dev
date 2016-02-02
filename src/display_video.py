#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Display raw feed and low framerate feed.

"""
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image


class Display(object):
    """
    Displays images in an OpenCV window.

    Remap image to the target node from the launch file.

    """
    def __init__(self):
        self.subscriber = rospy.Subscriber("image", Image, self.callback,
                                           queue_size=1)
        self.bridge = CvBridge()
        self.contents = rospy.get_param("~window_name", "camera")

    def callback(self, image):
        """
        Display the image.

        Parameters
        ----------
        image : Image
            The image to display.

        """
        # Convert to CV2.
        try:
            cv2_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            cv2.imshow(self.contents.capitalize(), cv2_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)


def shutdown_hook():
    """
    Run on shutdown.

    """
    cv2.destroyAllWindows()


def main():
    """
    Main entry point for script.

    """
    rospy.init_node("image_feature", anonymous=True)
    rospy.on_shutdown(shutdown_hook)
    display = Display()
    rospy.loginfo("Displaying {}.".format(display.contents))
    rospy.spin()


if __name__ == "__main__":
    main()
