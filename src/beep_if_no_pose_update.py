#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Beep when pose does not change for `TIMEOUT_TIME`

"""
import pygame
import rospy
from geometry_msgs.msg import PoseStamped


TIMEOUT = 0.25  # seconds


class Beeper(object):
    def __init__(self):
        pygame.init()
        pygame.mixer.music.load("audio/beep.wav")
        self.beep()
        rospy.loginfo("Trial beep made.")

        self.subscriber = rospy.Subscriber("/ardrone/pose", PoseStamped,
                                           self.callback, queue_size=1)

        self._last_pose = None
        self._tracking_lost = False

    def beep(self):
        """
        Beeps an audio file.

        """
        pygame.mixer.music.play()

    def callback(self, pose):
        if self._tracking_lost and self._last_pose.pose != pose.pose:
            self._tracking_lost = False
            rospy.loginfo("Tracking reacquired.")

        if (not self._tracking_lost and self._last_pose is not None and
            (self._last_pose.header.stamp - rospy.Time.now())
                .to_secs() > TIMEOUT):
            self._tracking_lost = True
            rospy.loginfo("Tracking lost.")
            self.beep()

        self._last_pose = pose


def shutdown_hook():
    """Runs on shutdown."""
    pygame.quit()


def main():
    rospy.init_node("pose_determiner", anonymous=True)
    rospy.on_shutdown(shutdown_hook)
    Beeper()
    rospy.spin()


if __name__ == "__main__":
    main()