#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Beep when pose does not change for `TIMEOUT_TIME`

"""
import os

from geometry_msgs.msg import PoseStamped
import pygame
import rospy


TIMEOUT = 0.25  # seconds
SPIRIT_ROOT = os.path.expanduser("~/catkin_ws/src/spirit")


class Beeper(object):
    def __init__(self):
        pygame.init()
        pygame.mixer.music.load(os.path.join(SPIRIT_ROOT, "audio/beep.wav"))

        self.subscriber = rospy.Subscriber("/ardrone/pose", PoseStamped,
                                           self.callback, queue_size=1)

        self._last_pose = None
        self._last_pose_change_time = None
        self._tracking_lost = False
        self._started = False
        self._start_time = rospy.Time.now()

        rospy.loginfo("Waiting for a connection")

    def beep(self):
        """
        Beeps an audio file.

        """
        pygame.mixer.music.play()

    def callback(self, pose):
        if not self._started:
            rospy.loginfo("Connection active")
            self._started = True

        if self._last_pose is not None and pose is not None:
            # TODO (masasin): `self._last_pose.pose != pose.pose` always True
            if self._last_pose.pose == pose.pose:
                if self._last_pose_change_time is None:
                    reference_time = self._start_time
                else:
                    reference_time = self._last_pose_change_time

                if ((rospy.Time.now() - reference_time).to_sec() > TIMEOUT and
                        not self._tracking_lost):
                    self.handle_tracking_lost()
            else:
                self._last_pose_change_time = pose.header.stamp
                if self._tracking_lost:
                    self.handle_tracking_found()

        self._last_pose = pose

    def handle_tracking_lost(self):
        self._tracking_lost = True
        if self._last_pose_change_time is None:
            rospy.logwarn("AR.Drone is not being tracked! "
                          "Please check your setup.")
        else:
            rospy.logwarn("Tracking lost!")
            self.beep()

    def handle_tracking_found(self):
        self._tracking_lost = False
        rospy.loginfo("Tracking reacquired")


def shutdown_hook():
    """Runs on shutdown."""
    pygame.quit()


def main():
    rospy.init_node("beeper", anonymous=True)
    rospy.on_shutdown(shutdown_hook)
    Beeper()
    rospy.spin()


if __name__ == "__main__":
    main()
