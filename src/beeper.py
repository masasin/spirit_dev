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

        self.connected = False
        self.last_updated = None

        self._last_pose = None
        self._tracking = None
        self._start_time = rospy.Time.now()

        rospy.loginfo("Waiting for a connection")

    def beep(self):
        """
        Beeps an audio file.

        """
        pygame.mixer.music.play()

    def callback(self, pose):
        if pose is not None:
            if not self.connected:
                rospy.loginfo("Connection active")
                self.connected = True

            # TODO (masasin): `self._last_pose.pose != pose.pose` always True
            if self._last_pose and self._last_pose.pose == pose.pose:
                if self.last_updated is None:
                    reference_time = self._start_time
                else:
                    reference_time = self.last_updated

                if ((rospy.Time.now() - reference_time).to_sec() > TIMEOUT and
                        self.tracking is not False):  # Has a None init state
                    self.tracking = False
            else:
                self.last_updated = pose.header.stamp
                if not self.tracking and self._last_pose is not None:
                    self.tracking = True

            self._last_pose = pose

        else:
            rospy.logwarn("Received empty data")

    @property
    def tracking(self):
        return self._tracking

    @tracking.setter
    def tracking(self, value):
        if not value:
            self._handle_no_tracking()
        elif self.connected:
            self._handle_tracking()

        self._tracking = value

    def _handle_no_tracking(self):
        if self.tracking is None:
            rospy.logwarn("AR.Drone is not being tracked! "
                          "Please check your setup.")
        else:
            rospy.logwarn("Tracking lost!")
            self.beep()

    def _handle_tracking(self):
        if self.last_updated is None:
            rospy.loginfo("Tracking acquired")
        else:
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
