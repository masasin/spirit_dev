#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Publish the tracking status of the drone.

Analyze the pose stream to ensure that the pose is being updated. If the pose
does not change within `TIMEOUT`, then it is considered not to be tracked. If
tracking was lost, beep to let the operator know.

"""
import os

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import pygame
import rospy


TIMEOUT = 0.25  # seconds
SPIRIT_ROOT = os.path.expanduser("~/catkin_ws/src/spirit")


class Verifier(object):
    """
    Verify tracking.

    Attributes
    ----------
    connected : bool
        Whether a connection has been established.
    last_updated : rospy.rostime.Time
        The time at which the last change in pose occured.
    tracking : bool
        Whether the drone is being tracked. Initially, its value is None.

    """
    def __init__(self):
        pygame.init()
        pygame.mixer.music.load(os.path.join(SPIRIT_ROOT, "audio/beep.wav"))

        self.subscriber = rospy.Subscriber("/ardrone/pose", PoseStamped,
                                           self.callback, queue_size=1)
        self.publisher = rospy.Publisher("/ardrone/tracked", Bool,
                                         latch=True, queue_size=1)

        self.connected = False
        self.last_updated = None

        self._last_pose = None
        self._tracking = None
        self._start_time = rospy.Time.now()

        rospy.loginfo("Waiting for a connection")

    def beep(self):
        """
        Plays an audio file containing a beep.

        """
        pygame.mixer.music.play()

    def callback(self, pose):
        """
        Receive pose updates and act accordingly.

        Parameters
        ----------
        pose : StampedPose
            A StampedPose containing the latest pose.

        """
        if pose is None:
            rospy.logwarn("Received empty data")
            return

        if not self.connected:
            rospy.loginfo("Connection active")
            self.connected = True

        if self._last_pose is not None:  # Not the first update.
            if self._last_pose.pose == pose.pose:  # Pose unchanged.
                reference_time = self.last_updated or self._start_time
                if ((rospy.Time.now() - reference_time).to_sec() > TIMEOUT and
                        self.tracking is not False):  # Has a None init state
                    self.tracking = False

            else:  # Pose changed.
                self.last_updated = pose.header.stamp
                if not self.tracking:
                    self.tracking = True

        self._last_pose = pose

    @property
    def tracking(self):
        """
        Check whether the drone is being tracked.

        Returns
        -------
        bool
            True if the drone is being tracked.

        """
        return self._tracking

    @tracking.setter
    def tracking(self, value):
        """
        Change the value of the tracking, and publish the new state.

        Depending on the value and other class attributes, additional actions
        may be taken.

        Parameters
        ----------
        value : bool
            The new value of the tracking state.

        """
        if not value:
            if self.tracking is None:
                rospy.logwarn("AR.Drone is not being tracked! "
                              "Please check your setup.")
            else:
                rospy.logwarn("Tracking lost!")
                self.beep()
        elif self.connected:
            if self.last_updated is None:
                rospy.loginfo("Tracking acquired")
            else:
                rospy.loginfo("Tracking reacquired")

        self._tracking = value
        self.publisher.publish(Bool(value))


def shutdown_hook():
    """Runs on shutdown."""
    pygame.quit()


def main():
    """Main entry point for script."""
    rospy.init_node("tracking_verifier", anonymous=True)
    rospy.on_shutdown(shutdown_hook)
    Verifier()
    rospy.spin()


if __name__ == "__main__":
    main()
