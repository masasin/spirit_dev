from __future__ import division
import time
try:
    from unittest import mock
except ImportError:
    import mock

import pygame
import pytest

import rospy
from geometry_msgs.msg import PoseStamped

import tracking_verifier


try:
    rospy.init_node("pytest", anonymous=True)
except rospy.exceptions.ROSException:
    pass


@pytest.fixture(scope="module")
def pygame_teardown():
    def fin():
        pygame.quit()


class TestVerifier(object):
    def setup(self):
        self.timeout = 5e-4
        tracking_verifier.TIMEOUT = self.timeout
        with mock.patch("rospy.Time.now",
                        return_value=rospy.rostime.Time(time.time())):
            self.tracking_verifier = tracking_verifier.Verifier()

    def create_pose(self):
        with mock.patch("rospy.Time.now",
                        return_value=rospy.rostime.Time(time.time())):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            return pose

    def make_steady_state(self):
        pose = self.create_pose()
        pose.pose.position.x = 1
        self.tracking_verifier.callback(pose)
        pose = self.create_pose()
        self.tracking_verifier.callback(pose)

    @mock.patch.object(tracking_verifier.Verifier, "beep", autospec=True)
    def test_changing_pose_steady_state(self, mock_beep):
        self.make_steady_state()

        for i in range(21):
            pose = self.create_pose()
            pose.pose.position.x = i
            self.tracking_verifier.callback(pose)
            time.sleep(self.timeout / 10)

        assert not mock_beep.called
        assert self.tracking_verifier.tracking
        assert self.tracking_verifier.last_updated == pose.header.stamp

    @mock.patch.object(tracking_verifier.Verifier, "beep", autospec=True)
    def test_not_changing_pose_steady_state(self, mock_beep):
        self.make_steady_state()
        last_change_time = self.tracking_verifier.last_updated

        for i in range(21):
            self.tracking_verifier.callback(self.create_pose())
            time.sleep(self.timeout / 10)

        assert mock_beep.called
        assert not self.tracking_verifier.tracking
        assert self.tracking_verifier.last_updated == last_change_time

    @mock.patch.object(tracking_verifier.Verifier, "beep", autospec=True)
    def test_tracking_reacquired_steady_state(self, mock_beep):
        self.make_steady_state()
        self.tracking_verifier._tracking = False

        time.sleep(self.timeout / 10)
        pose = self.create_pose()
        pose.pose.position.x = 1
        self.tracking_verifier.callback(pose)

        assert not mock_beep.called
        assert self.tracking_verifier.tracking

    @mock.patch.object(tracking_verifier.Verifier, "beep", autospec=True)
    def test_changing_pose_at_start(self, mock_beep):
        pose = self.create_pose()
        self.tracking_verifier.callback(pose)

        assert not mock_beep.called
        assert self.tracking_verifier.tracking is None
        assert self.tracking_verifier.last_updated is None

        time.sleep(self.timeout / 10)
        pose = self.create_pose()
        pose.pose.position.x = 1
        self.tracking_verifier.callback(pose)

        assert not mock_beep.called
        assert self.tracking_verifier.tracking
        assert self.tracking_verifier.last_updated == pose.header.stamp

    @mock.patch.object(tracking_verifier.Verifier, "beep", autospec=True)
    def test_not_changing_pose_at_start(self, mock_beep):
        pose = self.create_pose()
        self.tracking_verifier.callback(pose)

        for i in range(21):
            self.tracking_verifier.callback(self.create_pose())
            time.sleep(self.timeout / 10)

        assert not mock_beep.called
        assert self.tracking_verifier.tracking is False
        assert self.tracking_verifier.last_updated is None

    @mock.patch.object(tracking_verifier.Verifier, "beep", autospec=True)
    def test_tracking_reacquired_after_bad_start(self, mock_beep):
        pose = self.create_pose()
        self.tracking_verifier.callback(pose)

        for i in range(21):
            self.tracking_verifier.callback(self.create_pose())
            time.sleep(self.timeout / 10)

        time.sleep(self.timeout / 10)
        pose = self.create_pose()
        pose.pose.position.x = 1
        self.tracking_verifier.callback(pose)

        assert not mock_beep.called
        assert self.tracking_verifier.tracking

    @mock.patch.object(tracking_verifier.Verifier, "beep", autospec=True)
    def test_empty_pose(self, mock_beep):
        self.tracking_verifier.callback(None)

        assert not mock_beep.called
        assert self.tracking_verifier.tracking is None
        assert self.tracking_verifier.last_updated is None
