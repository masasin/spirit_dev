from __future__ import division
import time
try:
    from unittest import mock
except ImportError:
    import mock

from geometry_msgs.msg import PoseStamped
import rospy

import beeper


class TestBeeper(object):
    def setup(self):
        self.timeout = 1e-4
        beeper.TIMEOUT = self.timeout
        rospy.init_node("beeper_test", anonymous=True)
        self.beeper = beeper.Beeper()

    @mock.patch.object(beeper.Beeper, "beep", autospec=True)
    def test_changing_pose_steady_state(self, mock_beep):
        initial_pose = PoseStamped()
        initial_pose.header.stamp = rospy.Time.now()

        self.beeper.connected = True
        self.beeper._last_pose = initial_pose
        self.beeper.last_updated = rospy.Time.now()

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        for i in range(21):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = i
            self.beeper.callback(pose)
            time.sleep(self.timeout / 10)

        assert not mock_beep.called
        assert self.beeper.tracking
        assert self.beeper.last_updated == pose.header.stamp

    @mock.patch.object(beeper.Beeper, "beep", autospec=True)
    def test_not_changing_pose_steady_state(self, mock_beep):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        self.beeper.connected = True
        self.beeper._last_pose = pose
        self.beeper.last_updated = rospy.Time.now()
        last_change_time = self.beeper.last_updated

        for i in range(21):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            self.beeper.callback(pose)
            time.sleep(self.timeout / 10)

        assert mock_beep.called
        assert not self.beeper.tracking
        assert self.beeper.last_updated == last_change_time

    @mock.patch.object(beeper.Beeper, "beep", autospec=True)
    def test_tracking_reacquired(self, mock_beep):
        initial_pose = PoseStamped()
        initial_pose.header.stamp = rospy.Time.now()

        self.beeper.connected = True
        self.beeper._last_pose = initial_pose
        self.beeper.last_updated = rospy.Time.now()
        self.beeper._tracking = False

        time.sleep(self.timeout / 10)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 1

        self.beeper.callback(pose)
        assert not mock_beep.called
        assert self.beeper.tracking
