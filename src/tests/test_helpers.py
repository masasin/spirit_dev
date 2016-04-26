from __future__ import division
import time

import mock
import numpy as np

from geometry_msgs.msg import Point, Quaternion, PoseStamped, TransformStamped

from helpers import (unit_vector, get_pose_components, pose_from_components,
                     pose_from_tf, tf_from_pose, angle_between,
                     angle_between_quaternions, fov_diagonal2vertical,
                     fov_vertical2horizontal)


class TestUnitVector(object):
    def test_regular_unit_vector(self):
        assert np.isclose(unit_vector([1, 0, -1]),
                          np.array([np.sqrt(0.5), 0, -np.sqrt(0.5)])).all()

    def test_zero_vector(self):
        assert (unit_vector([0, 0, 0]) == np.zeros(3)).all()


class TestPoseAndComponents(object):
    def setup(self):
        self.position = np.array([0, 1, 2])
        self.orientation = np.array([3, 4, 5, 6])
        self.components = (self.position, self.orientation)

        self.pose = PoseStamped()
        self.pose.pose.position = Point(*self.position)
        self.pose.pose.orientation = Quaternion(*self.orientation)

    def test_get_pose_components(self):
        assert all([(i == j).all()
                    for i, j in zip(get_pose_components(self.pose),
                                    self.components)])

    def test_pose_from_components(self):
        assert pose_from_components(*self.components) == self.pose

    def test_invariance(self):
        assert (pose_from_components(*get_pose_components(self.pose))
                == self.pose)
        assert all([(i == j).all()
                    for i, j in zip(
                        get_pose_components(
                            pose_from_components(*self.components)),
                        self.components)])


class TestPoseAndTf(object):
    def setup(self):
        self.position = [0, 1, 2]
        self.orientation = [3, 4, 5, 6]

        self.pose = PoseStamped()
        self.pose.pose.position = Point(*self.position)
        self.pose.pose.orientation = Quaternion(*self.orientation)

        self.tf = TransformStamped()
        self.tf.transform.translation = Point(*self.position)
        self.tf.transform.rotation = Quaternion(*self.orientation)

    def test_pose_from_tf(self):
        with mock.patch("rospy.Time.now"):
            assert pose_from_tf(self.tf).pose == self.pose.pose

    def test_tf_from_pose(self):
        with mock.patch("rospy.Time.now"):
            assert tf_from_pose(self.pose).transform == self.tf.transform

    def test_invariance(self):
        with mock.patch("rospy.Time.now"):
            assert pose_from_tf(tf_from_pose(self.pose)).pose == self.pose.pose
            assert (tf_from_pose(pose_from_tf(self.tf)).transform
                    == self.tf.transform)


class TestAngle(object):
    def test_angle_between_any_two_vectors(self):
        assert angle_between([1, 0, 0], [0, 1, 0]) == np.pi / 2
        assert angle_between([1, 0, 0], [1, 0, 0]) == 0
        assert angle_between([1, 0, 0], [-1, 0, 0]) == np.pi
        assert angle_between([0, 0, 0, 1], [0, 0, 0, 1]) == 0
        assert np.isclose(angle_between([0, 0, 0, 1], [0, -1, 0, 1]), np.pi / 4)

    def test_angle_between_quaternions(self):
        assert angle_between_quaternions([0, 0, 0, 1], [0, 0, 0, 1]) == 0
        assert np.isclose(angle_between_quaternions([0, 0, 0, 1], [0, -1, 0, 1]),
                          np.pi / 2)


class TestFieldOfView(object):
    def test_fov_diagonal_to_vertical(self):
        assert np.isclose(fov_diagonal2vertical(73, 16 / 9), 40, rtol=0.1)
        assert np.isclose(fov_diagonal2vertical(83, 16 / 9), 47, rtol=0.1)
        assert np.isclose(fov_diagonal2vertical(90, 16 / 9), 52, rtol=0.1)
        assert np.isclose(fov_diagonal2vertical(98, 16 / 9), 59, rtol=0.1)
        assert np.isclose(fov_diagonal2vertical(113, 16 / 9), 73, rtol=0.1)

        assert np.isclose(fov_diagonal2vertical(69, 16 / 10), 40, rtol=0.1)
        assert np.isclose(fov_diagonal2vertical(84, 16 / 10), 51, rtol=0.1)
        assert np.isclose(fov_diagonal2vertical(90, 16 / 10), 56, rtol=0.1)
        assert np.isclose(fov_diagonal2vertical(99, 16 / 10), 64, rtol=0.1)
        assert np.isclose(fov_diagonal2vertical(109, 16 / 10), 73, rtol=0.1)

    def test_fov_vertical_to_horizontal(self):
        assert np.isclose(fov_vertical2horizontal(40, 16 / 9), 66, rtol=0.1)
        assert np.isclose(fov_vertical2horizontal(47, 16 / 9), 75, rtol=0.1)
        assert np.isclose(fov_vertical2horizontal(52, 16 / 9), 82, rtol=0.1)
        assert np.isclose(fov_vertical2horizontal(59, 16 / 9), 90, rtol=0.1)
        assert np.isclose(fov_vertical2horizontal(73, 16 / 9), 106, rtol=0.1)

        assert np.isclose(fov_vertical2horizontal(40, 16 / 10), 60, rtol=0.1)
        assert np.isclose(fov_vertical2horizontal(51, 16 / 10), 75, rtol=0.1)
        assert np.isclose(fov_vertical2horizontal(56, 16 / 10), 81, rtol=0.1)
        assert np.isclose(fov_vertical2horizontal(64, 16 / 10), 90, rtol=0.1)
        assert np.isclose(fov_vertical2horizontal(73, 16 / 10), 100, rtol=0.1)
