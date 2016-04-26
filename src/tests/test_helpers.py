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
        assert (get_pose_components(self.pose) == self.components).all()

    def test_pose_from_components(self):
        assert pose_from_components(*self.components) == self.pose

    def test_invariance(self):
        assert (pose_from_components(*get_pose_components(self.pose))
                == self.pose)
        assert (get_pose_components(pose_from_components(*self.components))
                == self.components).all()


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
        assert pose_from_tf(self.tf) == self.pose

    def test_tf_from_pose(self):
        assert tf_from_pose(self.pose) == self.tf

    def test_invariance(self):
        assert pose_from_tf(tf_from_pose(self.pose)) == self.pose
        assert tf_from_pose(pose_from_tf(self.tf)) == self.tf


class TestAngle(object):
    def test_angle_between_any_two_vectors(self):
        assert angle_between([1, 0, 0], [0, 1, 0]) == np.pi / 2
        assert angle_between([1, 0, 0], [1, 0, 0]) == 0
        assert angle_between([1, 0, 0], [-1, 0, 0]) == np.pi
        assert angle_between([0, 0, 0, 1], [0, 0, 0, 1]) == 0
        assert angle_between([0, 0, 0, 1], [0, -1, 0, 1]) == np.pi / 4

    def test_angle_between_quaternions(self):
        assert angle_between_quaternions([0, 0, 0, 1], [0, 0, 0, 1]) == 0
        assert angle_between_quaternions([0, 0, 0, 1],
                                         [0, -1, 0, 1]) == np.pi / 2


class TestFieldOfView(object):
    def test_fov_diagonal_to_vertical(self):
        assert np.isclose(fov_diagonal2vertical(73, 16 / 9), 40)
        assert np.isclose(fov_diagonal2vertical(83, 16 / 9), 47)
        assert np.isclose(fov_diagonal2vertical(90, 16 / 9), 52)
        assert np.isclose(fov_diagonal2vertical(98, 16 / 9), 59)
        assert np.isclose(fov_diagonal2vertical(113, 16 / 9), 73)

        assert np.isclose(fov_diagonal2vertical(69, 16 / 10), 40)
        assert np.isclose(fov_diagonal2vertical(84, 16 / 10), 51)
        assert np.isclose(fov_diagonal2vertical(90, 16 / 10), 56)
        assert np.isclose(fov_diagonal2vertical(99, 16 / 10), 64)
        assert np.isclose(fov_diagonal2vertical(109, 16 / 10), 73)

    def test_fov_vertical_to_horizontal(self):
        assert np.isclose(fov_vertical2horizontal(40, 16 / 9), 66)
        assert np.isclose(fov_vertical2horizontal(47, 16 / 9), 75)
        assert np.isclose(fov_vertical2horizontal(52, 16 / 9), 82)
        assert np.isclose(fov_vertical2horizontal(59, 16 / 9), 90)
        assert np.isclose(fov_vertical2horizontal(73, 16 / 9), 106)

        assert np.isclose(fov_vertical2horizontal(40, 16 / 10), 60)
        assert np.isclose(fov_vertical2horizontal(51, 16 / 10), 75)
        assert np.isclose(fov_vertical2horizontal(56, 16 / 10), 81)
        assert np.isclose(fov_vertical2horizontal(64, 16 / 10), 90)
        assert np.isclose(fov_vertical2horizontal(73, 16 / 10), 100)
