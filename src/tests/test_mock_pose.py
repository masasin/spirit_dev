try:
    from unittest.mock import patch, MagicMock
except ImportError:
    from mock import patch, MagicMock

import pytest

import rospy

MockTf2 = MagicMock()
modules = {"tf2_ros": MockTf2}
patcher = patch.dict("sys.modules", modules)
patcher.start()


try:
    rospy.init_node("pytest", anonymous=True)
except rospy.exceptions.ROSException:
    pass


@pytest.fixture(scope="module")
def teardown_module():
    def fin():
        patcher.stop()


class TestPoseGenerator(object):
    def test_tf_and_pose_same(self):
        from mock_pose import PoseGenerator

        pose = PoseGenerator.generate_pose()
        transform = PoseGenerator.pose_to_tf(pose)

        assert transform.transform.translation.x == pose.pose.position.x
        assert transform.transform.translation.y == pose.pose.position.y
        assert transform.transform.translation.z == pose.pose.position.z

        assert transform.transform.rotation.x == pose.pose.orientation.x
        assert transform.transform.rotation.y == pose.pose.orientation.y
        assert transform.transform.rotation.z == pose.pose.orientation.z
        assert transform.transform.rotation.w == pose.pose.orientation.w
