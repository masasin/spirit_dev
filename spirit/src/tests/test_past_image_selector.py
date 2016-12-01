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
    pass
