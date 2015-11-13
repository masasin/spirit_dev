from unittest import mock

from geometry_msgs.msg import PoseStamped
import rospy
from beeper import Beeper


class TestBeeper(object):
    def setup(self):
        rospy.init_node("beeper_test", anonymous=True)
        self.beeper = Beeper()
        self.pose = PoseStamped()

    @mock.patch.object(Beeper, "beep", autospec=True)
    def test_beep(self, mock_beep):
        self.beeper.beep()
        mock_beep.assert_called_with(self.beeper)


