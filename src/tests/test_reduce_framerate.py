try:
    from unittest import mock
except ImportError:
    import mock

from sensor_msgs.msg import Image

from reduce_framerate import FramerateReducer


def test_callback():
    r = FramerateReducer()
    with mock.patch.object(r, "image_publisher", autospec=True) as mock_pub:
        for i in range(16):
            r.frame_callback(Image())
        assert mock_pub.publish.call_count == 2  # At 0 and 15
