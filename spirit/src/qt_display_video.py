#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
A Qt display window.

Displays a video feed, and can also show the drone state and connection problems
in the status bar.

"""
import sys
from threading import Lock

from PyQt5 import QtCore, QtGui, QtWidgets

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata

from drone_state import DroneState


CONNECTION_CHECK_PERIOD = 250  # ms
GUI_UPDATE_PERIOD = 20  # ms


class DroneVideoDisplay(QtWidgets.QMainWindow):
    state_messages = {
        DroneState.Emergency: "Emergency",
        DroneState.Inited: "Initialized",
        DroneState.Landed: "Landed",
        DroneState.Flying: "Flying",
        DroneState.Hovering: "Hovering",
        DroneState.Test: "Test",
        DroneState.TakingOff: "Taking Off",
        DroneState.GotoHover: "Going to Hover Mode",
        DroneState.Landing: "Landing",
        DroneState.Looping: "Looping"
    }
    msg_disconnected = "Disconnected"
    msg_unknown = "Unknown State"
    msg_status_template = "{state} (Battery: {battery:.0f}%){tracked}"

    def __init__(self):
        super(DroneVideoDisplay, self).__init__()

        # Setup GUI - a label which fills the whole window and holds our image.
        self.setWindowTitle(rospy.get_param("~window_name",
                                            "AR.Drone Video Feed"))
        self.image_box = QtWidgets.QLabel(self)
        self.setCentralWidget(self.image_box)

        rospy.Subscriber("/ardrone/navdata", Navdata, self.cbk_navdata)
        rospy.Subscriber("/ardrone/tracked", Bool, self.cbk_tracked,
                         queue_size=1)
        rospy.Subscriber("image", Image, self.cbk_image, queue_size=1)

        # Holds drone tracking status.
        self.tracked = False

        # Holds image frame received from drone for processing by GUI.
        self.image = None
        self._image_lock = Lock()

        # Holds the status message to be displayed on the next GUI update.
        self.msg_status_bar = ""

        # Tracks whether we have received data since the last connection check.
        self._comm_since_timer = False

        # A timer to check whether we"re still connected.
        self.timer_connection = QtCore.QTimer(self)
        self.timer_connection.timeout.connect(self.cbk_connection)
        self.timer_connection.start(CONNECTION_CHECK_PERIOD)

        # A timer to redraw the GUI.
        self.timer_redraw = QtCore.QTimer(self)
        self.timer_redraw.timeout.connect(self.cbk_redraw)
        self.timer_redraw.start(GUI_UPDATE_PERIOD)

    @property
    def is_connected(self):
        return self._comm_since_timer

    def cbk_connection(self):
        """
        Called every CONNECTION_CHECK_PERIOD.

        If we haven"t received anything since the last callback, we will assume
        that we are having network troubles and display a message in the status
        bar.

        """
        self._comm_since_timer = False

    def cbk_redraw(self):
        if self.image is not None:
            with self._image_lock:
                # Convert the ROS image into a QImage which we can display.
                image = QtGui.QPixmap.fromImage(
                    QtGui.QImage(self.image.data, self.image.width,
                                 self.image.height, QtGui.QImage.Format_RGB888))

            # Further processing can be done here.
            self.resize(image.width(), image.height())
            self.image_box.setPixmap(image)

        # Update the status bar.
        self.statusBar().showMessage(
            self.msg_status_bar if self.is_connected
            else self.msg_disconnected)

    def cbk_image(self, data):
        self._comm_since_timer = True

        with self._image_lock:
            self.image = data  # Save ros image for display thread processing.

    def cbk_navdata(self, navdata):
        self._comm_since_timer = True

        # Update the message to be displayed.
        state = self.state_messages.get(navdata.state, self.msg_unknown)
        self.msg_status_bar = self.msg_status_template.format(
            state=state,
            battery=navdata.batteryPercent,
            tracked="" if self.tracked else " TRACKING LOST"
        )

    def cbk_tracked(self, tracked):
        self.tracked = tracked.data


def main():
    rospy.init_node("ardrone_video_display")
    app = QtWidgets.QApplication(sys.argv)
    display = DroneVideoDisplay()
    display.show()
    app.exec_()
    rospy.spin()


if __name__ == '__main__':
    main()
