#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
A controller interface for the drone.

"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata

from drone_state import DroneState


# Some Constants
COMMAND_PERIOD = 100  # ms


class DroneController(object):
    def __init__(self):
        self.state = None

        # Subscribe to navdata
        rospy.Subscriber('/ardrone/navdata', Navdata, self.update_state)

        # Publish to the /ardrone/takeoff, land and reset topics
        self.pub_land = rospy.Publisher('/ardrone/land', Empty)
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
        self.pub_reset = rospy.Publisher('/ardrone/reset', Empty)

        # Publish to /cmd_vel to control drone
        self.pub_command = rospy.Publisher('/cmd_vel', Twist)

        # Setup regular publishing of control packets
        self.command = Twist()
        rospy.Timer(rospy.Duration(COMMAND_PERIOD / 1000.0), self.send_command)

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.land)

    def update_state(self, navdata):
        self.state = navdata.state

    def takeoff(self):
        """Send a takeoff message if landed."""
        if self.state == DroneState.Landed:
            self.pub_takeoff.publish(Empty())

    def land(self):
        """Send a land message."""
        self.pub_land.publish(Empty())

    def emergency(self):
        """Send a reset message."""
        self.pub_reset.publish(Empty())

    def set_command(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        """Set the current command."""
        self.command.linear.x = pitch
        self.command.linear.y = roll
        self.command.linear.z = z_velocity
        self.command.angular.z = yaw_velocity

    def send_command(self, event):
        """Send the current command based on the periodic timer."""
        if self.state in (DroneState.Flying, DroneState.GotoHover,
                          DroneState.Hovering):
            self.pub_command.publish(self.command)
