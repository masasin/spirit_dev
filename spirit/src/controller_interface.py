#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
A controller interface for the drone.

"""
from __future__ import division

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty
from ardrone_autonomy.msg import Navdata

from drone_state import DroneState


COMMAND_PERIOD = 100  # ms
FLYING_STATES = (DroneState.Flying, DroneState.GotoHover, DroneState.Hovering)


class DroneController(object):
    """
    Interface with which to control the drone.

    Attributes
    ----------
    state : int
        The state of the drone, as defined in `drone_state`.
    arrived : bool
        Whether the user believes that the drone has arrived at the target.
    pub_land : rospy.Publisher
        Publisher to /ardrone/land. Initiates landing.
    pub_reset : rospy.Publisher
        Publisher to /ardrone/reset. Resets the drone.
    pub_takeoff : rospy.Publisher
        Publisher to /ardrone/takeoff. Initiates takeoff.
    pub_command : rospy.Publisher
        Publisher to /cmd_vel. Controls the drone.
    pub_arrived : rospy.Publisher
        Publisher to /ardrone/arrived. Records whether the user believes that
        the drone is at the target.
    command : Twist
        Twist message containing the current commands to the drone.

    """
    def __init__(self):
        self.state = None
        self.arrived = False

        # Subscribe to navdata
        rospy.Subscriber('/ardrone/navdata', Navdata, self.update_state)

        # Publish to the /ardrone/take_off, land and reset topics
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty,
                                           queue_size=1)
        self.pub_reset = rospy.Publisher('/ardrone/reset', Empty, queue_size=1)

        # Publish to /cmd_vel to control drone
        self.pub_command = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Publish arrival status.
        self.pub_arrived = rospy.Publisher("/ardrone/arrived", Bool,
                                           queue_size=1)

        # Setup regular publishing of control packets
        self.command = Twist()
        rospy.Timer(rospy.Duration(COMMAND_PERIOD / 1000), self.send_command)
        rospy.Timer(rospy.Duration(COMMAND_PERIOD / 1000),
                    self.publish_arrival_status)

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.land)

    def update_state(self, navdata):
        """
        Update state.

        Parameters
        ----------
        navdata : Navdata
            A navdata message.

        """
        self.state = navdata.state

    def take_off(self):
        """
        Send a take_off message.

        Only take off when the drone is landed.

        """
        if self.state == DroneState.Landed:
            self.pub_takeoff.publish(Empty())

    def land(self):
        """
        Send a land message.

        """
        self.pub_land.publish(Empty())

    def emergency(self):
        """
        Send a reset message.

        """
        self.pub_reset.publish(Empty())

    def publish_arrival_status(self, event):
        """
        Publish the arrival status.

        Parameters
        ----------
        event : TimerEvent
            Used by the Timer.

        """
        self.pub_arrived.publish(Bool(self.arrived))

    def set_command(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        """
        Set the current command.

        Parameters
        ----------
        roll : float
            The roll input.
        pitch : float
            The pitch input.
        yaw_velocity : float
            The yaw input.
        z_velocity : float
            The z-velocity input.

        """
        self.command.linear.x = pitch
        self.command.linear.y = roll
        self.command.linear.z = z_velocity
        self.command.angular.z = yaw_velocity

    def send_command(self, event):
        """
        Send the current command to the drone if it is already flying.

        Parameters
        ----------
        event : TimerEvent
            Used by the Timer.

        """
        if self.state in FLYING_STATES:
            self.pub_command.publish(self.command)
