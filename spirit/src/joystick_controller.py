#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
A joystick controller for the drone.

"""
from __future__ import division
import sys

import rospy
from sensor_msgs.msg import Joy

from controller_interface import DroneController


class Button(object):
    """
    Button mapping.

    By default, the PS3 controller is used with the following mapping:

    - Landing: L2
    - Arrived at Destination: R2
    - Takeoff: L1
    - Emergency: R1

    """
    land = 8
    arrived = 9
    takeoff = 10
    emergency = 11


class Axis(object):
    """
    Axis mapping.

    By default, the PS3 controller is used with the following mapping:

    - Roll: Left analog, left-right
    - Pitch: Left analog, up-down
    - Yaw: Right analog, left-right
    - Height: Right analog, up-down

    """
    roll = 0
    pitch = 1
    yaw = 2
    height = 3


class Scale(object):
    """
    Scale mapping.

    """
    roll = pitch = yaw = height = 1


def handle_joystick(data):
    """
    Receive and handle joystick message data.

    Parameters
    ----------
    data : Joy
        Joystick data.

    """
    if data.buttons[Button.emergency]:
        rospy.loginfo("Emergency Button Pressed")
        controller.emergency()
    elif data.buttons[Button.land]:
        rospy.loginfo("Land Button Pressed")
        controller.land()
    elif data.buttons[Button.takeoff]:
        rospy.loginfo("Takeoff Button Pressed")
        controller.take_off()
    else:
        controller.set_command(data.axes[Axis.roll] * Scale.roll,
                               data.axes[Axis.pitch] * Scale.pitch,
                               data.axes[Axis.yaw] * Scale.yaw,
                               data.axes[Axis.height] * Scale.height)

    if data.buttons[Button.arrived]:
        rospy.loginfo("Arrival Button Pressed")
        controller.arrived = True
    else:
        controller.arrived = False


def update_mappings():
    """
    Update the button, axis, and scale mappings from the launch file.

    """
    Button.emergency = int(rospy.get_param("~ButtonEmergency",
                                           Button.emergency))
    Button.land = int(rospy.get_param("~ButtonLand", Button.land))
    Button.takeoff = int(rospy.get_param("~ButtonTakeoff", Button.takeoff))
    Button.arrived = int(rospy.get_param("~ButtonArrived", Button.arrived))
    Axis.roll = int(rospy.get_param("~AxisRoll", Axis.roll))
    Axis.pitch = int(rospy.get_param("~AxisPitch", Axis.pitch))
    Axis.yaw = int(rospy.get_param("~AxisYaw", Axis.yaw))
    Axis.height = int(rospy.get_param("~AxisZ", Axis.height))
    Scale.roll = float(rospy.get_param("~ScaleRoll", Scale.roll))
    Scale.pitch = float(rospy.get_param("~ScalePitch", Scale.pitch))
    Scale.yaw = float(rospy.get_param("~ScaleYaw", Scale.yaw))
    Scale.height = float(rospy.get_param("~ScaleZ", Scale.height))


if __name__ == "__main__":
    rospy.init_node("ardrone_joystick_controller")
    controller = DroneController()
    update_mappings()
    rospy.Subscriber("/joy", Joy, handle_joystick)
    rospy.spin()
