#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
Select the best image for SPIRIT.

"""
from __future__ import division
from collections import deque, OrderedDict
import os

import yaml

import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from evaluators import get_evaluator
from helpers import (get_pose_components, rotation_matrix, quaternion_rotation,
                     quat2euler)


class Pose(object):
    def __init__(self, pose_stamped):
        self._pose = pose_stamped
        self.position, self.orientation = get_pose_components(pose_stamped)
        self.header = pose_stamped.header

    def dpos(self, pose, rotmat=None):
        if rotmat is None:
            rotmat = rotation_matrix(self.orientation)
        return rotmat.dot(pose.position - self.position)

    def _dquat(self, pose):
        return quaternion_rotation(pose.orientation, self.orientation)

    def deuler(self, pose):
        return quat2euler(self._dquat(pose))

    def distance(self, pose):
        """
        Calculate the distance to another frame.

        Parameters
        ----------
        pose : Pose
            The target pose.

        Returns
        -------
        float
            The distance to the target frame.

        """
        return norm(pose.position - self.position)


class Frame(object):
    """
    Encapsulate an image and the pose it was taken in.

    Parameters
    ----------
    pose_stamped : PoseStamped
        The pose of the drone when the image was taken.
    image : Image
        The image that was taken.

    Attributes
    ----------
    pose_stamped : PoseStamped
        The pose of the drone at which the image was taken.
    pose : Pose
        The pose of the drone at which the image was taken.
    rotmat : np.ndarray
        The rotation matrix of the frame orientation.
    image : Image
        The image that was taken.
    stamp : rospy.rostime.Time
        The timestamp of the pose.
    stamp_str : str
        The timestamp of the pose, in human readable format.

    """
    def __init__(self, pose_stamped, image):
        self.pose_stamped = pose_stamped
        self.pose = Pose(pose_stamped)
        self.rotmat = rotation_matrix(self.pose.orientation)
        self.image = image
        self.stamp = self.pose.header.stamp
        self.stamp_str = strftime("%Y-%m-%d %H:%M:%S",
                                  localtime(self.stamp.to_time()))

    def dpos(self, pose):
        return self.pose.dpos(pose, rotmat=self.rotmat)

    def deuler(self, pose):
        return self.pose.deuler(pose)

    def distance(self, pose):
        return self.pose.distance(pose)

    def __str__(self):
        return "Frame ({position}): {time}".format(
            position=self.pose.position.tolist(),
            time=self.stamp)


class Selector(object):
    """
    Selects and publishes the best image for SPIRIT.

    The evaluation function is determined by a rosparam which must be set before
    launch.

    Attributes
    ----------
    can_make_frame
    current_frame : Frame
        The current frame which is being shown.
    frames : list of Frame
        A chronological list of frames.
    past_image_pub : rospy.Publisher
        The publisher for the past images.

    Raises
    ------
    KeyError
        If the rosparam has not been set.

    """
    def __init__(self):
        self.clear()
        self.current_frame = None

        self.frames = deque([], rospy.get_param("~image_queue_length"))

        self.image = None
        self.pose = None
        self.current_frame = None
        self.tracked = None

        eval_method = rospy.get_param("~eval_method")
        self.evaluator = get_evaluator(eval_method, parent=self)

        with open(os.path.join(rospkg.RosPack().get_path("spirit"),
                               "config", "launch_params.yaml")) as fin:
            params = yaml.load(fin)
        self.eval_method_params = params["past_image"][eval_method].keys()
        self.eval_coeffs = OrderedDict()
        for param, coefficient in params["past_image"][eval_method].items():
            if param.startswith("coeff_"):
                component = param.split("coeff_", maxsplit=1)[1]
                self.eval_coeffs[component] = coefficient

        rospy.Subscriber("/ardrone/slow_image_raw", Image, self.image_callback)
        rospy.Subscriber("/ardrone/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/ardrone/tracked", Bool, self.tracked_callback)

        self.past_image_pub = rospy.Publisher("/ardrone/past_image", Image,
                                              queue_size=1)
        self.past_pose_pub = rospy.Publisher("/ardrone/past_pose", PoseStamped,
                                             queue_size=1)

    def image_callback(self, image):
        """
        Update `image`, and store frames if all the data is available.

        Parameters
        ----------
        image : Image
            An image message.

        """
        rospy.logdebug("New image")
        self.image = image
        if self.can_make_frame:
            rospy.logdebug("Adding frames to queue")
            frame = Frame(self._pose_stamped, self.image)
            self.frames.append(frame)
            self.clear()

    def pose_callback(self, pose_stamped):
        """
        Update `pose`, and select the best past image.

        Parameters
        ----------
        pose_stamped : PoseStamped
            A pose message.

        """
        rospy.logdebug("New pose")
        self._pose_stamped = pose_stamped
        self.pose = Pose(pose_stamped)

        best_frame = self.evaluator.evaluate()
        if best_frame is not None:
            self.current_frame = best_frame
            self.past_image_pub.publish(best_frame.image)
            self.past_pose_pub.publish(best_frame.pose_stamped)

    def tracked_callback(self, tracked):
        """
        Update the `tracked` variable.

        Parameters
        ----------
        tracked : Bool
            Whether the drone is being tracked.

        """
        self.tracked = tracked.data

    @property
    def can_make_frame(self):
        """
        Check if we can make a frame.

        Returns
        -------
        bool
            Whether we can make a frame.

        """
        return self.image and self.pose and self.tracked

    def clear(self):
        """
        Reset status attributes to default values.

        """
        self.image = None
        self.tracked = None

    def __getattr__(self, name):
        """
        Return undefined attributes.

        Upon first access, the value of the parameter is obtained from the ros
        parameter dictionary and stored as an attribute, from where it is read
        on subsequent accesses.

        Parameters
        ----------
        name : str
            The attribute to get.

        Raises
        ------
        AttributeError
            If the attribute does not exist.
        KeyError
            If the ros parameter has not been defined.

        """
        if name in self.eval_method_params:
            self.__setattr__(name, rospy.get_param("~{n}".format(n=name)))
        return self.__getattribute__(name)


def main():
    """
    Main entry point for script.

    """
    rospy.init_node("past_image_selector")
    Selector()
    rospy.loginfo("Started the past image selector")
    rospy.spin()


if __name__ == "__main__":
    main()
