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
from helpers import Pose, Frame


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
    def __init__(self, image_queue_length=None, eval_method=None, debug=False):
        if image_queue_length is None:
            image_queue_length = rospy.get_param("~image_queue_length")
            if image_queue_length == "None":
                image_queue_length = None
        if eval_method is None:
            eval_method = rospy.get_param("~eval_method")

        self.clear()

        self.frames = deque([], image_queue_length)

        self.image = None
        self.pose = None
        self.current_frame = None
        self.tracked = None
        self.debug = debug

        self.evaluator = get_evaluator(eval_method, parent=self)

        with open(os.path.join(rospkg.RosPack().get_path("spirit"),
                               "config", "launch_params.yaml")) as fin:
            params = yaml.load(fin)
        self.eval_method_params = params["past_image"][eval_method].keys()
        self.eval_coeffs = OrderedDict()
        for param, coefficient in params["past_image"][eval_method].items():
            if param.startswith("coeff_") and coefficient != 0:
                component = param.split("coeff_", 1)[1]
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
        if self.can_make_frame and (self.moved or not self.frames):
            rospy.logdebug("Adding frames to queue")
            frame = Frame(self._pose_stamped, self.image)
            self.frames.append(frame)
            self.clear()

    def pose_callback(self, pose_stamped):
        """
        Update `pose`, and select the best past image.

        Short circuits if an evaluation function is still being run.

        Parameters
        ----------
        pose_stamped : PoseStamped
            A pose message.

        """
        rospy.logdebug("New pose")
        self._pose_stamped = pose_stamped
        self.pose = Pose(pose_stamped)

        if self.evaluator.is_busy:
            return

        best_frame = self.evaluator.select_best_frame()
        if best_frame is not None:
            self.current_frame = best_frame
            if not self.debug:
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

    @property
    def moved(self):
        """
        Check if we have moved significantly.

        If there are no thresholds, return True.

        Returns
        -------
        bool
            Whether we have moved.

        """
        if (self.thresh_distance is None) and (self.thresh_yaw is None):
            return True

        if self.frames:
            if self.thresh_distance is not None:
                distance = self.frames[-1].distance(self.pose)
                if distance > self.thresh_distance:
                    return True

            if self.thresh_yaw is not None:
                yaw = self.current_frame.rel_euler(self.pose)[2]
                if yaw > self.thresh_yaw:
                    return True

        return False

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
    rospy.init_node("past_image_selector", log_level=rospy.INFO)
    Selector()
    rospy.loginfo("Started the past image selector")
    rospy.spin()


if __name__ == "__main__":
    main()
