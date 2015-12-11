#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Selects the best image for SPIRIT.

"""
from __future__ import division
from time import localtime, strftime

import numpy as np
from numpy.linalg import norm

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from ntree import Octree
from helpers import get_pose_components, tf_from_pose


class Frame(object):
    """
    Encapsulate an image and the pose it was taken in.

    Parameters
    ----------
    pose : PoseStamped
        The pose of the drone when the image was taken.
    image : Image
        The image that was taken.

    Attributes
    ----------
    pose : PoseStamped
        The pose of the drone at which the image was taken.
    image : Image
        The image that was taken.
    coordinates : ndarray
        The rounded x, y, and z coordinates of the pose.
    coords_precise : ndarray
        The x, y, and z coordinates of the pose.
    orientation : ndarray
        The x, y, z, w values of the quaternion
    stamp : rospy.rostime.Time
        The timestamp of the pose.
    stamp_str : str
        The timestamp of the pose, in human readable format.

    """
    def __init__(self, pose, image):
        self.pose = pose
        self.coords_precise, self.orientation = get_pose_components(self.pose)
        self.coordinates = self.coords_precise * 1000 // 100  # 10 cm
        self.image = image
        self.stamp = self.pose.header.stamp
        self.stamp_str = strftime("%Y-%m-%d %H:%M:%S",
                                  localtime(self.stamp.to_time()))

    def distance_to(self, other):
        """
        Calculate the distance to another frame.

        Parameters
        ----------
        other : Frame
            The target frame.

        Returns
        -------
        float
            The distance to the target frame.

        """
        return norm(self.coords_precise - other.coords_precise)

    def __repr__(self):
        """
        Description of the frame.

        """
        return "Frame ({coords}): {time}".format(
            coords=self.coords_precise.tolist(),
            time=self.stamp)


class Evaluators(object):
    """
    Contains evaluation functions.

    Parameters
    ----------
    method : str
        The name of the method to use.
    parent : Selector
        The selector whose attributes to use.

    Attributes
    ----------
    evaluate : callable
        The evaluation function to use.
    save_chrono : bool
        Whether to save frames chronologically.
    save_ntree : bool
        Whether to save frames in an ntree.

    """
    def __init__(self, method, parent):
        self._parent = parent
        self.evaluate = self.__getattribute__(method)
        self.save_chrono = self.save_ntree = False

    def constant_time_delay(self, pose):
        """
        Return the frame delayed by a fixed amount.

        If the delay has not yet passed, return the first frame.

        """
        self.save_chrono = True
        if len(self._parent.frames):
            optimum_timestamp = pose.header.stamp.to_sec() - self._parent._delay

            for frame in reversed(self._parent.frames):
                if frame.stamp.to_sec() < optimum_timestamp:
                    return frame
            return self._parent.frames[0]

    def constant_distance(self, pose):
        """
        Return the frame a fixed distance away.

        If the distance has not yet been crossed, return the last frame.

        """
        self.save_chrono = True
        if len(self._parent.frames):
            optimum_distance = err_current = err_min = self._parent._distance
            position, orientation = get_pose_components(pose)
            for frame in reversed(self._parent.frames):
                frame_distance = norm(frame.coords_precise - position)
                if frame_distance > optimum_distance:
                    err_current = abs(frame_distance - optimum_distance)
                    if (err_current < err_min):
                        return frame
            return self._parent.frames[-1]

    def murata(self, pose):
        """
        Use the evaluation function from Murata's thesis. [#]_

        Notes
        -----
        The evaluation function is:

        .. math::
            E = a_1 ((z_{camera} - z_{ref})/z_{ref})^2 +
                a_2 (β_{xy}/(π / 2))^2 +
                a_3 (α/φ_v)^2 +
                a_4 ((|\mathbf{a}| - L)/L)^2 +
                a_5 (|\mathbf{x}^v - \mathbf{x}^v_{curr}|/
                     |\mathbf{x}^v_{curr}|)^2

        where :math:`a_1` through :math:`a_4` are coefficients, :math:`z` is the
        difference in height of the drone, :math:`α` is the tilt angle,
        :math:`β` is the difference in yaw, :math:`\mathbf{a}` is the distance
        vector, :math:`L` is the optimal distance, and :math:`φ_v` is the angle
        of the vertical field of view.

        :math:`φ_v` is calculated using :math:`φ_h`, the angle of the horizontal
        field of view:

        .. math::
            φ_v = 2\mathrm{arctan}(γ\mathrm{tan}(φ_h/2))

        where :math:`γ` is the aspect ratio of the display.

        References
        ----------
        .. [#] Ryosuke Murata, Undergrad Thesis, Kyoto University, 2013
               過去画像履歴を用いた移動マニピュレータの遠隔操作システム

        """
        def eval_func(pose, frame):
            coords, orientation = get_pose_components(pose)
            best_state_vector = np.hstack((coords, orientation))
            old_state_vector = np.hstack(
                (self._parent.current_frame.coords_precise,
                 self._parent.current_frame.orientation))
            change = best_state_vector - old_state_vector

            a1, a2, a3, a4, a5 = (1, 2, 3, 4, 5)
            z_camera = change[2]
            z_ref = 0.5

            beta = 1
            alpha = 1
            fov_v = np.pi / 3

            a_mag = norm(coords - self._parent.current_frame.coords_precise)
            L_ref = 1

            score = (a1 * ((z_camera - z_ref)/z_ref) ** 2 +
                     a2 * (beta / (np.pi / 2)) ** 2 +
                     a3 * (alpha / fov_v) ** 2 +
                     a4 * ((a_mag - L_ref)/L_ref) ** 2 +
                     a5 * (change / old_state_vector) ** 2)

            return score

        self.save_chrono = True

        if self._parent.current_frame is None:
            return self._parent.frames[0]

        if len(self._parent.frames):
            results = {}
            for frame in reversed(self._parent.frames):
                results[frame] = eval_func(pose, frame)
            return min(results, key=results.get)

    def spirit(self, pose):
        """
        Not implemented yet.

        """
        def eval_func(pose, frame):
            pass

        self.save_ntree = True
        raise NotImplementedError

        position, orientation = get_pose_components(pose)
        nearest_ten = self._parent.octree.get_nearest(position, k=10)
        results = {}
        for location in nearest_ten:
            for frame in location:
                results[frame] = eval_func(pose, frame)
        return min(results, key=results.get)


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
    octree : Octree
        The octree holding all frames according to their position in 3D.
    frames : list of Frame
        A chronological list of frames.
    evaluate : callable
        The evaluation function to use.
    past_image_pub : rospy.Publisher
        The publisher for the past images.
    tf2_pub : tf2_ros.TransformBroadcaster
        The publisher for TF.

    Raises
    ------
    KeyError
        If the rosparam has not been set.

    """
    def __init__(self):
        self.clear()
        self.current_frame = None

        method = rospy.get_param("~eval_method")
        _evaluator = Evaluators(method, parent=self)
        self.save_chrono = _evaluator.save_chrono
        self.save_ntree = _evaluator.save_ntree
        self.evaluate = _evaluator.evaluate

        if self.save_chrono:
            self.frames = []  # Chronological
        if self.save_ntree:
            self.ntree = Octree((0, 0, 0), 1000)  # 100 m per side

        rospy.Subscriber("/ardrone/slow_image_raw", Image, self.image_callback)
        rospy.Subscriber("/ardrone/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/ardrone/tracked", Bool, self.tracked_callback)

        self.past_image_pub = rospy.Publisher("/ardrone/past_image", Image,
                                              queue_size=1)
        self.tf_pub = tf2_ros.TransformBroadcaster()

    def image_callback(self, image):
        """
        Update `image`, and store frames if all the data is available.

        """
        rospy.logdebug("New image")
        self.image = image
        if self.can_make_frame:
            rospy.logdebug("Adding frames to octree and queue")
            frame = Frame(self.pose, self.image)
            if self.save_chrono:
                self.frames.append(frame)
            if self.save_ntree:
                self.ntree.insert(frame)
            self.clear()

    def pose_callback(self, pose):
        """
        Update tf and `pose`, and select the best past image.

        """
        rospy.logdebug("New pose")
        self.pose = pose
        self.tf_pub.sendTransform(tf_from_pose(pose, child="ardrone"))

        best_frame = self.evaluate(pose)
        if best_frame is not None:
            self.current_frame = best_frame
            self.past_image_pub.publish(best_frame.image)

    def tracked_callback(self, tracked):
        """
        Update the `tracked` variable.

        """
        self.tracked = tracked.data

    @property
    def can_make_frame(self):
        """
        Check if we can make a frame.

        """
        return self.image and self.pose and self.tracked

    def clear(self):
        """
        Reset status attributes to default values.

        """
        self.image = None
        self.pose = None
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
        if name in ("_delay", "_distance"):
            self.__setattr__(name, rospy.get_param("~{n}".format(n=name[1:])))
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
