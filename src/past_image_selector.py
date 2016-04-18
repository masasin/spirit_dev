#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Selects the best image for SPIRIT.

"""
from __future__ import division
from time import localtime, strftime
import os

import numpy as np
from numpy.linalg import norm
import yaml

import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from ntree import Octree
from helpers import (get_pose_components, fov_diagonal2vertical,
                     angle_between_quaternions, d2r)


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
    coordinates : np.ndarray
        The rounded x, y, and z coordinates of the pose.
    coords_precise : np.ndarray
        The x, y, and z coordinates of the pose.
    orientation : np.ndarray
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

    """
    def __init__(self, method, parent):
        self._parent = parent
        self.evaluate = self.__getattribute__(method)

    def constant_time_delay(self):
        """
        Return the frame delayed by a fixed amount.

        If the delay has not yet passed, return the first frame.

        """
        if self.frames:
            optimum_timestamp = self.pose.header.stamp.to_sec() - self.delay

            for frame in reversed(self.frames):
                if frame.stamp.to_sec() < optimum_timestamp:
                    return frame
            return self.frames[0]

    def constant_distance(self):
        """
        Return the frame a fixed distance away.

        If the distance has not yet been crossed, return the last frame.

        """
        if self.frames:
            optimum_distance = err_min = self.distance
            position, orientation = get_pose_components(self.pose)
            for frame in reversed(self.frames):
                frame_distance = norm(frame.coords_precise - position)
                if frame_distance > optimum_distance:
                    err_current = abs(frame_distance - optimum_distance)
                    if err_current < err_min:
                        return frame
            return self.frames[-1]

    def murata(self):
        """
        Use the evaluation function from Murata's thesis. [#]_

        Extended Notes
        --------------
        The evaluation function is:

        .. math::
            E = a1 ((z_{camera} - z_{ref})/z_{ref})^2 +
                a2 (β_{xy}/(π / 2))^2 +
                a3 (α/φ_v)^2 +
                a4 ((|\mathbf{a}| - l_ref)/l_ref)^2 +
                a5 (|\mathbf{x}^v - \mathbf{x}^v_{curr}|/
                     |\mathbf{x}^v_{curr}|)^2

        where :math:`a1` through :math:`a5` are coefficients, :math:`z` is the
        difference in height of the drone, :math:`α` is the tilt angle,
        :math:`β` is the difference in yaw, :math:`\mathbf{a}` is the distance
        vector, :math:`l_ref` is the optimal distance, and :math:`φ_v` is the
        angle of the vertical field of view.

        References
        ----------
        .. [#] Ryosuke Murata, Undergrad Thesis, Kyoto University, 2013
               過去画像履歴を用いた移動マニピュレータの遠隔操作システム

        """
        def eval_func(pose, frame):
            coords, orientation = get_pose_components(pose)
            current_state_vector = np.hstack((self.current_frame.coords_precise,
                                              self.current_frame.orientation))
            frame_state_vector = np.hstack((frame.coords_precise,
                                            frame.orientation))

            dx, dy, dz = coords - frame.coords_precise
            beta = angle_between_quaternions(orientation, frame.orientation)
            alpha = np.arctan2(dz, dy)
            fov_y = d2r(fov_diagonal2vertical(92))

            a_mag = norm(coords - frame.coords_precise)
            # if a_mag < self.l_ref:
            #    return float("inf")

            return (
                self.coeff_height * ((dz - self.z_ref) / self.z_ref) ** 2
                + self.coeff_direction * (beta / (np.pi / 2)) ** 2
                + self.coeff_elevation * (alpha / fov_y) ** 2
                + self.coeff_distance * ((a_mag - self.l_ref) / self.l_ref) ** 2
                + self.coeff_similarity * (
                    norm(frame_state_vector - current_state_vector)
                    / norm(frame_state_vector)) ** 2
            )

        return self._get_best_frame(eval_func)

    def spirit(self):
        def eval_func(pose, frame):
            coords, orientation = get_pose_components(pose)
            current_state_vector = np.hstack((self.current_frame.coords_precise,
                                              self.current_frame.orientation))
            frame_state_vector = np.hstack((frame.coords_precise,
                                            frame.orientation))

            dx, dy, dz = coords - frame.coords_precise
            beta = angle_between_quaternions(orientation, frame.orientation)
            a_mag = norm(coords - frame.coords_precise)

            centrality = (np.arctan2(np.sqrt(dx**2 + dz**2), dy) / (np.pi / 2)) ** 2
            # centrality = ((dx / dy)**2 + (dz / dy)**2)
            direction = (beta / (np.pi / 2)) ** 2
            distance = ((a_mag - self.l_ref) / self.l_ref) ** 2
            similarity = (norm(frame_state_vector - current_state_vector)
                          / norm(frame_state_vector)) ** 2

            # Dropping criteria
            # if a_mag < self.l_ref:
            #    return float("inf")
            # if centrality > np.pi / 3:
            #     print("dropping {}".format(np.rad2deg(centrality)))
            #     return float("inf")

            return (self.coeff_centred * centrality
                    + self.coeff_direction * direction
                    + self.coeff_distance * distance
                    + self.coeff_similarity * similarity
            )

        return self._get_best_frame(eval_func)

    def _get_best_frame(self, eval_func):
        if self.frames:
            if self.current_frame is None:
                return self.frames[0]

            results = {}
            for frame in reversed(self.frames):
                results[frame] = eval_func(self.pose, frame)
            return min(results, key=results.get)

    def __getattr__(self, name):
        """
        Return undefined attributes.

        Upon first access, the value of the parameter is obtained from the
        parent class and stored as an attribute, from where it is read on
        subsequent accesses.

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
        if name in self._parent._method_params:
            self.__setattr__(name, self._parent.__getattr__(name))
            return self.__getattribute__(name)
        else:
            return self._parent.__getattribute__(name)


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
    ntree : Octree
        The octree holding all frames according to their position in 3D.
    frames : list of Frame
        A chronological list of frames.
    evaluate : callable
        The evaluation function to use.
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

        self.ntree = Octree((0, 0, 0), 1000)  # 100 m per side
        self.frames = []  # Chronological

        method = rospy.get_param("~eval_method")
        self.evaluate = Evaluators(method, parent=self).evaluate

        with open(os.path.join(rospkg.RosPack().get_path("spirit"),
                               "config", "launch_params.yaml")) as fin:
            params = yaml.load(fin)
        self._method_params = params["past_image"][method].keys()

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
        # TODO: Make sure we are not using old poses.
        rospy.logdebug("New image")
        self.image = image
        if self.can_make_frame:
            rospy.logdebug("Adding frames to octree and queue")
            frame = Frame(self.pose, self.image)
            # self.ntree.insert(frame)
            self.frames.append(frame)
            self.clear()

    def pose_callback(self, pose):
        """
        Update `pose`, and select the best past image.

        Parameters
        ----------
        pose : PoseStamped
            A pose message.

        """
        rospy.logdebug("New pose")
        self.pose = pose

        best_frame = self.evaluate()
        if best_frame is not None:
            self.current_frame = best_frame
            self.past_image_pub.publish(best_frame.image)
            self.past_pose_pub.publish(best_frame.pose)

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
        if name in self._method_params:
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
