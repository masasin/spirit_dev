#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
"""
Selects the best image for SPIRIT.

"""
from __future__ import division
from time import localtime, strftime

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
        self._coords_precise, self._orientation = get_pose_components(self.pose)
        self.coordinates = self._coords_precise // 10 * 100  # Round to 10 cm
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
        return norm(self._coords_precise - other._coords_precise)

    def __repr__(self):
        """
        Description of the frame.

        """
        return "Frame ({coords}): {time}".format(
            coords=self._coords_precise.tolist(),
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
    def __init__(self, method, parent=None):
        self.parent = parent
        self.evaluate = self.__getattribute__(method)

    def constant_time_delay(self, pose):
        """
        Return the frame delayed by a fixed amount.

        If the delay has not yet passed, return the first frame.

        """
        if len(self.parent.frames):
            optimum_timestamp = pose.header.stamp.to_sec() - self.parent._delay

            for frame in reversed(self.parent.frames):
                if frame.stamp.to_sec() < optimum_timestamp:
                    return frame
            return self.parent.frames[-1]

    def constant_distance(self, pose, distance=None):
        """
        Return the frame a fixed distance away.

        If the distance has not yet been crossed, return the first frame.

        """
        if len(self.parent.frames):
            optimum_distance = error_current = error_min = self.parent._distance
            position, orientation = get_pose_components(pose)
            for frame in reversed(self.parent.frames):
                frame_distance = norm(frame._coords_precise - position)
                if frame_distance > optimum_distance:
                    error_current = abs(frame_distance - optimum_distance)
                    if (error_current < error_min):
                        return frame
            return self.parent.frames[-1]

    def test(self, pose):
        """
        Not implemented yet.

        """
        def eval_func(pose, frame):
            pass

        raise NotImplementedError

        position, orientation = get_pose_components(pose)
        nearest_ten = self.parent.octree.get_nearest(position, k=10)
        results = {}
        for location in nearest_ten:
            for frame in location:
                results[frame] = eval_func(frame)
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
        self.octree = Octree((0, 0, 0), 100000)  # 100 m
        self.frames = []  # Chronological

        method = rospy.get_param("~eval_method")
        self.evaluate = Evaluators(method, parent=self).evaluate

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
            self.octree.insert(frame)
            self.frames.append(frame)
            self.clear()

    def pose_callback(self, pose):
        """
        Update tf and `pose`, and select the best past image.

        """
        rospy.logdebug("New pose")
        self.pose = pose
        best_frame = self.evaluate(pose)
        if best_frame is not None:
            self.past_image_pub.publish(best_frame.image)
        self.tf_pub.sendTransform(tf_from_pose(pose, child="ardrone/body"))

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
