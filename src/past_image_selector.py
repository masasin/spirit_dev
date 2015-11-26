#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2015  Jean Nassar
# Released under BSD version 4
from __future__ import division
from time import localtime, strftime

import numpy as np
from numpy.linalg import norm

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from ntree import Octree


def get_pose_components(pose):
    coords = np.array([pose.pose.position.x,
                       pose.pose.position.y,
                       pose.pose.position.z])

    orientation = np.array([pose.pose.orientation.x,
                            pose.pose.orientation.y,
                            pose.pose.orientation.z,
                            pose.pose.orientation.w])

    return coords, orientation


class Frame(object):
    def __init__(self, pose, image):
        self.pose = pose
        self._coords_precise, self._orientation = get_pose_components(self.pose)
        self.coordinates = self._coords_precise // 10 * 100  # Round to 10 cm
        self.image = image
        self.stamp = self.pose.header.stamp
        self.stamp_str = strftime("%Y-%m-%d %H:%M:%S",
                                  localtime(self.stamp.to_time()))

    def distance_to(self, other):
        return norm(self._coords_precise - other._coords_precise)

    def __repr__(self):
        return "Frame ({}): {}".format(self._coords_precise.tolist(),
                                       self.stamp)


class Evaluator(object):
    def __init__(self, method, parent=None):
        self.parent = parent
        self.evaluate = self.__getattribute__(method)

    def constant_time_delay(self, pose):
        if len(self.parent.frames):
            optimum_timestamp = pose.header.stamp.to_sec() - self.parent._delay

            for frame in reversed(self.parent.frames):
                if frame.stamp.to_sec() < optimum_timestamp:
                    return frame
            return self.parent.frames[-1]

    def constant_distance(self, pose, distance=None):
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
        def eval_func(pose, frame):
            pass

        position, orientation = get_pose_components(pose)
        nearest_ten = self.parent.octree.get_nearest(position, k=10)
        results = {}
        for location in nearest_ten:
            for frame in location:
                results[frame] = eval_func(frame)
        return min(results, key=results.get)


class Selector(object):
    def __init__(self):
        self.clear()
        self.octree = Octree((0, 0, 0), 100000)  # 100 m
        self.frames = []  # Chronological

        method = rospy.get_param("~eval_method")
        self.evaluate = Evaluator(method, self).evaluate

        rospy.Subscriber("/ardrone/slow_image_raw", Image, self.image_callback)
        rospy.Subscriber("/ardrone/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/ardrone/tracked", Bool, self.tracked_callback)

        self.past_image_pub = rospy.Publisher("/ardrone/past_image", Image,
                                              queue_size=1)
        self.tf_pub = tf2_ros.TransformBroadcaster()

    def image_callback(self, image):
        rospy.logdebug("New image")
        self.image = image
        if self.is_ready:
            rospy.logdebug("Adding frames to octree and queue")
            frame = Frame(self.pose, self.image)
            self.octree.insert(frame)
            self.frames.append(frame)
            self.clear()

    def pose_callback(self, pose):
        rospy.logdebug("New pose")
        self.pose = pose
        best_frame = self.evaluate(pose)
        if best_frame is not None:
            self.past_image_pub.publish(best_frame.image)
        self.update_tf(pose)

    def tracked_callback(self, tracked):
        self.tracked = tracked.data

    def update_tf(self, pose):
        position, orientation = get_pose_components(pose)

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "ardrone/body"

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]

        self.tf_pub.sendTransform(t)

    @property
    def is_ready(self):
        return self.image and self.pose and self.tracked

    def clear(self):
        self.image = None
        self.pose = None
        self.tracked = None

    def __getattr__(self, name):
        if name == "_delay":
            self._delay = rospy.get_param("~delay")
            return self._delay
        if name == "_distance":
            self._distance = rospy.get_param("~distance")
            return self._distance


def main():
    """Initialize ROS node."""
    rospy.init_node("past_image_selector")
    Selector()
    rospy.loginfo("Started the past image selector")
    rospy.spin()


if __name__ == "__main__":
    main()
