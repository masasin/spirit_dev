# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
"""
Evaluators for solving SPIRIT.

"""
import sys

import numpy as np

from helpers import Pose, Frame


def get_evaluator(method, parent):
    """
    Get an evaluator.

    Parameters
    ----------
    method : str
        The name of the evaluator.
    parent : Selector
        The selector using the evaluator.

    Returns
    -------
    Evaluator
        The initialized evaluator.

    """
    return getattr(sys.modules[__name__], method)(parent)


class Evaluator(object):
    """
    Base class for evaluators.

    The methods used and their coefficients can be set in the configuration file
    by setting the ``coeff_{method_name}`` for the appropriate evaluator.

    Parameters
    ----------
    parent : Selector
        The selector using the evaluator.

    """
    def __init__(self, parent):
        self._parent = parent
        self._vars_frame = {}

    def _evaluate_frame(self, pose, frame):
        """
        Evaluate the score for a pose against a frame.

        Parameters
        ----------
        pose : Pose
            The pose to be evaluated.
        frame : Frame
            The frame against which the pose is evaluated.

        Returns
        -------
        float
            The score for the pose against the frame.

        """
        frame_score = sum(coeff * self.__getattribute__(component)(pose, frame)
                          for component, coeff in self.eval_coeffs.items())
        self._vars_frame = {}
        return frame_score

    def select_best_frame(self):
        """
        Select the best frame using the minimum of all individual frame scores.

        Returns
        -------
        Frame
            The best frame.

        """
        if self.frames:
            if self.current_frame is None:
                return self.frames[0]

            results = {}
            try:
                for frame in self.frames:
                    results[frame] = self._evaluate_frame(self.pose, frame)
            except RuntimeError:
                # A new frame was added. Cancel the calculation and keep the
                # displayed frame the same.
                return self.current_frame
            else:
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
        if name in self._parent.eval_method_params:
            self.__setattr__(name, self._parent.__getattr__(name))
            return self.__getattribute__(name)
        else:
            return self._parent.__getattribute__(name)


class ConstantTimeDelay(Evaluator):
    """
    Use an evaluator which keeps a constant time delay.

    """
    def time(self, pose, frame):
        """
        Get the time difference between pose and frame.

        Parameters
        ----------
        pose : Pose
            The pose to be evaluated.
        frame : Frame
            The frame against which the pose is evaluated.

        Returns
        -------
        float
            The difference between the optimum and frame timestamps.

        """
        optimum_timestamp = pose.header.stamp.to_sec() - self.ref_delay
        return abs(frame.stamp.to_sec() - optimum_timestamp)


class ConstantDistance(Evaluator):
    """
    Use an evaluator which maintains a constant absolute distance.

    """
    def distance(self, pose, frame):
        """
        Get the absolute distance between pose and frame.

        Parameters
        ----------
        pose : Pose
            The pose to be evaluated.
        frame : Frame
            The frame against which the pose is evaluated.

        Returns
        -------
        float
            The difference of the distance between frame and pose, and the
            reference distance.

        """
        return abs(frame.distance(pose) - self.ref_distance)


class Spirit(Evaluator):
    """
    Use the evaluator from SPIRIT.

    """
    @staticmethod
    def centrality(pose, frame):
        """
        Get how close to the centre of the frame the pose is.

        If the pose is behind the frame, or the output is too high, cap.

        Parameters
        ----------
        pose : Pose
            The pose to be evaluated.
        frame : Frame
            The frame against which the pose is evaluated.

        Returns
        -------
        float
            The centrality score.

        """
        dx, dy, dz = frame.rel_position(pose)
        return min((dx**2 + dz**2) / dy**2, 0.2) if dy < 0 else 0.2

    @staticmethod
    def direction(pose, frame):
        """
        Get how close the yaws of pose and frame are.

        Parameters
        ----------
        pose : Pose
            The pose to be evaluated.
        frame : Frame
            The frame against which the pose is evaluated.

        Returns
        -------
        float
            The direction score.

        """
        return frame.rel_euler(pose)[2] ** 2

    def distance(self, pose, frame):
        """
        Get the closeness to the reference distance.

        Parameters
        ----------
        pose : Pose
            The pose to be evaluated.
        frame : Frame
            The frame against which the pose is evaluated.

        Returns
        -------
        float
            The distance score.

        """
        return ((frame.distance(pose) - self.ref_distance)
                / self.ref_distance)**2

    # noinspection PyUnusedLocal
    def direction_with_current(self, pose, frame):
        """
        Get how close the yaws of pose and the currently displayed frame are.

        Parameters
        ----------
        pose : Pose
            The pose to be evaluated.
        frame : Frame
            (Unused) The frame against which the pose is evaluated.

        Returns
        -------
        float
            The similar direction score.

        """
        return self.direction(pose, self.current_frame)

    # noinspection PyUnusedLocal
    def distance_with_current(self, pose, frame):
        """
        Get the closeness to the currently displayed frame.

        Parameters
        ----------
        pose : Pose
            The pose to be evaluated.
        frame : Frame
            (Unused) The frame against which the pose is evaluated.

        Returns
        -------
        float
            The similar distance score.

        """
        return self.distance(pose, self.current_frame)


class Murata(Evaluator):
    """
    Use the evaluator from Murata's thesis. [#]_

    Extended Notes
    --------------
    The evaluation function is:

    .. math::
        E = a1 ((z_{camera} - z_{ref})/z_{ref})^2 +
            a2 β_{xy}^2 +
            a3 α^2 +
            a4 ((|\mathbf{a}| - l_ref)/l_ref)^2

    where :math:`a1` through :math:`a4` are coefficients, :math:`z` is the
    difference in height of the drone, :math:`α` is the tilt angle,
    :math:`β` is the difference in yaw, :math:`\mathbf{a}` is the distance
    vector, and :math:`l_ref` is the reference distance.

    References
    ----------
    .. [#] Ryosuke Murata, Undergrad Thesis, Kyoto University, 2013
           過去画像履歴を用いた移動マニピュレータの遠隔操作システム

    """
    def _evaluate_frame(self, pose, frame):
        self._frame_vars.update(
            {var: value for var, value in zip(("dxg", "dyg", "dzg"),
                                              pose.position - frame.position)}
        )
        super(Murata, self)._evaluate_frame(pose, frame)

    # noinspection PyUnusedLocal
    def height(self, pose, frame):
        """
        Get the closeness to a reference height.

        Parameters
        ----------
        pose : Pose
            (Unused) The pose to be evaluated.
        frame : Frame
            (Unused) The frame against which the pose is evaluated.

        Returns
        -------
        float
            The height score.

        """
        return ((self._frame_vars["dzg"] - self.ref_height)
                / self.ref_height)**2

    @staticmethod
    def direction(pose, frame):
        """
        Get how close the yaws of pose and frame are.

        Parameters
        ----------
        pose : Pose
            The pose to be evaluated.
        frame : Frame
            The frame against which the pose is evaluated.

        Returns
        -------
        float
            The direction score.

        """
        return frame.rel_euler(pose)[2] ** 2

    # noinspection PyUnusedLocal
    def elevation(self, pose, frame):
        """
        Get the closeness to the reference elevation.

        Parameters
        ----------
        pose : Pose
            (Unused) The pose to be evaluated.
        frame : Frame
            (Unused) The frame against which the pose is evaluated.

        Returns
        -------
        float
            The elevation score.

        """
        return np.arctan2(self._frame_vars["dzg"], self._frame_vars["dyg"]) ** 2

    def distance(self, pose, frame):
        """
        Get the closeness to the reference distance.

        Parameters
        ----------
        pose : Pose
            The pose to be evaluated.
        frame : Frame
            The frame against which the pose is evaluated.

        Returns
        -------
        float
            The distance score.

        """
        return ((frame.distance(pose) - self.ref_distance)
                / self.ref_distance)**2
