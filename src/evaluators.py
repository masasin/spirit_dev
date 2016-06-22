#!/usr/bin/env python
# -*- coding: utf-8 -*-
# (C) 2016  Jean Nassar
# Released under BSD version 4
import abc
import sys

import numpy as np
import six

from helpers import fov_diagonal2vertical, d2r


def get_evaluator(method, parent):
    return getattr(sys.modules[__name__], method)(parent)


@six.add_metaclass(abc.ABCMeta)
class Evaluator(object):
    def __init__(self, parent):
        self._parent = parent

    @abc.abstractmethod
    def _evaluate_frame(self, pose, frame):
        return

    def evaluate(self):
        if self.frames:
            if self.current_frame is None:
                return self.frames[0]

            results = {}
            for frame in reversed(list(self.frames)):
                results[frame] = self._evaluate_frame(self.pose, frame)
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


class CoeffedEvaluator(Evaluator):
    def _evaluate_frame(self, pose, frame):
        return sum(coeff * self.__getattribute__(component)(pose, frame)
                   for component, coeff in self.eval_coeffs.items())


class ConstantTimeDelay(Evaluator):
    def _evaluate_frame(self, pose, frame):
        optimum_timestamp = pose.header.stamp.to_sec() - self.ref_delay
        return abs(frame.stamp.to_sec() - optimum_timestamp)


class ConstantDistance(Evaluator):
    def _evaluate_frame(self, pose, frame):
        return abs(frame.distance(pose) - self.ref_distance)


class Spirit(CoeffedEvaluator):
    @staticmethod
    def centrality(pose, frame):
        dx, dy, dz = frame.dpos(pose)
        return min((dx**2 + dz**2) / dy**2, 0.2) if dy < 0 else 0.2

    @staticmethod
    def direction(pose, frame):
        return frame.deuler(pose)[2]**2

    def distance(self, pose, frame):
        return ((frame.distance(pose) - self.ref_distance)
                / self.ref_distance)**2


class Konishi(CoeffedEvaluator):
    pass


class Murata(CoeffedEvaluator):
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
    def height(self, pose, frame):
        self._dxg, self._dyg, self.dzg = pose.position - frame.position
        return ((self._dzg - self.ref_height) / self.ref_height)**2

    @staticmethod
    def direction(pose, frame):
        beta = frame.deuler(pose)[2]
        return beta**2

    def elevation(self, pose, frame):
        alpha = np.arctan2(self._dzg, self._dyg)
        fov_y = d2r(fov_diagonal2vertical(92))
        return (alpha / fov_y)**2

    def distance(self, pose, frame):
        return ((frame.distance(pose) - self.ref_distance)
                / self.ref_distance)**2