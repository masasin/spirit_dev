from __future__ import division

import numpy as np

from helpers import unit_vector


class TestUnitVector(object):
    def test_regular_unit_vector(self):
        assert np.isclose(unit_vector([1, 0, -1]),
                          np.array([np.sqrt(0.5), 0, -np.sqrt(0.5)])).all()

    def test_zero_vector(self):
        assert (unit_vector([0, 0, 0]) == np.zeros(3)).all()
