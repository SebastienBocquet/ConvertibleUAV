# -*- coding: utf-8 -*-
"""
Python Flight Mechanics Engine (PyFME).
Copyright (c) AeroPython Development Team.
Distributed under the terms of the MIT License.

Frames of Reference orientation test functions
----------------------------------------------
"""
import pytest
from pytest import approx
import numpy as np
from numpy.testing import (assert_array_almost_equal, assert_almost_equal)

from uav_model.wing_aero import (TrapezoidalWingAero)

ABS_TOL = 1e-9


@pytest.mark.xfail
def test_bad_geometry():
    w = TrapezoidalWingAero(0.2, 0.3, 1.4, 0.)


def test_geometrical_characteristics():
    rectangular_wing = TrapezoidalWingAero(0.2, 0.2, 1.4, 0.)

    assert rectangular_wing.surface == pytest.approx(0.2 * 1.4, abs=ABS_TOL)
    assert rectangular_wing.x_ac_le == pytest.approx(0.25 * 0.2, abs=ABS_TOL)
    assert rectangular_wing.mean_chord == pytest.approx(0.2, abs=ABS_TOL)
    assert rectangular_wing.mean_sweep == pytest.approx(0., abs=ABS_TOL)
    assert rectangular_wing.eff == pytest.approx(1., abs=ABS_TOL)
    assert rectangular_wing._lambda == pytest.approx(1.4 / 0.2, abs=ABS_TOL)

