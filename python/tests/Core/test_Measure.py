import numpy as np
from numpy.testing import assert_allclose
from inekf import SE2, SO3, MeasureModel, ERROR
import pytest

Group = SE2[2,1]


def test_bConstructor():
    b = np.array([0,0,0,1])
    M = np.eye(2) 
    H = np.zeros((2,6))
    H[0:2,3:5] = np.eye(2)

    l = MeasureModel[Group](b, M, ERROR.LEFT)
    assert_allclose(H, l.H)

    r = MeasureModel[Group](b, M, ERROR.RIGHT)
    assert_allclose(-H, r.H)

    b[0] = 1
    b[1] = 2
    H[0,0] = -2;
    H[1,0] = 1
    l = MeasureModel[Group](b, M, ERROR.LEFT)
    assert_allclose(H, l.H)

    b = np.array([1,2,3])
    H = -SO3.Wedge(b)
    l = MeasureModel[SO3](b, np.eye(3), ERROR.LEFT)
    assert_allclose(H, l.H)

def test_processZ():
    b = np.array([0,0,0,1])
    M = np.eye(2) 
    state = Group()
    S = MeasureModel[Group](b, M, ERROR.LEFT)

    z = np.array([2,2])
    expected = np.array([2,2,0,1])

    assert_allclose(expected, S.processZ(z, state))