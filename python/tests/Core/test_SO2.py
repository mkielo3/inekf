import numpy as np
from numpy.testing import assert_allclose
from scipy.linalg import expm, logm
from numpy.linalg import inv
from inekf import SO2
import pytest

def test_BaseConstructor1():
    mtx = np.eye(2)
    sigma = np.eye(3)
    A = np.ones(2)

    x = SO2[2](mtx, sigma, A)

    assert_allclose(mtx, x.mat)
    assert_allclose(sigma, x.cov)
    assert_allclose(A, x.aug)

def test_BaseConstructor2():
    mtx = np.eye(2)
    sigma = np.eye(3)
    A = np.ones(2)

    with pytest.raises(Exception):
        SO2["D"](mtx, sigma)

    SO2["D"](mtx, sigma, A)
    SO2["D"]()

def test_ThetaConstructor():
    x = SO2(np.pi/4)
    v = 1 / np.sqrt(2)
    r = np.array([[v, -v],[v, v]])

    assert_allclose(r, x.mat)

def test_TangentConstructor():
    x = np.arange(3)

    state = SO2["D"](x)

    assert_allclose(x[1:], state.aug)
    assert_allclose(state.mat, np.eye(2))

def test_Addaug():
    x = SO2["D"]()

    x.addAug(2)
    assert x.aug[-1] == 2

    with pytest.raises(Exception):
        y = SO2()
        y.addAug(2)

def test_Inverse():
    x = SO2()
    assert_allclose(inv(x.mat), x.inverse.mat)

def test_exp():
    x = np.arange(1,4)

    ours = SO2["D"].exp(x)
    theirs = expm( SO2["D"].wedge(x) )

    assert_allclose(theirs, ours.mat)
    assert_allclose(x[-2:], ours.aug)
    with pytest.raises(Exception):
        SO2[3].exp(x)

def test_log():
    xi = np.arange(3)
    x = SO2["D"](xi)

    assert_allclose(xi, x.log)

def test_wedge():
    x = np.arange(1,4)
    ours = SO2["D"].wedge(x)
    theirs =  np.array([[0, -1],
                        [1, 0]])
    
    assert_allclose(theirs, ours)

    with pytest.raises(Exception):
        SO2[3].wedge(x)

def test_Adjoint():
    assert_allclose(np.ones(1), SO2().Ad)
    assert_allclose(np.eye(3), SO2[2]().Ad)