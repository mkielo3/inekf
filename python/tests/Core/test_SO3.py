import numpy as np
from numpy.testing import assert_allclose
from scipy.linalg import expm, logm
from numpy.linalg import inv
from inekf import SO3
import pytest

def test_BaseConstructor1():
    mtx = np.eye(3)
    sigma = np.eye(5)
    A = np.ones(2)

    x = SO3[2](mtx, sigma, A)

    assert_allclose(mtx, x.mat)
    assert_allclose(sigma, x.cov)
    assert_allclose(A, x.aug)

def test_BaseConstructor2():
    mtx = np.eye(3)
    sigma = np.eye(5)
    A = np.ones(2)

    with pytest.raises(Exception):
        SO3["D"](mtx, sigma)

    SO3["D"](mtx, sigma, A)
    SO3["D"]()

def test_ThetaConstructor():
    x = SO3(np.pi/4, 0, 0)
    v = 1 / np.sqrt(2)
    r = np.array([[1, 0, 0],
                [0, v, -v],
                [0, v, v]])

    assert_allclose(r, x.mat)

def test_TangentConstructor():
    x = np.array([0,0,0,4,5])

    state = SO3["D"](x)

    assert_allclose(x[-2:], state.aug)
    assert_allclose(state.mat, np.eye(3))

def test_Addaug():
    x = SO3["D"]()

    x.addaug(2)
    assert x.aug[-1] == 2

    with pytest.raises(Exception):
        y = SO3()
        y.addaug(2)

def test_Inverse():
    x = SO3(np.pi/4, np.pi/4, np.pi/4)
    assert_allclose(inv(x.mat), x.inverse().mat)

def test_exp():
    x = np.arange(1,6)

    ours = SO3["D"].exp(x)
    theirs = expm( SO3["D"].wedge(x) )

    assert_allclose(theirs, ours.mat)
    assert_allclose(x[-2:], ours.aug)
    with pytest.raises(Exception):
        SO3[3].exp(x)

def test_log():
    xi = np.array([.1, .2, .3, 5, 6])
    x = SO3["D"](xi)

    assert_allclose(xi, x.log())

def test_wedge():
    x = np.arange(1,4)
    ours = SO3["D"].wedge(x)
    theirs =  np.array([[0, -3,  2],
                        [3,  0, -1],
                        [-2, 1,  0]])
    
    assert_allclose(theirs, ours)

    with pytest.raises(Exception):
        SO3[3].wedge(x)

def test_Adjoint():
    x = SO3[1](1,1,1)
    expected = np.eye(4)
    expected[0:3,0:3] = x.mat
    assert_allclose(expected, x.Ad())
    assert_allclose(expected, SO3[1].Adjoint(x))