import numpy as np
from numpy.testing import assert_allclose
from scipy.linalg import expm, logm
from numpy.linalg import inv
from inekf import SE3, SO3
import pytest

def test_BaseConstructor1():
    state = np.eye(5)
    sigma = np.eye(11)
    aug = np.zeros(2)

    x = SE3[2,2](state, sigma, aug)

    assert_allclose(state, x.mat)
    assert_allclose(sigma, x.cov)
    assert_allclose(aug, x.aug)

def test_BaseConstructor2():
    state = np.eye(5)
    sigma = np.eye(10)
    aug = np.zeros(1)

    with pytest.raises(Exception):
        x = SE3["D"](state, sigma)

    x = SE3["D",1](state, sigma, aug)
    x = SE3[2,"D"](state, sigma, aug)

def test_TangentConstructor1():
    x = np.arange(10)

    state = SE3[2,1](x)

    assert_allclose(state.R.mat, SO3.exp(x[:3]).mat)
    assert_allclose(state[0], x[3:6])
    assert_allclose(state[1], x[6:9])
    assert state.aug[0] == x[-1]

def test_TangentConstructor2():
    x = np.arange(10)

    state = SE3["D", 1](x)
    assert_allclose(state.R.mat, SO3.exp(x[:3]).mat)
    assert_allclose(state[0], x[3:6])
    assert_allclose(state[1], x[6:9])
    assert state.aug[0] == x[-1]

    state = SE3[2,"D"](x)
    assert_allclose(state.R.mat, SO3.exp(x[:3]).mat)
    assert_allclose(state[0], x[3:6])
    assert_allclose(state[1], x[6:9])
    assert state.aug[0] == x[-1]

def test_PlainConstructor():
    x = SE3(0,0,0,4,5,6)
    assert_allclose(x.R.mat, np.eye(3))
    assert_allclose(x[0], [4,5,6])

def test_AddCol():
    x = SE3["D",0]()
    assert_allclose(x.mat, np.eye(4))

    x.addCol(np.ones(3))

    assert_allclose(x[1], np.ones(3))

def test_Addaug():
    x = SE3[1,"D"]()

    x.addAug(2)
    assert x.aug[-1] == 2

    with pytest.raises(Exception):
        y = SE3()
        y.addAug(2)

def test_Inverse():
    x = SE3()
    assert_allclose(inv(x.mat), x.inverse.mat)

def test_exp():
    x = np.arange(10)

    ours = SE3[2,1].exp(x)
    theirs = expm( SE3[2,1].wedge(x) )

    assert_allclose(theirs, ours.mat)
    assert x[-1] == ours.aug[0]
    with pytest.raises(Exception):
        SE3["D",2].exp(x)

def test_log():
    xi = np.array([.1, .2, .3, 4, 5, 6])
    x = SE3.exp(xi)

    assert_allclose(x.log, xi)

def test_wedge():
    x = np.arange(1,11)
    ours = SE3[2,1].wedge(x)
    theirs =  np.array([[0,  -3,  2,  4,  7],
                        [3,   0, -1,  5,  8],
                        [-2,  1,  0,  6,  9],
                        [0,   0,  0,  0,  0],
                        [0,   0,  0,  0,  0]])
    
    assert_allclose(theirs, ours)

    with pytest.raises(Exception):
        SE3["D",2].wedge(x)

def test_Adjoint():
    x = np.arange(1,11)
    x = SE3[2,1](x)
    ad = x.Ad

    for i in range(3):
        assert_allclose(ad[3*i:3*i+3, 3*i:3*i+3], x.R.mat)
    
    for i in range(2):
        top = SO3.wedge(x[i])@x.R.mat
        assert_allclose(ad[3+3*i:6+3*i,0:3], top)

    assert ad[9,9] == 1
