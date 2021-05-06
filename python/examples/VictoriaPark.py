import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

np.set_printoptions(suppress=True)

from tree_extraction import extract_trees, data_association
from inekf import OdometryProcessDynamic, MeasureModel
from inekf import SE2, InEKF, ERROR

class GPSSensor(MeasureModel[SE2["D"]]):
    def __init__(self, std):
        super().__init__()
        self.M = np.eye(2)*std**2
        self.error = ERROR.LEFT

    def processZ(self, z, state):
        curr_cols = state.State.shape[0] - 2
        curr_dim  = 1 + 2*curr_cols

        H = np.zeros((2, curr_dim))
        H[:,1:3] = np.eye(2)
        self.H = H
        if z.shape[0] == 2:
            z_ = np.zeros(state.State.shape[0])
            z_[0:2] = z
            z_[2] = 1
            return z_
        else:
            return z

class LandmarkSensor(MeasureModel[SE2["D"]]):
    def __init__(self, std_r, std_b):
        super().__init__()
        self.M_rb = np.diag([std_r**2, std_b**2])
        self.error = ERROR.RIGHT
        self.b = np.zeros(3)

    def sawLandmark(self, idx, state):
        self.b = np.zeros(state.State.shape[0])
        self.b[2] = 1
        self.b[idx+3] = -1

        curr_cols = state.State.shape[0] - 2
        curr_dim  = 1 + 2*curr_cols
        H = np.zeros((2, curr_dim))
        H[:,1:3] = np.eye(2)
        H[:,3+2*idx:5+2*idx] = -np.eye(2)
        self.H = H

    def processZ(self, z, state):
        # convert range and bearing into x and y
        if z.shape[0] == 2:
            r, b = z
            z_ = self.b.copy()
            z_[0] = r*np.cos(b)
            z_[1] = r*np.sin(b)

            # convert r/b cov -> x/y cov
            G = np.array([[np.cos(b), -r*np.sin(b)],
                            [np.sin(b), r*np.cos(b)]])

            self.M = G @ self.M_rb @ G.T

            return z_

def addLandmark(z, state):
    x, y = state[0]
    phi = state.log()[0]
    r, b = z

    xl = x + r*np.cos(b+phi)
    yl = y + r*np.sin(b+phi)

    # TODO Verify adding this to sigmas
    state.addCol([xl, yl], np.eye(2)*1000)

def x(l):
    return np.array([i[0][0] for i in l])

def y(l):
    return np.array([i[0][1] for i in l])

def wrapAngle(theta):
    while theta >= np.pi:
        theta -= 2*np.pi
    while theta <= np.pi:
        theta += np.pi
    return theta

def makeOdometry(u, dt, state):
    x, y = state[0]
    phi = state.log()[0]

    a = 3.78
    b = 0.50
    L = 2.83
    H = 0.76

    Ve, alpha = u
    alpha = wrapAngle(alpha)
    Vc = Ve / (1 - np.tan(alpha)*H/L)

    # TODO Double check these
    motion = np.zeros(3)
    motion[0] = Vc/L*np.tan(alpha)
    motion[1] = Vc - Vc/L*np.tan(alpha)*b
    motion[2] = Vc/L*np.tan(alpha)*a

    return SE2(dt*motion)

# setup initial state
x0 = [0,0,0]
sig = np.diag([.1,.1,.1])
x0 = SE2["D"](x0, sig)

#### SETUP InEKF
gps = GPSSensor(3)
laser = LandmarkSensor(0.5, 0.5*np.pi/180)

iekf = InEKF[OdometryProcessDynamic](x0, ERROR.RIGHT)
iekf.addMeasureModel("GPS", gps)
iekf.addMeasureModel("Laser", laser)
iekf.pModel.setQ([0.5*np.pi/180, 0.05, 0.05])

#### LOAD IN DATA
def read_data(filename):
    with open(filename, 'r') as f:
        data = [[float(x) for x in line.strip().split('\t')] for line in f.readlines()]
    return np.array(data)

odo_data = read_data("../../data/victoria_park_ascii/DRS.txt")
gps_data = read_data("../../data/victoria_park_ascii/GPS.txt")
laser_data = read_data("../../data/victoria_park_ascii/LASER.txt")
laser_data = laser_data.reshape(-1, 362)

events =      [('gps',   x[0], x[1:]) for x in gps_data]
events.extend([('odo',   x[0], x[1:]) for x in odo_data])
events.extend([('laser', x[0], x[1:]) for x in laser_data])
events = sorted(events, key=lambda x: x[1])

#### GET PLOT READY
fig = plt.figure()

states = [x0]

last_t = 0
for i, (e, t, data) in (enumerate(events[:8000])):
    # If it's Odometry
    print(e)
    if e == 'odo':
        dt = t - last_t
        last_t = t

        u = makeOdometry(data, dt, iekf.state)
        s = iekf.Predict(u, dt)
        states.append(s)

    # GPS Measurement
    if e == 'gps':
        s = iekf.Update(data, "GPS")
        states[-1] = s

    # Laser Measurement
    if e == 'laser':
        # identify landmarks
        trees = extract_trees(data)
        assoc = data_association(iekf.state, trees, laser)
        # iterate through them (note data here is still r/b)
        for idx, data in zip(assoc, trees):
            if idx == -1:
                addLandmark(data, iekf.state)
            elif idx == -2:
                continue
            else:
                laser.sawLandmark(idx, iekf.state)
                iekf.Update(data, "Laser")


states = np.array(states)

# plot
plt.plot(x(states), y(states))
plt.scatter(gps_data[:,1], gps_data[:,2])
plt.show()