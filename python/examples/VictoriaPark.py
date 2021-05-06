import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

np.set_printoptions(suppress=True)

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
    def __init__(self, std):
        super().__init__()
        self.M = np.eye(2)*std**2
        self.error = ERROR.RIGHT

    def sawLandmark(self, idx, state):
        pass

    def processZ(self, z, state):
        pass

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
# TODO: Setup import from file later
x0 = [36*np.pi/180, -67.649, -41.714]
x0 = [0,0,0]
sig = np.diag([.1,.1,.1])
x0 = SE2["D"](x0, sig)


#### SETUP InEKF
gps = GPSSensor(np.sqrt(3))
# lidar = LandmarkSensor(2)

iekf = InEKF[OdometryProcessDynamic](x0, ERROR.RIGHT)
iekf.addMeasureModel("GPS", gps)
# iekf.addMeasureModel("Lidar", lidar)
iekf.pModel.setQ([0.5*np.pi/180, 0.05, 0.05])

#### LOAD IN DATA
def read_data(filename):
    with open(filename, 'r') as f:
        data = [[float(x) for x in line.strip().split('\t')] for line in f.readlines()]
    return np.array(data)

odo_data = read_data("../../data/victoria_park_ascii/DRS.txt")
gps_data = read_data("../../data/victoria_park_ascii/GPS.txt")
# laser = read_data("../../data/victoria_park_ascii/LASER.txt")

events =      [('gps', x[0], x[1:]) for x in gps_data]
events.extend([('odo', x[0], x[1:]) for x in odo_data])
# events.extend([('laser', x) for x in laser])

events = sorted(events, key=lambda x: x[1])

states = [x0]

last_t = 0
for i, (e, t, data) in tqdm(enumerate(events[:8000])):

    if e == 'odo':
        dt = t - last_t

        u = makeOdometry(data, dt, iekf.state)
        s = iekf.Predict(u, dt)

        states.append(s)
        last_t = t

    if e == 'gps':
        s = iekf.Update(data, "GPS")
        states[-1] = s

    if e == 'laser':
        pass

states = np.array(states)

# plot
plt.plot(x(states), y(states))
plt.scatter(gps_data[:,1], gps_data[:,2])
plt.show()