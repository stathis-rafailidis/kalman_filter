"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math

import pandas as pd

# Estimation parameter of EKF
<<<<<<< HEAD
Q = np.diag([0.1, 0.1])**2  # Observation x,y position covariance
R = np.diag([0.1, 0.1, np.deg2rad(0), 0])**2  # predict state covariance

#  Simulation parameter
Qsim = np.diag([0.1, 0.1])**2
Rsim = np.diag([0.1, np.deg2rad(0)])**2
=======
Q = np.diag([1, 1])**2  # Observation x,y position covariance
R = np.diag([1, 1, np.deg2rad(10.0), 10.0])**2  # predict state covariance

#  Simulation parameter
# Qsim = np.diag([0.5, 0.5])**2
# Rsim = np.diag([1.0, np.deg2rad(30.0)])**2
>>>>>>> 3f20edd4df78bb3dd8796401583a68f0c2bf2b76

DT = 0.1  # time tick [s]
SIM_TIME = 114  # simulation time [s]

show_animation = False


def calc_input():
    v = 2.0  # [m/s]
    yawrate = 0.0  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u

def motion_model(x, u):

    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F.dot(x) + B.dot(u)

    return x


def observation_model(x):
    #  Observation Model
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H.dot(x)

    return z


def jacobF(x, u):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacobH(x):
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(xEst, PEst, z, u):

    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacobF(xPred, u)
    PPred = jF.dot(PEst).dot(jF.T) + R

    #  Update
    jH = jacobH(xPred)
    zPred = observation_model(xPred)
    y = z.T - zPred
    S = jH.dot(PPred).dot(jH.T) + Q
    K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    xEst = xPred + K.dot(y)
    PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

    return xEst, PEst

def main():
    print(__file__ + " start!!")

    time = 0.0

    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1))
    PEst = np.eye(4)

    #My Code#
<<<<<<< HEAD

    #File_data = pd.read_excel(r'C:\Users\stathis\OneDrive\Διπλωματική\data from kalman scipts\17_7_23\kalman_data_17_7_23.xlsx',  sheet_name="OdomCoords")

    File_data = pd.read_excel(r'C:\Users\stathis\OneDrive\Διπλωματική\data from sim.xlsx',  sheet_name="OdomCoords")

    #File_data = np.loadtxt("Desktop/OdomCoords.txt", delimiter= "," , dtype=float)

=======
    File_data = pd.read_excel(r'C:\Users\stath\OneDrive\Διπλωματική\data from kalman scipts\17_7_23\kalman_data_17_7_23.xlsx',  sheet_name="OdomCoords")
>>>>>>> 3f20edd4df78bb3dd8796401583a68f0c2bf2b76
    File_data = np.asarray(File_data)

    temp = np.array([[File_data[0][0]], [File_data[0][1]]])
    temp2 = temp.transpose()
    i = 0

    # history
    # hxEst = xEst
    # hxTrue = xTrue
    # hxDR = xTrue
    # hz = np.zeros((1, 2))

    while SIM_TIME >= time:
        time += DT
        u = calc_input()
        xEst, PEst = ekf_estimation(xEst, PEst, temp2, u)
        i = i + 1
        temp = np.array([[File_data[i][0]], [File_data[i][1]]])
        temp2 = temp.transpose()
        f = open("xEst.txt", "a")
        f.write(str(xEst[0]).strip('] [') + " " + str(xEst[1]).strip('] [') + "\n")
        f.close()

if __name__ == '__main__':
    main()