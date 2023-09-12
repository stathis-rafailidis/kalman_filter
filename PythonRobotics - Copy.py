"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""

import numpy as np
import math
import matplotlib.pyplot as plt

import pandas as pd

# Estimation parameter of EKF
Q = np.diag([0.1, 0.1])**2  # Observation x,y position covariance
R = np.diag([0.1, 0.1, np.deg2rad(0), 0])**2  # predict state covariance

#  Simulation parameter
Qsim = np.diag([0.1, 0.1])**2
Rsim = np.diag([0.1, np.deg2rad(0)])**2

DT = 0.1  # time tick [s]
SIM_TIME = 500.0  # simulation time [s]

show_animation = False


def calc_input():
    v = 2.0  # [m/s]
    yawrate = 0.0  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u


def observation(xTrue, xd, u):

    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    zx = xTrue[0, 0] + np.random.randn() * Qsim[0, 0]
    zy = xTrue[1, 0] + np.random.randn() * Qsim[1, 1]
    z = np.array([[zx, zy]])

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


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


def plot_covariance_ellipse(xEst, PEst):
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    print(__file__ + " start!!")

    time = 0.0

    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1))
    xTrue = np.zeros((4, 1))
    PEst = np.eye(4)

    xDR = np.zeros((4, 1))  # Dead reckoning


    #My Code#

    #File_data = pd.read_excel(r'C:\Users\stathis\OneDrive\Διπλωματική\data from kalman scipts\17_7_23\kalman_data_17_7_23.xlsx',  sheet_name="OdomCoords")

    File_data = pd.read_excel(r'C:\Users\stathis\OneDrive\Διπλωματική\data from sim.xlsx',  sheet_name="OdomCoords")

    #File_data = np.loadtxt("Desktop/OdomCoords.txt", delimiter= "," , dtype=float)

    File_data = np.asarray(File_data)

    #arr = np.array([[1, 2, 3], [4, 5, 6]])
    temp = np.array([[File_data[0][0]], [File_data[0][1]]])
    temp2 = temp.transpose()
    i = 0
    #  #


    # history
    # hxEst = xEst
    # hxTrue = xTrue
    # hxDR = xTrue
    # hz = np.zeros((1, 2))

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)
        print(u)
        # f = open("xTrue.txt", "a")
        # f.write(str(xTrue[0]) + ", " + str(xTrue[1]) + "\n")
        # f.close()
        

        #xEst, PEst = ekf_estimation(xEst, PEst, temp2, u)
        # to kato einai to arxiko
        xEst, PEst = ekf_estimation(xEst, PEst, temp2, u)
        i = i + 1
        temp = np.array([[File_data[i][0]], [File_data[i][1]]])
        temp2 = temp.transpose()
        f = open("xEst.txt", "a")
        f.write(str(xEst[0]).strip('] [') + " " + str(xEst[1]).strip('] [') + "\n")
        f.close()



        
        


if __name__ == '__main__':
    main()