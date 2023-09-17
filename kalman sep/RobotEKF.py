from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import array, sqrt
import sympy
from sympy import symbols, Matrix
from math import sqrt, tan, cos, sin, atan2
import numpy as np
import pandas as pd



class RobotEKF(EKF):

    i = 0
    File_data = pd.read_excel(r'C:\Users\stathis\Desktop\kalman_data_17_7_23.xlsx',  sheet_name="Sheet2")
    File_data = np.asarray(File_data)
    #temp = np.array([[File_data[0][0]], [File_data[0][1]]])

    def __init__(self, dt, wheelbase, std_vel, std_steer):
        EKF.__init__(self, 3, 2, 2)
        self.dt = dt
        self.wheelbase = wheelbase
        self.std_vel = std_vel
        self.std_steer = std_steer

        a, x, y, v, w, theta, time = symbols(
            'a, x, y, v, w, theta, t')
        d = v*time
        beta = (d/w)*sympy.tan(a)
        r = w/sympy.tan(a)
    
        self.fxu = Matrix(
            [[x-r*sympy.sin(theta)+r*sympy.sin(theta+beta)],
             [y+r*sympy.cos(theta)-r*sympy.cos(theta+beta)],
             [theta+beta]])

        self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
        self.V_j = self.fxu.jacobian(Matrix([v, a]))

        # save dictionary and it's variables for later use
        self.subs = {x: 0, y: 0, v:0, a:0, 
                     time:dt, w:wheelbase, theta:0}
        self.x_x, self.x_y, = x, y 
        self.v, self.a, self.theta = v, a, theta

    def predict(self, u):
        #self.x = self.move(self.x, u, self.dt)        
        self.x = (self.move_2(self.i, self.File_data))

        self.subs[self.x_x] = self.x[0, 0]
        self.subs[self.x_y] = self.x[1, 0]

        self.subs[self.theta] = self.x[2, 0]
        self.subs[self.v] = u[0]
        self.subs[self.a] = u[1]

        F = array(self.F_j.evalf(subs=self.subs)).astype(float)
        V = array(self.V_j.evalf(subs=self.subs)).astype(float)

        # covariance of motion noise in control space
        M = array([[self.std_vel**2, 0], 
                   [0, self.std_steer**2]])

        self.P = F @ self.P @ F.T + V @ M @ V.T

    def move(self, x, u, dt):
        hdg = x[2, 0]
        # print(f"this is x: {x}")
        # print(f"this is hdg: {hdg}")
        vel = u[0]
        steering_angle = u[1]
        dist = vel * dt

        if abs(steering_angle) > 0.001: # is robot turning?
            beta = (dist / self.wheelbase) * tan(steering_angle)
            r = self.wheelbase / tan(steering_angle) # radius

            dx = np.array([[-r*sin(hdg) + r*sin(hdg + beta)], 
                           [r*cos(hdg) - r*cos(hdg + beta)], 
                           [beta]])
        else: # moving in straight line
            dx = np.array([[dist*cos(hdg)], 
                           [dist*sin(hdg)], 
                           [0]])
        #print(x.shape)
        # File_data = pd.read_excel(r'C:\Users\stathis\Desktop\kalman_data_17_7_23.xlsx',  sheet_name="Sheet2")
        # File_data = np.asarray(File_data)
        # temp = np.array([[File_data[0][0]], [File_data[0][1]]])
        #temp2 = temp.transpose()
        return x + dx
    
    def move_2(self, i, File_data):
        temp = np.array([[File_data[i][0]], [File_data[i][1]], [File_data[i][1]]] )
        return temp
