# -*- coding: utf-8 -*-
"""Applies a Kalman filter and returns current state x,y,theta as a 3*1 np.array
Inputs:
    - state = np.array([x_k-1,y_k-1,theta_k-1]) is the state at step k-1. theta is in rad.
    - Sigma is the 3*3 np.array which corresponds to the covariance matrix of the states
    - motorspeed = [speedR, speedL] is the motor.target which is given to Thymio at step k (units unknown)
    - history = [state_(k-N), state_(k-(N-1)), ..., state(k-1)] where N is the horizon chosen for keeping the history
    - camera = [(x,y), theta, is_working] is the position of thymio given by vision. 
    is_working is a boolean that indicates whether the camera faced an issue.
    - Ts is the sampling time in seconds

Outputs:
    updated versions of state, Sigma, history at step k
"""
import numpy as np
import math

def kalmanfilter(state,Sigma,motorspeed,history, camera, Ts):
    #Main function to be used
    
    #convert thymio motorspeed to linear speed (calibration)
    motorspeed = calibrate_motorspeed(motorspeed) 
    
    #predict next step
    state, Sigma, history = predict(state, Sigma, motorspeed, history, Ts)
    
    if camera[2] == True:
        #if the vision is available, proceed to update
        state, Sigma = update(state, Sigma, motorspeed, history, camera)
    
    return state, Sigma, history


def predict(state, Sigma, motorspeed, history, Ts):
    #equations:
        #x_k = Ax_(k-1) + f(Ts, theta_k)*[vright, vleft]
        #y_k = Cx_k
    #note: f is linearized to obtain B
    
    #parameters
    std_wheelspeed = 0.01   #std dev. for actual vs. expected linear speed of wheels
    L = 0.095               #axle length
    
    #initialize Q (noise due to wheels) and A
    Q = np.random.normal(0.0, std_wheelspeed, (2,1))
    A = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])    
    
    #obtain B
    theta_new = state[2] + Ts*(motorspeed[0]-motorspeed[1])/(2*L) #only used to calculate B
    B = 0.5*np.array([[math.cos(theta_new),math.cos(theta_new)],\
                      [math.sin(theta_new),math.sin(theta_new)],\
                          [1/L, -1/L]])
    
    #update history and state (note: @ is the matrix multiplication)
    history = np.roll(history, -1, 0)
    history[-1] = np.reshape(state,(1,3))
    state = A @ state + Ts*B @ motorspeed
    
    #update Sigma, matrix of covariance
    process_noise = np.diag([(B @ Q) [0][0], (B@Q) [1][0], (B@Q) [2][0]])
    Sigma = (A.T @ Sigma @ A.T) + process_noise
    
    return state,Sigma,history


def update(state, Sigma, motorspeed, history, camera):
    #parameters: 
        #thresholds to discard wrong values from camera
        #measurement noise
    threshold_distance = 10
    threshold_theta = 10
    std_measure_coords = 0.01
    std_measure_theta = 0.01
    
    #initialize matrices
    avg_history = np.mean(history,0)
    C = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]) #y_k = C*x_k
    R = np.diag([np.random.normal(0.0, std_measure_coords), \
                 np.random.normal(0.0, std_measure_coords), \
                 np.random.normal(0.0, std_measure_theta)]) #measure noise
    
    #calculate updated state
    y = np.array([[camera[0][0]], [camera[0][1]], [camera[1]]])  #measured states
    innov = y - (C @ state)
    s = (C @ Sigma @ C.T) + R
    K = Sigma @ C.T @ np.linalg.inv(s) #optimal gain
    
    state_update = state + K @ innov
    
    #if the measured value is obviously wrong, then do NOT proceed to update
    if np.linalg.norm(state_update[0:1] - avg_history[0:1]) < threshold_distance\
            and abs(state_update[2] - avg_history[0][2]) < threshold_theta:
                state = state_update
                Sigma = (np.eye(3) - K@C) @ Sigma
    
    return state, Sigma


def calibrate_motorspeed(motorspeed):
    #convert motor speed to linear speed (m/s)
    
    #need to implement a function for calibration (interpolation between known points)
    
    
    motorspeed = np.array([[motorspeed[0]],[motorspeed[1]]]) #convert to 2-by-1 np.array
    return motorspeed







#Test: movement in straight line at 45°, camera measures x=y=theta=0 always
state0 = np.array([[0],[0],[45*math.pi/180]]) #3-by-1 np.array
Sigma0 = np.diag([0.01,0.01,0.01])

N = 15 #size of history
history = [np.reshape(state0,(1,3))]*N

motorspeed = [1,1] #command sent to Thymio
camera = [(0,0),0,True] #live output of the camera
Ts = 0.1 #sampling time

#test it
state, Sigma, history = kalmanfilter(state0,Sigma0,motorspeed,history, camera, Ts)
iter = 1
print("Iteration {}".format(iter))
print("State {}".format(state))
print("Sigma {}".format(Sigma))
#print("History {}".format(history))
print("\n \n")
for k in range(5):
    state, Sigma, history = kalmanfilter(state,Sigma,motorspeed,history, camera, Ts)
    iter += 1
    print("Iteration {}".format(iter))
    print("State {}".format(state))
    print("Sigma {}".format(Sigma))
    #print("History {}".format(history))
    print("\n \n")