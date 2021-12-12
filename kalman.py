import numpy as np
import math
from scipy.interpolate import interp1d

STD_WHEELSPEED = 20         #std dev. for actual vs. expected linear speed of 
                            #wheels, in pixels
STD_MEASURE_COORDS = 20     #in pixels
STD_MEASURE_THETA = 0.01

THRESHOLD_DISTANCE = 80     #in pixels
THRESHOLD_THETA = 0.8

POS_CORRECTION_FACTOR = 3
ANGLE_CORRECTION_FACTOR = 3.6


DISTANCE = 2.51  

#%%
def kalmanfilter(state,Sigma,motorspeed,history,camera, Ts, meters_to_pixels,\
                 AXLE_LENGTH, errpos_history,errtheta_history, camera_history):
    """
    ---
    Description: Applies a Kalman filter on the position and the angle
    ---
    Inputs:
        - state = np.array([x_k-1,y_k-1,theta_k-1]) is the state at step k-1. 
                  theta is in rad.
        - Sigma: 3*3 np.array, covariance matrix of the states
        - motorspeed = [speedR, speedL] is the speed of motors as read by
                      Thymio's sensors
        - history = [state_(k-N), state_(k-(N-1)), ..., state(k-1)] 
                    where N is the horizon chosen for the history
        - camera = [(x,y), theta, is_working]:
                   position of thymio according to vision. 
                   is_working is a boolean: indicates whether Thymio was found
        - Ts: the sampling time in seconds
        - meters_to_pixels: factor to convert m/s --> pixels/s
        - AXLE_LENGTH: length of the axle that connects the wheels
        - errpos_history: history of errors between camera's measures
                          and predicted position
        - errtheta_history: history of errors between camera's measures
                            and predicted angle
        - camera_history: history of positions and angles read by the camera
    ---
    Outputs: updated versions of the inputs
    """
    #convert thymio motorspeed to linear speed (calibration)
    motorspeed = calibrate_motorspeed(motorspeed, meters_to_pixels)
    
    #predict next step
    state, Sigma, history = predict(state, Sigma, motorspeed, history, Ts, \
                                    AXLE_LENGTH)
    #put it in [-pi,pi]
    state[2][0] = (state[2][0] + math.pi)%(2*math.pi) - math.pi  
    
    if camera[2] == True:
        #if the vision is available, proceed to update
        state, Sigma, errpos_history, errtheta_history, camera_history\
            = update(state, Sigma, history, camera,\
                     errpos_history, errtheta_history, camera_history)
        #put it in [-pi,pi]
        state[2][0] = (state[2][0] + math.pi)%(2*math.pi) - math.pi  
    
    return state,Sigma,history,errpos_history,errtheta_history,camera_history

#%%
def predict(state, Sigma, motorspeed, history, Ts, AXLE_LENGTH):
    """
    ---
    Description: Prediction step of the Kalman filter
    ---
    Inputs:
        - state = np.array([x_k-1,y_k-1,theta_k-1]) is the state at step k-1. 
                  theta is in rad.
        - Sigma: 3*3 np.array, covariance matrix of the states
        - motorspeed = [speedR, speedL] is the speed of motors in pixels/s
        - history = [state_(k-N), state_(k-(N-1)), ..., state(k-1)] 
                    where N is the horizon chosen for the history
        - Ts: sampling time in seconds
        - AXLE_LENGTH: length of the axle that connects the wheels
    ---
    Outputs:
        - state: prediction of the state
        - Sigma, history: updated versions of the inputs

    """
    #equations:
        #x_k = Ax_(k-1) + f(Ts, theta_k)*[vright, vleft]
        #y_k = Cx_k
    #note: f is linearized to obtain B
    
    #initialize Q (noise due to wheels) and A
    Q = np.random.normal(0.0, STD_WHEELSPEED, (2,1))
    A = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])    
    
    #obtain B
    theta_new = state[2] + ANGLE_CORRECTION_FACTOR*Ts*\
                (motorspeed[0] - motorspeed[1])/(2*AXLE_LENGTH) 
    theta_avg =(theta_new + state[2])/2
    B = 0.5*np.array([[POS_CORRECTION_FACTOR*math.cos(theta_avg),\
                       POS_CORRECTION_FACTOR*math.cos(theta_avg)],\
                      [- POS_CORRECTION_FACTOR*math.sin(theta_avg),\
                       - POS_CORRECTION_FACTOR*math.sin(theta_avg)],\
                      [ANGLE_CORRECTION_FACTOR*1/AXLE_LENGTH, \
                       - ANGLE_CORRECTION_FACTOR*1/AXLE_LENGTH]])
    
    #update history and state (note: @ is the matrix multiplication)
    history = np.roll(history, -1, 0)
    history[-1] = np.reshape(state,(1,3))
    state = A @ state + Ts*B @ motorspeed
    
    #update Sigma, matrix of covariance
    process_noise = np.diag([(B @ Q) [0][0], (B@Q) [1][0], (B@Q) [2][0]])
    Sigma = (A.T @ Sigma @ A.T) + process_noise
    
    return state,Sigma,history

#%%
def update(state_predicted, Sigma, history, camera, errpos_history, \
           errtheta_history, camera_history):
    """
    ---
    Description: Update step of the Kalman filter, with some logic to
                 ignore camera's measure if needed.
    ---
    Inputs:
        - state_predicted = np.array([x_k-1,y_k-1,theta_k-1]) contains the 
                            states calculated from the prediction state
        - Sigma (3 by 3 np.array): covariance matrix obtained after 
                                    the prediction step
        - history = [state_(k-N), state_(k-(N-1)), ..., state(k-1)] 
                    where N is the horizon chosen for the history
        - camera = [(x,y), theta, is_working]:
                    position of thymio according to vision. 
                    is_working is a boolean: indicates whether Thymio was found
        - errpos_history: history of errors between camera's measures
                          and predicted position
        - errtheta_history: history of errors between camera's measures
                            and predicted angle
        - camera_history: history of positions and angles read by the camera
    ---
    Outputs:
        - state: filtered version of the state if the update is applied ; 
                 state_predicted otherwise
        - Sigma: filtered version of Sigma if the update is applied ; 
                 equal to the input otherwise
        - other outputs: updated versions of histories
    """
    state = state_predicted
    #update camera history
    camera_history = np.roll(camera_history, -1, 0)
    camera_history[-1]= np.reshape([camera[0][0],camera[0][1],camera[1]],(1,3))
    
    #initialize matrices
    avg_history = np.mean(history,0)
    #y_k = C*x_k
    C = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]) 
    R = np.diag([np.random.normal(0.0, STD_MEASURE_COORDS), \
                 np.random.normal(0.0, STD_MEASURE_COORDS), \
                 np.random.normal(0.0, STD_MEASURE_THETA)]) #measure noise
    
    #calculate updated state
    #measured states
    y = np.array([[camera[0][0]], [camera[0][1]], [camera[1]]])  
    innov = y - (C @ state_predicted)
    s = (C @ Sigma @ C.T) + R
    #optimal gain
    K = Sigma @ C.T @ np.linalg.inv(s) 
    
    state_update = state_predicted + K @ innov
    
    #if the Thymio is lost or its angle keeps being wrong
    #(continuous mismatch between prediction and measure)
    #then force a "hard" update of the model (i.e. assume measure=reality)
    if thymio_is_lost(errpos_history)\
    or thymio_looks_elsewhere(errtheta_history):
        cam_x = np.mean([measure[0][0] for measure in camera_history])
        cam_y = np.mean([measure[0][1] for measure in camera_history])
        cam_theta = np.mean([measure[0][2] for measure in camera_history])
        
        state = np.array([[cam_x],[cam_y],[cam_theta]])
        
        #reset error histories
        errpos_history = [0 for i in errpos_history]
        errtheta_history = [0 for i in errtheta_history]
        
        print("WARNING: A hard-update was completed")
    
    #if the thymio is not lost and the update value is not an outlier,
    #then proceed to update
    #(prevents isolated mismatch between prediction and measure,
    #when camera "jumps" for just one frame)
    else:
        #update error histories
        errpos_history = np.roll(errpos_history, -1)
        
        #euclidean distance b/w (x,y) measured & predicted
        errpos_history[-1] = math.dist((state_predicted[0][0],\
                                        state_predicted[1][0]),\
                                       (camera[0][0], camera[0][1]))    
        errtheta_history = np.roll(errtheta_history, -1)
        #abs difference b/w theta measured & predicted
        errtheta_history[-1] = abs(state_predicted[2][0] - camera[1])  
        
        #check whether state_update must be kept
        posx = state_update[0][0]
        posy = state_update[1][0]
        hisx = avg_history[0][0]
        hisy = avg_history[0][1]
        
        if (math.dist((posx,posy),(hisx,hisy)) < THRESHOLD_DISTANCE\
        and abs(state_update[2] - avg_history[0][2]) < THRESHOLD_THETA):
            state = state_update
            Sigma = (np.eye(3) - K@C) @ Sigma
            
            HISTORY_SIZE = np.size(history, 0)
            history = [np.reshape(state,(1,3))]*HISTORY_SIZE
    
        
    return state, Sigma, errpos_history, errtheta_history, camera_history

#%%
def calibrate_motorspeed(motorspeed, meters_to_pixels):
    """
    ---
    Description: Converts the reading from odometry (unknown units)
                to a linear speed in pixels/s.    
                Measurements done in a 2.51m long corridor
    ---
    Inputs:
        - motorspeed, wheel speed of Thymio read by sensors (unknown units)
        - meters_to_pixels: factor to go from m/s to pixels/s
    ---
    Outputs:
        - motorspeed, linear wheel speed of Thymio in pixels/s

    """
    #convert motor speed to linear speed (m/s)
    
    #instruction given -> ??? which instructions? 
    thymio_instruction = [-300,-250,-200,-150,-100,-50,50,100,150,200,250,300]   
    #time to cross corridor [s]
    chrono = [-27,-33,-41,-54,-119,-140,140,119,54,41,33,27]  
    #speed [m/s]                
    lin_speed = [DISTANCE/t for t in chrono]        
    
    #interpolate to convert instructions (motorspeed) to linear speed
    f = interp1d(thymio_instruction, lin_speed)
    
    speedRThymio = motorspeed[0]
    speedLThymio = motorspeed[1]
 
    if speedRThymio > 0 and speedLThymio > 0 :
        speedRMetric = f(min(speedRThymio,300)).item()
        speedLMetric = f(min(speedLThymio,300)).item()
    elif speedRThymio < 0 and speedLThymio < 0 :
        speedRMetric = f(max(speedRThymio,-300)).item()
        speedLMetric = f(max(speedLThymio,-300)).item()
    elif speedRThymio > 0 and speedLThymio < 0 :
        speedRMetric = f(min(speedRThymio, 300)).item()
        speedLMetric = f(max(speedLThymio,-300)).item()
    else: #both 0 or speedRThymio < 0 and speedLThymio > 0
        speedRMetric = f(max(speedRThymio, -300)).item()
        speedLMetric = f(min(speedLThymio, 300)).item()

    speedRPixels = int(speedRMetric*meters_to_pixels)
    speedLPixels = int(speedLMetric*meters_to_pixels)
    
    #return a 2-by-1 np.array
    motorspeed = np.array([[speedRPixels],[speedLPixels]])
    return motorspeed

#%%
def thymio_is_lost(errpos_history):
    """
    ---
    Description: Indicates if Thymio is not where we think it is.
    ---
    Inputs:
        - errpos_history: history of differences between prediction and
                          camera's measure
    ---
    Outputs:
        - isLost: True if all errors in the history are above threshold
                  False if at least one error in the history is below 
    """
    
    isLost = True
    for err in errpos_history:
        if err < THRESHOLD_DISTANCE:
            isLost = False
               
    return isLost

#%%
def thymio_looks_elsewhere(errtheta_history):
    """
    ---
    Description: Indicates if Thymio faces a wrong direction
    ---
    Inputs: 
        - errtheta_history: history of differences between prediction and
                            camera's measure
    ---
    Outputs: 
        - looksElsewhere: True if all errors in the history are above threshold
                          False if at least one error in the history is below 
    """
    looksElsewhere = True
    for err in errtheta_history:
        if err < THRESHOLD_THETA:
            looksElsewhere = False
    
    return looksElsewhere
