from typing import Optional

import numpy as np
import math

WHEEL_GAP = 96
SAMPLING_TIME = 0.107
GAIN_SAMPLING = 0.8
#HEIGHT and WIDTH measured on the environment
HEIGHT_MM = 594
WIDTH_MM = 865
SAMPLING_TIME_WORKING = GAIN_SAMPLING * SAMPLING_TIME

def Kalman_filtering(actual_state: np.ndarray, mu_previous_state: np.ndarray, covar_previous_state: np.ndarray, LeftMotor_Speed: float, RightMotor_Speed: float, scaleHeight: float, scaleWidth: float) -> tuple[np.ndarray, np.ndarray]:
    """ Extended Kalman filtering for robot poisiton estimation """   

    PXL_OVER_MM_HEIGHT = HEIGHT_MM / scaleHeight
    PXL_OVER_MM_WIDTH = WIDTH_MM / scaleWidth

    Q = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.04]])
    R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.04]])
    
    RightSpeed = RightMotor_Speed*0.4 #mm/s
    LeftSpeed = LeftMotor_Speed*0.4
    
    omega = (RightSpeed - LeftSpeed) / WHEEL_GAP
    theta = SAMPLING_TIME_WORKING * omega
    v = (RightSpeed + LeftSpeed) / 2

    x_estimation = mu_previous_state[0]
    y_estimation = mu_previous_state[1]
    angle_estimation = mu_previous_state[2]
    angle_estimation = math.radians(angle_estimation)

    cos_angle = np.cos(theta + angle_estimation)
    sin_angle = np.sin(theta + angle_estimation)

    mu_prediction = np.zeros(3)
    mu_prediction[0] = x_estimation + v * SAMPLING_TIME_WORKING * cos_angle * PXL_OVER_MM_WIDTH
    mu_prediction[1] = y_estimation + v * SAMPLING_TIME_WORKING * sin_angle * PXL_OVER_MM_HEIGHT
    mu_prediction[2] = theta + angle_estimation #radians
    
    Jacob = np.eye(3)
    Jacob[0, 2] = -v * sin_angle
    Jacob[1, 2] = v * cos_angle
    covar_prediction = Jacob @ covar_previous_state @ Jacob.T + Q

    if actual_state is not None:
        y = list(actual_state)  # Convert tuple to list
        y[2] = math.radians(y[2])  # Angle in radians
        y = tuple(y)  # Convert back to tuple if needed                                                                                                          
    else:
        y = mu_prediction

    i = y - mu_prediction
    
    S = covar_prediction + R
    S_inv = np.linalg.inv(S)
    K = covar_prediction @ S_inv

    x_estimation = mu_prediction + K @ i
    covar_estimation = covar_prediction - K @ covar_prediction
    x_estimation[2] = math.degrees(x_estimation[2])
    x_estimation[2] = x_estimation[2]%360
 

    return x_estimation, covar_estimation