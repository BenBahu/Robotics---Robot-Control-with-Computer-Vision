import numpy as np
import math

ROBOT_CENTER_TO_WHEEL = 0.048

def motor_controller(actual_state, goal_state) -> tuple[float, float, bool]:  
    """ Astolfi controller to go from one point in the path to the next one """
    
    goal_radius = 10      # Radius (in pixels from the grid) to consider the point reached
    error_threshold = 30  # Angular error threshold to privilege the robot turning on itself before starting moving forward
    v = 200               # Linear speed
    
    # Extract the values from the arguments (position and goal)
    x, y, theta_deg = actual_state
    x_goal, y_goal = goal_state
    
    theta = math.radians(theta_deg)
    
    # Distance to the goal error
    dx = x_goal - x
    dy = y_goal - y
    rho = math.sqrt(dx**2 + dy**2)
    
    # Angle to reach the goal
    angle_to_goal = math.atan2(dy, dx) 
    
    # Angle error between the robot and the goal angle
    alpha = angle_to_goal - theta
    
    # Alpha normalization between -Pi and Pi
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))

    alpha = math.degrees(alpha)

    # Only turning to reduce the angular error before moving forward
    if np.abs(alpha) > error_threshold:
        v = 0
    
    # Cheking if next point reached
    if rho <= goal_radius:
        return 0, 0, True
    
    # Astolfi angle gain
    k_alpha = 50.0

    # Angular speed
    omega = k_alpha * alpha
    
    # Speeds calculation for the motors
    v_left = v - (ROBOT_CENTER_TO_WHEEL * omega)
    v_right = v + (ROBOT_CENTER_TO_WHEEL * omega)

    return v_left, v_right, False