def local_speeds(prox_values):
    """ Local speeds calculation for obstacle avoidance based on the sensors values """

    #Variables initialization
    motor_left_speed = 0
    motor_right_speed = 0
    threshold = 5
    obstacle = True

    # Weights of each sensor
    w_l = [40,  20, -20, -20, -40]
    w_r = [-40, -20, -20,  20,  40]

    # Scale factors for sensors
    sensor_scale = 1000
    
    y = [0,0]
    x = [0,0,0,0,0]
        
    if obstacle:
        for i in range(len(x)):
            # Get and scale inputs
            x[i] = prox_values[i] / sensor_scale
            
            # Compute outputs of neurons and set motor powers
            y[0] = y[0] + x[i] * w_l[i]
            y[1] = y[1] + x[i] * w_r[i]

    if not obstacle: 
        # Switch from goal tracking to obstacle avoidance if an obstacle is detected
        for i in range(len(x)):
            if prox_values[i] >= threshold:
                obstacle = True 
                 
    elif obstacle:
        # Switch back to goal tracking only if all sensors are below seuil_max
        all_below_seuil = True 
        for i in range(len(x)):
            if prox_values[i] >= threshold:
                all_below_seuil = False  
                 
        if all_below_seuil:
            obstacle = False

    if  not obstacle :
        # goal tracking: no local speeds added
        motor_left_speed = 0 
        motor_right_speed = 0 
    else:
        # obstacle avoidance: accelerate wheels near obstacle
        motor_left_speed = y[0]
        motor_right_speed = y[1]

    return(obstacle, motor_left_speed, motor_right_speed)