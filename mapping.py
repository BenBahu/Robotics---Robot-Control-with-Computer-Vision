import cv2
import numpy as np
import math

# Global variables initialization
running = False
cap = None
start_pos = None
goal_pos = None
start_angle = None
reference_pos = None
ref_angle = None
robot_pos = None  # (x, y, angle)

# Define grid size and workspace dimensions
GRID_SIZE = 30
workspace_width = 500  # Width in pixels
workspace_height = 500  # Height in pixels
MARGIN = 100

def pixel_to_grid(x, y, grid_width, grid_height, workspace_width, workspace_height):
    """ Maps the pixels coordinates to the grid workspace """
    
    grid_x = min(int(x / (workspace_width / grid_width)), grid_width - 1)
    grid_y = grid_height - 1 - min(int(y / (workspace_height / grid_height)), grid_height - 1)
    return grid_x, grid_y

def start_capture():
    """ Video capture from the camera """
    
    global running, cap
    if not running:
        cap = cv2.VideoCapture(0)  # Adjust camera index if necessary
        while not cap.isOpened():
            print("Error: Could not open the webcam.")
            cap = cv2.VideoCapture(1)
        running = True
    return cap

def reset_grid():
    global start_pos, goal_pos, start_angle, reference_pos, ref_angle
    start_pos = None
    goal_pos = None
    start_angle = None
    reference_pos = None
    ref_angle = None

def get_grid():
    """ Markers recognition on the environment, grid and positions caluclations """
    
    global start_pos, goal_pos, start_angle, reference_pos, ref_angle
    
    # Define the ArUco dictionary and detector parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    parameters = cv2.aruco.DetectorParameters()
    
    cap = start_capture()
    
    # Define grid size and workspace dimensions
    GRID_SIZE = 500
    workspace_width = 500  # Width in pixels
    workspace_height = 500  # Height in pixels
    
    # Initialize the grid
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
    
    # Start video capture
    ret, frame = cap.read()
    while not ret:
        print("Error: Could not read the frame.")
        ret, frame = cap.read()
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)		
    
    # Default values for positions
    start_pos = None
    robot_pos = None
    
    if ids is not None:
        for i, corner in enumerate(corners):
            marker_id = ids[i][0]
            
            # Extract the top-left (0) and top-right (1) corner points
            top_left = corner[0][0]
            top_right = corner[0][1]
            bottom_right = corner[0][2]
            bottom_left = corner[0][3]
    
            # Compute the angle of the top edge with respect to the X-axis
            dx = top_right[0] - top_left[0]
            dy = top_right[1] - top_left[1]
            angle = math.degrees(math.atan2(dy, dx))  # Angle in degrees
    
            # Calculate the center of the marker
            center_x = int((top_left[0] + top_right[0]) / 2)
            center_y = int((top_left[1] + top_right[1]) / 2)
            grid_x, grid_y = pixel_to_grid(center_x, center_y, GRID_SIZE, GRID_SIZE, workspace_width, workspace_height)
    
            tr_x, tr_y = pixel_to_grid(top_right[0], top_right[1], GRID_SIZE, GRID_SIZE, workspace_width, workspace_height)
            bl_x, bl_y = pixel_to_grid(bottom_left[0], bottom_left[1], GRID_SIZE, GRID_SIZE, workspace_width, workspace_height)
    
            if marker_id == 3:  # Goal marker
                grid[grid_x, grid_y] = 1
                goal_pos = (grid_x, grid_y)
                cv2.putText(frame, f"Goal, ID = {marker_id}", (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Goal, Pos = {grid_x, grid_y}", (center_x, center_y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
            elif marker_id in [7]:  # Obstacle markers, optionally add multiple ids
                for i in range(bl_x-MARGIN,tr_x+MARGIN):
                    for j in range(bl_y-MARGIN,tr_y+MARGIN):	
                        grid[j, i] = -1
                cv2.putText(frame, f"Obstacle, ID = {marker_id}", (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
            elif marker_id == 4:  # Start position
                grid[grid_x, grid_y] = 0
                start_pos = (grid_x, grid_y)
                robot_pos = (grid_x, grid_y, 180-angle)  # Update robot_pos
                if ref_angle is not None:
                    start_angle = angle - ref_angle
                    cv2.putText(frame, f"Start, ID = {marker_id}", (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(frame, f"Angle: {180-angle:.2f}", (center_x, center_y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(frame, f"Start, Pos = {grid_x, grid_y}", (center_x, center_y - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, "Start", (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
            elif marker_id == 1:  # Reference marker
                reference_pos = (grid_x, grid_y)
                ref_angle = angle
                cv2.putText(frame, f"Reference, ID = {marker_id}", (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                cv2.putText(frame, f"Reference, Pos = {grid_x, grid_y}", (center_x, center_y - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
            else:  # Default: Free space
                grid[grid_x, grid_y] = 0
                cv2.putText(frame, "Free", (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
    
    # If the start marker is not detected, ensure robot_pos is None
    if start_pos is None:
            robot_pos = None
            goal_pos
    
    ########################################
    # Display the video feed with annotations
    cv2.imshow('Aruco Marker Detection and Mapping', frame)
    return grid, start_pos, goal_pos, start_angle, robot_pos