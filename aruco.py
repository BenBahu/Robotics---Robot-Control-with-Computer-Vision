import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

# Specify the size of the tag image (e.g., 200x200 pixels)
tag_size = 200
for tag_id in range(10):
    tag = cv2.aruco.generateImageMarker(aruco_dict, tag_id, tag_size)
    output_filename = f"aruco_tag_{tag_id}.png"
    cv2.imwrite(output_filename, tag)
    print(f"ArUco tag with ID {tag_id} saved as {output_filename}")
