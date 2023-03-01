
import numpy as np
import math
import cv2

print(cv2.__version__)

# Define camera intrinsic parameters
fx = 500.0
fy = 500.0
cx = 320.0
cy = 240.0

# Define camera extrinsic parameters
cam_pos = np.array([0, 0, 1])
cam_dir = np.array([0, 0, -1])
cam_up = np.array([0, 1, 0])
cam_rot = cv2.Rodrigues(cam_dir.astype('float32'))[0]

# Define robot arm parameters
link_lengths = [1.0, 1.0, 1.0]

# Capture image from camera
cap = cv2.VideoCapture(0)
ret, frame = cap.read()

# Detect object in image
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 127, 255, 0)
contours, hierarchy = cv2.findContours(
    thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
if len(contours) > 0:
    contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(contour)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    object_pos_cam = np.array([cx, cy, 1])

    # Convert camera coordinates to world coordinates
    object_pos_world = np.matmul(np.linalg.inv(
        cam_rot), (object_pos_cam - [cx, cy, 0])) + cam_pos

    # Plan robot arm motion to grasp object
    dist = math.sqrt(object_pos_world[0]**2 + object_pos_world[1]**2)
    if dist <= sum(link_lengths):
        theta1 = math.atan2(object_pos_world[1], object_pos_world[0])
        d = math.sqrt(object_pos_world[0]**2 + object_pos_world[1]**2)
        alpha = math.atan2(object_pos_world[2], d)
        beta = math.acos((link_lengths[0]**2 + link_lengths[1]**2 - d**2 -
                         object_pos_world[2]**2) / (2 * link_lengths[0] * link_lengths[1]))
        theta2 = alpha + beta
        theta3 = math.acos((d**2 + object_pos_world[2]**2 + link_lengths[0]**2 - link_lengths[1]**2) / (
            2 * link_lengths[0] * math.sqrt(d**2 + object_pos_world[2]**2)))
        theta4 = math.atan2(object_pos_world[2], d) - theta2 - theta3

        # Send robot arm commands to execute motion
        # ...
        print('Send robot arm commands to execute motion')
# Release camera and close windows
cap.release()
cv2.destroyAllWindows()
