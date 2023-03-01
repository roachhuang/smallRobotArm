import cv2
import numpy as np
from freenect import sync_get_depth as get_depth, sync_get_video as get_video

# Initialize Kinect sensor
ctx = freenect.init()
dev = freenect.open_device(ctx, 0)

# Set depth mode to 11-bit resolution and 30 frames per second
freenect.set_depth_mode(dev, freenect.RESOLUTION_MEDIUM,
                        freenect.DEPTH_REGISTERED)

# Define ROI for object of interest
x, y, w, h = 100, 100, 200, 200

while True:
    # Capture RGB image and depth map from Kinect
    (depth, _), (rgb, _) = get_depth(), get_video()

    # Crop RGB image and depth map to ROI
    rgb_roi = rgb[y:y+h, x:x+w]
    depth_roi = depth[y:y+h, x:x+w]

    # Convert depth map to meters
    depth_meters = depth_roi.astype(np.float32) * 0.001

    # Calculate 3D coordinates of object of interest
    fx_d = 594.21
    fy_d = 591.04
    cx_d = 339.5
    cy_d = 242.7
    row_idx, col_idx = np.indices(depth_roi.shape)
    x = (col_idx - cx_d) * depth_meters / fx_d
    y = (row_idx - cy_d) * depth_meters / fy_d
    z = depth_meters

    # Compute object orientation and dimensions using OpenCV
    object_3d_points = np.array(
        [[0, 0, 0], [0, 0, h], [0, w, 0], [w, 0, 0]], dtype=np.float32)
    object_2d_points = np.array([[x[0][0], y[0][0]], [x[0][h-1], y[h-1][0]], [
                                x[w-1][0], y[0][w-1]], [x[h-1][w-1], y[h-1][w-1]]], dtype=np.float32)
    camera_matrix = np.array(
        [[fx_d, 0, cx_d], [0, fy_d, cy_d], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)
    _, rotation_vector, translation_vector = cv2.solvePnP(
        object_3d_points, object_2d_points, camera_matrix, dist_coeffs)

    # Draw object orientation and dimensions on RGB image
    rvec_matrix = cv2.Rodrigues(rotation_vector)[0]
    project_matrix = np.hstack((rvec_matrix, translation_vector))
    euler_angles = cv2.decomposeProjectionMatrix(project_matrix)[-1]
    pitch, yaw, roll = [math.radians(_) for _ in euler_angles]
    x1, y1, z1 = translation_vector
    h1, w1, l1 = h, w, z.max() - z.min()
    print("Position: ({:.2f}, {:.2f}, {:.2f}), Rotation: ({:.2f}, {:.2f}, {:.2f}), Dimensions: ({:.2f}, {:.2f}, {:.2f})".format(x1, y1, z1, pitch, yaw, roll, h1,
