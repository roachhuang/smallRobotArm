"""sudo apt-get update
sudo apt-get install libfreenect2-dev python-opencv
"""
import numpy as np
import cv2
import freenect2
import frame_convert2

# create a Kinect sensor object
kinect = freenect2.Kinect()

# start the Kinect sensor and enable depth capture
kinect.start()
kinect.enable_depth()

# capture a depth frame from the Kinect sensor
frame = kinect.get_last_depth_frame()

# convert the depth frame to a 2D array of distances
depth = frame_convert2.pretty_depth_cv(frame)

# set the intrinsic parameters of the Kinect sensor
fx = 365.456  # focal length in x direction
fy = 365.456  # focal length in y direction
cx = 256.0   # optical center x coordinate
cy = 212.0   # optical center y coordinate

# compute the world coordinates of each pixel in the depth frame
rows, cols = depth.shape
x, y = np.meshgrid(np.arange(cols), np.arange(rows))
x = x.flatten()
y = y.flatten()
z = depth.flatten()
X = (x - cx) * z / fx
Y = (y - cy) * z / fy
Z = z

# convert the world coordinates to a 3D point cloud
points = np.column_stack((X, Y, Z))

# find the centroid of the point cloud
centroid = np.mean(points, axis=0)

# print the coordinates of the centroid
print("Centroid:", centroid)

# stop the Kinect sensor
kinect.stop()
