import cv2
import numpy as np
import freenect

"""This code first gets the depth map and color image from the Kinect using the freenect library. It then converts the color image to grayscale and applies a threshold to find the contours of the objects on the table.

For each contour, the code finds the center of mass using the cv2.moments function, and then converts the (x, y) coordinates to 3D coordinates using the depth_to_3d function. The resulting 3D coordinates are then printed to the console and a circle is drawn around the object on the color image.

Finally, the color image is displayed using the cv2.imshow function. Note that you may need to adjust the threshold value or other parameters in this code to obtain accurate results for your specific setup."""

# Function to get the depth map from the Kinect
def get_depth():
    depth, _ = freenect.sync_get_depth()
    return depth

# Function to get the color image from the Kinect


def get_color():
    color, _ = freenect.sync_get_video()
    color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    return color

# Convert the depth map to 3D coordinates in meters


def depth_to_3d(x, y, depth):
    fx_d = 1.0 / 5.9421434211923247e+02
    fy_d = 1.0 / 5.9104053696870778e+02
    cx_d = 3.3930780975300314e+02
    cy_d = 2.4273913761751615e+02

    z = depth[y, x] * 0.001
    x_ = (x - cx_d) * z * fx_d
    y_ = (y - cy_d) * z * fy_d

    return (x_, y_, z)


# Get the depth map and color image from the Kinect
depth = get_depth()
color = get_color()

# Convert the color image to grayscale and apply a threshold
gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

# Find contours in the thresholded image
contours, hierarchy = cv2.findContours(
    thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Draw the contours on the color image
cv2.drawContours(color, contours, -1, (0, 255, 0), 3)

# For each contour, find the center of mass and the 3D coordinates of a point on the contour
for c in contours:
    M = cv2.moments(c)
    if M["m00"] == 0:
        continue
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    x, y, z = depth_to_3d(cx, cy, depth)
    print("Object at (", x, ", ", y, ", ", z, ")")
    cv2.circle(color, (cx, cy), 5, (0, 0, 255), -1)

# Display the color image
cv2.imshow("color", color)
cv2.waitKey(0)
cv2.destroyAllWindows()
