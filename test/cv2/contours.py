
import time
import random
import signal
import freenect as knect

import numpy as np
import cv2
import open3d as o3d

# from open3d import PointCloud.create_from_depth_image
"""
Grabs a depth map from the Kinect sensor and creates an image from it.
clips the array so that the maximum depth is 1023 (effectively removing distance objects and noise) and turns it into
an 8 bit array (which OpenCV can render as grayscale). The array returned from getDepthMap can be used like a grayscale
OpenCV image – to demonstrate I apply a Gaussian blur. Finally, imshow renders the image in a window and waitKey is 
there to make sure image updates actually show.
"""


def get_depth():
    depth, timestamp = knect.sync_get_depth()

    np.clip(depth, 0, 2**10 - 1, depth)
    depth >>= 2
    depth = depth.astype(np.uint8)

    return depth


def depth_to_3d(x, y, depth):
    fx_d = 1.0 / 5.9421434211923247e+02
    fy_d = 1.0 / 5.9104053696870778e+02
    cx_d = 3.3930780975300314e+02
    cy_d = 2.4273913761751615e+02

    z = depth[y, x] * 0.001
    x_ = (x - cx_d) * z * fx_d
    y_ = (y - cy_d) * z * fy_d

    return (x_, y_, z)

def get_video():
    """
    Args:
        video: A numpy array with 1 byte per pixel, 3 channels RGB
    Returns:
        A numpy array with with 1 byte per pixel, 3 channels BGR
    """
    array, _ = knect.sync_get_video()
    # array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array


def compute_pcd(depth_image):
    fx = 5.7616540758591043e+02
    fy = 5.7375619782082447e+02
    cx = 3.1837902690380988e+02
    cy = 2.3743481382791673e+02

    height, width = depth_image.shape
    length = height * width

    # compute indices:
    jj = np.tile(range(width), height)
    ii = np.repeat(range(height), width)

    # Compute constants:
    xx = (jj - cx) / fx
    yy = (ii - cy) / fy

    # transform depth image to vector of z:
    length = height * width
    z = depth_image.reshape(height * width)
    pcd = np.dstack([(ii - cx) * z / fx,
                     (jj - cy) * z / fy,
                     z]).reshape((length, 3))
    
    # compute point cloud
    pcd = np.dstack((xx * z, yy * z, z)).reshape((length, 3))
    return pcd

'''
ctx = knect.init()
dev = knect.open_device(ctx, 0)

# Set depth mode to 11-bit resolution and 30 frames per second
knect.set_depth_mode(dev, knect.RESOLUTION_MEDIUM,
                        knect.DEPTH_REGISTERED)

'''
#######################################################################################

# get the intrinsic camera parameters
fx = 5.7616540758591043e+02
fy = 5.7375619782082447e+02
cx = 3.1837902690380988e+02
cy = 2.3743481382791673e+02

print("fx: ", fx)
print("fy: ", fy)
print("cx: ", cx)
print("cy: ", cy)

# Load a depth image from the Kinect and convert it into a point cloud
depth_image= get_depth()
color = get_video()

# Convert the color image to grayscale and apply a threshold
gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

# Find contours in the thresholded image
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Draw the contours on the color image
cv2.drawContours(color, contours, -1, (0, 255, 0), 3)

# For each contour, find the center of mass and the 3D coordinates of a point on the contour
for c in contours:
    M = cv2.moments(c)
    if M["m00"] == 0:
        continue
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    x, y, z = depth_to_3d(cx, cy, depth_image)
    print("Object at (", x, ", ", y, ", ", z, ")")
    cv2.circle(color, (cx, cy), 5, (0, 0, 255), -1)

# Display the color image
cv2.imshow("color", color)
cv2.waitKey(0)
cv2.destroyAllWindows()


height, width = depth_image.shape
print(f"Image resolution: {depth_image.shape}")
print(f"Data type: {depth_image.dtype}")
print(f"Min value: {np.min(depth_image)}")
print(f"Max value: {np.max(depth_image)}")


# convert the depth frame to a point cloud using Open3D
intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
pcd = o3d.geometry.PointCloud.create_from_depth_image(
    o3d.geometry.Image(knect.sync_get_depth()), intrinsics)
# display the point cloud
o3d.visualization.draw_geometries([pcd])

# pcd = compute_pcd(depth_image)
pcd_o3d = o3d.geometry.PointCloud()  # create point cloud object
pcd_o3d.points = o3d.utility.Vector3dVector(pcd)  # set pcd_np as the point cloud points

# Create a 3D coordinate system:
origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
# geometries to draw:
geometries = [pcd_o3d, origin]

# Visualize:

#o3d.visualization.draw_geometries([pcd_o3d])
# o3d.visualization.draw_geometries(geometries)
'''
intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
pcd = o3d.geometry.PointCloud.create_from_depth_image(
    o3d.geometry.Image(depth_image), intrinsics)
'''

# pcd = o3d.geometry.create_point_cloud_from_depth_image(depth_image, intrinsics)

# visualize:
# flip it, otherwise the pointcloud will be upside down
# pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0, -1,0],[0,0,0,1]])
#o3d.visualization.draw_geometries([pcd])

# Segment the point cloud to isolate the table and the object of interest
plane_model, inliers = pcd_o3d.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
table_cloud = pcd_o3d.select_by_index(inliers)
object_cloud = pcd_o3d.select_by_index(inliers, invert=True)

# Estimate the centroid of the object in camera coordinates
object_center = np.asarray(object_cloud.points).mean(axis=0)
'''
# Transform the object centroid from camera coordinates to world coordinates
camera_to_world_transform =  # calculate transformation matrix from camera to world
object_center_world = np.matmul(
    camera_to_world_transform, np.append(object_center, 1))
'''
# Print the object center in world coordinates
print("Object center in world coordinates: ", object_center[:3])
#####################################################################################

cv2.namedWindow('Depth')
cv2.namedWindow('Video')
"""
keep_running = True
last_time = 0
def body(dev, ctx):
    global last_time
    if not keep_running:
        raise knect.Kill
    if time.time() - last_time < 3:
        return
    last_time = time.time()
    led = random.randint(0, 6)
    tilt = random.randint(0, 30)
    knect.set_led(dev, led)
    knect.set_tilt_degs(dev, tilt)
    print('led[%d] tilt[%d] accel[%s]' % (led, tilt, knect.get_accel(dev)))


def handler(signum, frame):
    #Sets up the kill handler, catches SIGINT
    global keep_running
    keep_running = False

print('Press Ctrl-C in terminal to stop')
signal.signal(signal.SIGINT, handler)
knect.runloop(body=body)
"""


while 1:
    # get a frame from RGB kinect camera
    bgr = get_video()
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Create kernel to use in filter
    kernel = np.ones((5, 5), np.uint8)
    # list yellow range
    lower_yellow = np.array([15, 60, 30])
    upper_yellow = np.array([30, 255, 255])
    # Create filter for yellow
    # Threshold the HSV image to get only yellow colors
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Use morphology open to rid of false pos and false neg (noise)
    opening_y = cv2.morphologyEx(
        yellow_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    # bitwise and the opening filter with the original frame
    result_y = cv2.bitwise_and(bgr, bgr, mask=opening_y)

    # list green range
    lower_green = np.array([45, 45, 10])
    upper_green = np.array([100, 255, 255])

    # list blue range
    lower_blue = np.array([110, 70, 30])
    upper_blue = np.array([130, 255, 255])

    # knect.camera_to_world()
    # knect.convert_depth_to_world()

    depth = get_depth()

    # o3d.geometry.create_point_cloud_from_depth_image(depth, intrinsic, extrinsic=(with default value), depth_scale=1000.0, depth_trunc=1000.0, stride=1)

    d3 = np.dstack((depth, depth, depth)).astype(np.uint8)
    da = np.hstack((d3, bgr))
    # Simple Downsample
    cv2.imshow('both', np.array(da[::2, ::2, ::-1]))
    if cv2.waitKey(10) == 27:
        break


while True:
    depth = get_depth()

    blur = cv2.GaussianBlur(depth, (5, 5), 0)

    cv2.imshow('image', blur)
    cv2.waitKey(10)


def draw():
    (depth, timestamp) = knect.sync_get_depth()
    video = knect.sync_get_video()
    # cv2.imshow(video)
    # depth's type is np.ndarray. gray scale. # (480, 640)
    print(depth)
    # data range from 0 ~ 2047
    print(depth.max())
    """
    0 is the furthest point, which is considered infinity. The larger numbers represent the pixels
    closest to the kinect. And 2047 in particular is considered a pixel error.
    In short, the depth data are data of 11 bits (211 = 2048).
    """
    # To show the image we had to convert uint16 to uint8 (as the color range of RGB is 2^8 = 256).
    output = depth.astype(np.uint8)
    output = cv2.cvtColor(output, cv2.COLOR_GRAY2RGB)

    output = np.hstack(video, depth)
    cv2.imshow('Depth', output)


def loop(function, delay=5):
    while True:
        function()
        cv2.waitKey(delay)  # Any key is expected to be pressed


loop(draw)
cv2.destroyAllWindows()  # A clean up is not too much
