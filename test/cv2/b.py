import freenect
import numpy as np
import cv2
import open3d as o3d


fx = 5.7616540758591043e+02
fy = 5.7375619782082447e+02
cx = 3.1837902690380988e+02
cy = 2.3743481382791673e+02
"""
The default extrinsic parameters assume that the Kinect is mounted at a height of 1.25 meters 
and pointing downwards at a 27-degree angle.
"""
Translation = [0.0, 0.0, 0.0]
Rotation = [-0.5, 0.5, -0.5, 0.5]

# Set up Open3D visualization
#vis = o3d.visualization.Visualizer()
# vis.create_window()

# Set up Kinect V1 device
#ctx = freenect.init()
#dev = freenect.open_device(ctx, 0)

# Set up depth and RGB streams
# freenect.sync_set_led(0, dev)
#freenect.sync_set_video_format(freenect.VIDEO_RGB, freenect.RESOLUTION_MEDIUM, dev)
#freenect.sync_set_depth_format(freenect.DEPTH_REGISTERED, freenect.RESOLUTION_MEDIUM, dev)

# Get depth and RGB frames from Kinect
depth, timestamp_depth = freenect.sync_get_depth()
height, width = depth.shape
rgb, _ = freenect.sync_get_video()
# cv2.imshow('rgb', img)

bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

intrinsics = o3d.camera.PinholeCameraIntrinsic(
    depth.shape[1], depth.shape[0], fx, fy, cx, cy)
extrinstic = np.eye(4)
# Convert the depth image to float32 format and meter in unit
depth = depth.astype('float32') / 1000.0

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    o3d.geometry.Image(rgb), o3d.geometry.Image(depth))
# Create pcd from depth and img.
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image, intrinsics, extrinstic)
arr_pcd_points = np.asarray(pcd.points)

# Set point cloud colors to be the RGB values from the image
pcd.colors = o3d.utility.Vector3dVector(rgb.reshape(-1, 3) / 255.0)

# flip it, otherwise the pcd will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

# Convert the point cloud to a numpy array
#arr_pcd_points = np.asarray(pcd.points)
#arr_pcd_points = arr_pcd_points.reshape(-1, 1, 3)

# convert point cloud colors to 0-255 range coz cv2 uses 0-255
arr_pcd_colors = np.asarray(pcd.colors) * 255
arr_pcd_colors = arr_pcd_colors.astype(np.uint8)
# arr_pcd_colors = arr_pcd_colors.reshape(height, width, 3) # Reshape to (height, width, channels)
arr_pcd_colors = arr_pcd_colors.reshape(-1, 1, 3)
hsv = cv2.cvtColor(arr_pcd_colors, cv2.COLOR_RGB2HSV)

# Define the lower and upper bounds of the color red in the HSV color space
lower_red = np.array([0, 50, 50], dtype=np.uint8)
upper_red = np.array([10, 255, 255], dtype=np.uint8)
mask1 = cv2.inRange(hsv, lower_red, upper_red)
lower_red = np.array([170, 50, 50])
upper_red = np.array([180, 255, 255])
mask2 = cv2.inRange(hsv, lower_red, upper_red)
# Create a mask that includes both ranges of red color
mask = cv2.bitwise_or(mask1, mask2)

# Apply a morphological opening operation to remove small noise
kernel = np.ones((5, 5), np.uint8)
#mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
red_pcd = pcd.select_by_index(np.where(mask)[0])
o3d.visualization.draw_geometries([red_pcd], window_name='red pcd')

# Apply morphological operations
filtered_pcd = red_pcd.voxel_down_sample(voxel_size=0.05)
filtered_pcd, _ = red_pcd.remove_statistical_outlier(
    nb_neighbors=10, std_ratio=1.0)
o3d.visualization.draw_geometries([filtered_pcd], window_name='filtered pcd')

red_box_points = np.asarray(red_pcd.points)[np.where(mask > 0)]
x, y, z = np.mean(red_box_points, axis=0)

print("Red box center coordinates: ({}, {}, {})".format(x, y, z))


# Release Kinect device and free resources
# freenect.close_device(dev)
# freenect.shutdown(ctx)
