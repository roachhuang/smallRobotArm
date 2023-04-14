import freenect
import numpy as np
import cv2
import open3d as o3d
import matplotlib.pyplot as plt

fx = 5.7616540758591043e+02
fy = 5.7375619782082447e+02
cx = 3.1837902690380988e+02
cy = 2.3743481382791673e+02

# Set up Open3D visualization
#vis = o3d.visualization.Visualizer()
#vis.create_window()

# Set up Kinect V1 device
#ctx = freenect.init()
#dev = freenect.open_device(ctx, 0)

# Set up depth and RGB streams
# freenect.sync_set_led(0, dev)
#freenect.sync_set_video_format(freenect.VIDEO_RGB, freenect.RESOLUTION_MEDIUM, dev)
#freenect.sync_set_depth_format(freenect.DEPTH_REGISTERED, freenect.RESOLUTION_MEDIUM, dev)

# Get depth and RGB frames from Kinect
depth, timestamp_depth = freenect.sync_get_depth()
rgb, _  = freenect.sync_get_video()
# cv2.imshow('rgb', img)

bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)


# Define the lower and upper bounds of the color red in the HSV color space
lower_red = np.array([0, 50, 50], dtype=np.uint8)
upper_red = np.array([10, 255, 255], dtype=np.uint8)
mask1 = cv2.inRange(hsv, lower_red, upper_red)

lower_red = np.array([170, 50, 50])
upper_red = np.array([180, 255, 255])
mask2 = cv2.inRange(hsv, lower_red, upper_red)

# Create a mask that includes both ranges of red color
mask = cv2.bitwise_or(mask1, mask2)

# Apply the mask to the original image to keep only the red pixels
res = cv2.bitwise_and(bgr, bgr, mask=mask.astype(np.uint8))

# Display the resulting image
# cv2.imshow('Red color detection', np.hstack([bgr, res]))

#cv2.waitKey(0)
#cv2.destroyAllWindows()

# Find contours of red object in mask
# contours, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


intrinsics = o3d.camera.PinholeCameraIntrinsic(depth.shape[1], depth.shape[0], fx, fy, cx, cy)
# Create Open3D point cloud from depth image
# Convert the depth image to float32 format
# depth = depth.astype('float32') / 1000.0
depth = depth.astype('float32')
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    o3d.geometry.Image(rgb), o3d.geometry.Image(depth))
      
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsics)

# Set point cloud colors to be the RGB values from the image
pcd.colors = o3d.utility.Vector3dVector(rgb.reshape(-1, 3) / 255.0)

# flip it, otherwise the pcd will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])

#cv2.imshow('rgbd', np.asarray(rgbd_image.color,dtype=np.uint8))
# Convert point cloud to numpy array, dtype=float
arr_rgb = np.asarray(pcd.colors, dtype=np.float32) 

# Define a red color range. need to nomalize them by dividing them by 255 
lower_red = np.array([100, 0, 0])/255.0
upper_red = np.array([255, 100, 100])/255.0
# mask = np.logical_and(arr_rgb > lower_red, arr_rgb < upper_red).all(axis=1)

# create a binary mask for the red color range
red_mask = (arr_rgb[:, 0] > 0.8) & (arr_rgb[:, 1] < 0.2) & (arr_rgb[:, 2] < 0.2)
# Create new point cloud with only the red points
red_pcd = pcd.select_by_index(np.where(red_mask)[0])
pt_red_pcd = np.asarray(red_pcd.points)
centroid = np.mean(pt_red_pcd, axis=0)
print('centroid', centroid)


# visualize the transformed red box point cloud
o3d.visualization.draw_geometries([red_pcd])

# Apply morphological operations
filtered_pcd = red_pcd.voxel_down_sample(voxel_size=0.05)
filtered_pcd, _ = red_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=1.0)
# Apply clustering
#labels = filtered_pcd.cluster_dbscan(eps=0.5, min_points=10, print_progress=True)
#max_label = labels.max()
#colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))

# Assign colors to clustered points
#filtered_pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([filtered_pcd])

red_box_points = np.asarray(pcd.points)[np.where(mask > 0)]
x, y, z = np.mean(red_box_points, axis=0)

print("Red box center coordinates: ({}, {}, {})".format(x, y, z))


# Release Kinect device and free resources
#freenect.close_device(dev)
#freenect.shutdown(ctx)
