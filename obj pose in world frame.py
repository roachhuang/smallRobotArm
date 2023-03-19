import open3d as o3d
import numpy as np
import freenect

# Define the intrinsic parameters of the Kinect v1 depth camera
fx_d = 594.21
fy_d = 591.04
cx_d = 320
cy_d = 240
intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 480, fx_d, fy_d, cx_d, cy_d)

# Define the voxel size and the maximum distance from the camera
voxel_size = 0.005
max_distance = 2.5

# Create a point cloud object
pcd = o3d.geometry.PointCloud()
try:
    # Start the Kinect v1 device
    ctx = freenect.init()
    
    dev = freenect.open_device(ctx, 0)
    freenect.set_depth_mode(dev, freenect.RESOLUTION_MEDIUM, freenect.DEPTH_MM)

    # Capture a depth frame from the Kinect v1 device
    depth, timestamp = freenect.sync_get_depth()

    # Convert the depth frame to a point cloud
    depth_image = depth.astype(np.float32)
    depth_image[depth_image > max_distance] = 0
    depth_image = depth_image / 1000.0
    pcd.points = o3d.utility.Vector3dVector(np.zeros((640*480, 3)))
    x = np.arange(640).reshape(1, -1).repeat(480, axis=0)
    y = np.arange(480).reshape(-1, 1).repeat(640, axis=1)
    pcd.points[:, 0] = (x - cx_d) * depth_image.reshape(-1)
    pcd.points[:, 1] = (y - cy_d) * depth_image.reshape(-1)
    pcd.points[:, 2] = depth_image.reshape(-1)
    pcd = pcd.voxel_down_sample(voxel_size)

    # Estimate the normal vectors of the point cloud
    # Estimate the object pose in the world frame using the RANSAC algorithm
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    inlier_mask = np.zeros((pcd.points.shape[0],), dtype=bool)
    inlier_mask[inliers] = True
    inlier_pcd = pcd.select_by_index(np.where(inlier_mask)[0])
    object_pose = inlier_pcd.get_oriented_bounding_box().get_rotation_matrix()

    print(object_pose)
except:
    freenect.close_device(dev)
    freenect.shutdown(ctx)
    raise 

# Stop the Kinect v1 device
freenect.close_device(dev)
freenect.shutdown(ctx)