import open3d as o3d

import numpy as np

# Transformation matrix from Kinect sensor frame to robot arm frame
T_kinect_to_arm = np.array([[0.707,  0.707,  0.,  0.],
                            [-0.707,  0.707,  0.,  0.],
                            [0.,  0.,  1.,  0.],
                            [0.,  0.,  0.,  1.]])


def get_depth():
    depth, _ = freenect.sync_get_depth()
    depth = depth.astype(np.float32)
    depth[depth > 2047] = 2047
    depth /= 2047
    return depth


depth = get_depth()
pcd = o3d.geometry.PointCloud.create_from_depth_image(depth, intrinsic)

# Define color range for red objects
color_range = [[200, 0, 0], [255, 50, 50]]

# Filter point cloud to isolate red objects
red_objects = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(
    min_bound=[0, 0, 0], max_bound=[1, 1, 1]))
red_objects = red_objects.voxel_down_sample(voxel_size=0.01)
red_objects.paint_uniform_color([1, 0.706, 0])
red_objects = red_objects.crop_point_cloud(red_objects)
red_objects = red_objects.extract_clusters(
    tolerance=0.05, min_points=50, max_points=100000)

# Extract 3D coordinates of red object
red_object = red_objects[0]
red_object_center = red_object.get_center()

# Transform red_object_center to robot arm frame
red_object_center_arm_frame = T_kinect_to_arm @ np.append(red_object_center, 1)
red_object_center_arm_frame = red_object_center_arm_frame[:3]

# Send coordinates to robot arm control interface
robot_arm.move_to(red_object_center)

# Pick up object
robot_arm.pickup()



################################## ver 2
while True:
    # capture point cloud from Kinect
    array, _ = freenect.sync_get_depth()
    array = array.astype(np.uint16)
    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        o3d.geometry.Image(array))

    # extract red objects from point cloud
    red_objects = pcd.select_by_index(
        np.where(np.logical_and(np.logical_and(pcd.colors[:, 0] > 0.8, pcd.colors[:, 1] < 0.2), pcd.colors[:, 2] < 0.2)))
    red_objects = red_objects.voxel_down_sample(voxel_size=0.01)
    red_objects.paint_uniform_color([1, 0.706, 0])
    red_objects = red_objects.crop_point_cloud(red_objects)
    red_objects = red_objects.extract_clusters(
        tolerance=0.05, min_points=50, max_points=100000)

    # if a red object is found, pick it up with the robot arm
    if len(red_objects) > 0:
        target = red_objects[0].get_center()
        robot_arm.move_to(target)
        robot_arm.pickup()
