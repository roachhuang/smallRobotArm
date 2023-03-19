"""
This code captures depth and color images from a Kinect camera using Freenect, 
converts the depth image to a point cloud using Open3D, and displays the point cloud 
in an Open3D visualization window. The program exits when the window is closed.

Please note that this code does not perform any filtering or segmentation of the point cloud,
and the point cloud is not transformed into any specific frame of reference. It is provided
as a simple example to demonstrate how to capture point clouds from a Kinect camera using
Open3D and Freenect.
"""
import numpy as np
import open3d as o3d
import freenect

# Define Kinect camera intrinsic parameters
K = np.array([[606.687, 0, 312.653],
              [0, 606.734, 237.018],
              [0, 0, 1]])

# Define robot arm's tool frame
T_tool_to_robot = np.array([[0, 0, 1, 0.05],
                            [-1, 0, 0, 0],
                            [0, -1, 0, 0.3],
                            [0, 0, 0, 1]])

# Define pick and place poses
pick_pose = np.array([[1, 0, 0, 0.2],
                      [0, 1, 0, 0.1],
                      [0, 0, 1, 0.05],
                      [0, 0, 0, 1]])
place_pose = np.array([[1, 0, 0, 0.2],
                       [0, 1, 0, -0.1],
                       [0, 0, 1, 0.05],
                       [0, 0, 0, 1]])


# Initialize Kinect camera and Open3D visualization window
ctx = o3d.visualization.Visualizer()
ctx.create_window()

# Define the dimensions of the object (in meters)
object_width = 0.1
object_height = 0.1
# Define the depth threshold for isolating the object
depth_threshold = 500

# get the intrinsic camera parameters
fx = 5.7616540758591043e+02
fy = 5.7375619782082447e+02
cx = 3.1837902690380988e+02
cy = 2.3743481382791673e+02

while 1:
    # grab a depth frame from the Kinect
    depth, timestamp = freenect.sync_get_depth()

    # convert the depth frame to a point cloud using Open3D
    intrinsics = o3d.camera.PinholeCameraIntrinsic(640, 480, fx, fy, cx, cy)
    pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depth), intrinsics)
 
    
    """
    # Filter point cloud to remove noise and extract objects
    pcd_down = pcd.voxel_down_sample(voxel_size=0.005)
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=100))
    clusts = np.asarray(pcd_down.cluster_dbscan(
        eps=0.05, min_points=50, print_progress=True))
    max_clust = np.argmax(np.bincount(clusts))
    pcd_filtered = pcd_down.select_by_index(np.where(clusts == max_clust)[0])
    """

    # Visualize point cloud in Open3D window
    # display the point cloud

     # Transform point cloud to robot arm's tool frame
    # pcd_tool = pcd_filtered.transform(T_tool_to_robot)
    pcd_tool = pcd.transform(T_tool_to_robot)

    # Visualize point cloud and robot arm in Open3D window
    ctx.add_geometry(pcd_tool)
    ctx.add_geometry(
        o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1))
    ctx.poll_events()
    ctx.update_renderer()

    #ctx.clear_geometries() # need clear?
    #ctx.add_geometry(pcd)
    #ctx.poll_events()
    #ctx.update_renderer()

    # Check if object is in pick position and execute pick and place
    if np.linalg.norm(pick_pose[:3, 3] - pcd_tool.get_center()) < 0.01:
        print("Object is in pick position!")
        # Move robot arm to pick pose
        # Close gripper to pick object
        # Move robot arm to place
        # Open gripper to release object
        # Move robot arm to home position
        # Break out of the loop
        break



    # Exit loop if window is closed   
    if not ctx.poll_events():
        break


ctx.destroy_window()
   

