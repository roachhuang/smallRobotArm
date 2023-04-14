import open3d as o3d
import numpy as np
import cv2
import freenect
print(o3d.__version__)


def get_depth():
    depth, _ = freenect.sync_get_depth()  
    depth = depth.astype(np.uint16)
    depth = cv2.flip(depth, 0)
    return depth

# create a video capture object for the Kinect camera
cap = cv2.VideoCapture(cv2.CAP_OPENNI)

# set the video mode to 640x480 and 30fps
cap.set(cv2.CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, cv2.CAP_OPENNI_VGA_30HZ)

# read a frame from the camera
ret, frame = cap.read()

# get the intrinsic camera parameters

fx = 5.7616540758591043e+02
fy = 5.7375619782082447e+02
cx = 3.1837902690380988e+02
cy = 2.3743481382791673e+02

# release the video capture object
cap.release()


# Load a depth image from the Kinect and convert it into a point cloud
depth_image =  get_depth() # capture depth image from Kinect
height, width = depth_image.shape
intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
pcd = o3d.geometry.PointCloud.create_from_depth_image(
    o3d.geometry.Image(depth_image), intrinsics)

# Segment the point cloud to isolate the table and the object of interest
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
table_cloud = pcd.select_by_index(inliers)
object_cloud = pcd.select_by_index(inliers, invert=True)

# Estimate the centroid of the object in camera coordinates
object_center = np.asarray(object_cloud.points).mean(axis=0)
print(object_center)

# Transform the object centroid from camera coordinates to world coordinates
#camera_to_world_transform =  # calculate transformation matrix from camera to world
#object_center_world = np.matmul(
#    camera_to_world_transform, np.append(object_center, 1))

# Print the object center in world coordinates
#print("Object center in world coordinates: ", object_center_world[:3])
