"""
In the code I provided, the calculate_position function returns only the (x, y) coordinates of the object, not the z coordinate. This is because the Kinect depth data is noisy and can be difficult to use for accurate depth measurement. However, if you want to obtain the z coordinate, you can modify the code to use the depth data to calculate it.

One approach is to use the cv2.convertScaleAbs function to convert the depth data to an 8-bit grayscale image, and then use a threshold to isolate the object in the image. You can then use the cv2.findContours function to find the contour of the object and calculate its area. With the area and the known dimensions of the object, you can then estimate its distance from the camera.


This code captures depth and video data from the Xbox 360 Kinect, detects the object using the depth data, and localizes the object using the x and y coordinates of the object's center. You can customize the object detection by adjusting the range in the inRange function, and customize the object localization by using the x and y coordinates as needed.
"""

import cv2
import freenect
import numpy as np
import matplotlib.pyplot as plt
import cv2
import open3d as o3d


def get_depth():
    depth, _ = freenect.sync_get_depth()
    depth = depth.astype(np.uint16)
    depth = cv2.flip(depth, 1)
    return depth


def get_video():
    video, _ = freenect.sync_get_video()
    video = cv2.cvtColor(video, cv2.COLOR_RGB2BGR)
    video = cv2.flip(video, 1)
    return video


fx = 5.7616540758591043e+02
fy = 5.7375619782082447e+02
cx = 3.1837902690380988e+02
cy = 2.3743481382791673e+02

color_raw = get_video()
depth_raw = get_depth()
height, width = depth_raw.shape

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    o3d.geometry.Image(color_raw), o3d.geometry.Image(depth_raw))
print(rgbd_image)

plt.subplot(1, 2, 1)
plt.title('Grayscale img')
plt.imshow(rgbd_image.color)

plt.subplot(1, 2, 2)
plt.title('Depth image')
plt.imshow(rgbd_image.depth)

plt.show()

# create the point cloud from images and camera intrisic params
intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

# compare the difference btw these 2 pcd 
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsics)
#pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depth_raw), intrinsics)
# flip it, otherwise the pcd will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

pcd.estimate_normals()
print("Displaying pointcloud with normals ...")
o3d.visualization.draw_geometries([pcd], point_show_normal=True)
print("Paint pointcloud")
pcd.paint_uniform_color([1, 0.706, 0])
o3d.visualization.draw([pcd])


# point_cloud_plane_segmentation.py
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
print("Displaying pointcloud with planar points in red ...")
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
o3d.visualization.draw([inlier_cloud, outlier_cloud])


axis_aligned_bounding_box = pcd.get_axis_aligned_bounding_box()
axis_aligned_bounding_box.color = (1, 0, 0)
oriented_bounding_box = pcd.get_oriented_bounding_box()
oriented_bounding_box.color = (0, 1, 0)
print(
    "Displaying axis_aligned_bounding_box in red and oriented bounding box in green ..."
)
o3d.visualization.draw(
    [pcd, axis_aligned_bounding_box, oriented_bounding_box])


# point_cloud_dbscan_clustering.py
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(pcd.cluster_dbscan(
        eps=0.02, min_points=10, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(
        labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw([pcd])

# point_cloud_farthest_point_sampling.py
pcd.paint_uniform_color([0.5, 0.5, 0.5])
# Get 1000 samples from original point cloud and paint to green.
pcd_down = pcd.farthest_point_down_sample(1000)
pcd_down.paint_uniform_color([0, 1, 0])

o3d.visualization.draw_geometries([pcd, pcd_down], point_show_normal=True)
