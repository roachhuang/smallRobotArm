import open3d as o3d
import numpy as np

# Load the object point cloud or mesh
object_mesh = o3d.io.read_triangle_mesh("object_mesh.ply")

# Estimate the object's normal direction
object_mesh.compute_vertex_normals()

# Estimate the object's center point
center_point = object_mesh.get_center()

# Create an oriented bounding box around the object
obb = o3d.geometry.OrientedBoundingBox.create_from_points(object_mesh.vertices)
# set the color of the bounding box to red for visualization
obb.color = (1, 0, 0)

# Compute the object's transformation matrix
T = obb.get_transform()

# Transform the object to the world frame
object_mesh.transform(T)

# Visualize the transformed object and bounding box
o3d.visualization.draw_geometries([object_mesh, obb])
