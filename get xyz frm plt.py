import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Create a simple 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
x = np.random.randn(100)
y = np.random.randn(100)
z = np.random.randn(100)
scatter = ax.scatter(x, y, z)

# Wait for the user to click a point and get its x, y, and z coordinates
print("Click on a point to get its x, y, and z coordinates:")
x_coord, y_coord, z_coord = plt.ginput(1)[0][:3]
#z_coord = scatter.get_zdata()[np.argmin(np.abs(scatter.get_offsets() - [x_coord, y_coord]).sum(axis=1))]
print(f"x={x_coord:.2f}, y={y_coord:.2f}, z={z_coord:.2f}")

plt.show()
