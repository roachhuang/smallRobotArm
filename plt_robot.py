from roboticstoolbox.backends.PyPlot import PyPlot
import numpy as np
from roboticstoolbox import DHRobot, SerialLink
from roboticstoolbox.tools import mesh_points

# Define the DH table
a = [0, 0.5, 0.3, 0.1]
alpha = [0, np.pi/2, 0, np.pi/2]
d = [0.1, 0, 0, 0.2]
theta = [0, 0, 0, 0]
dh_table = np.array([a, alpha, d, theta]).T

# Create a robot model using the DH table
robot = SerialLink(dh_table)

# Define the bounds of the workspace to plot
workspace_limits = np.array([[-0.5, 0.5], [-0.5, 0.5], [0, 0.5]])

# Define the resolution of the workspace grid
res = 0.05

# Compute the workspace
points = mesh_points(workspace_limits, res)
in_workspace = robot.ets(pts=points).all(axis=1)

# Create a PyPlot backend and plot the workspace points
plot = PyPlot()
plot.plot(points[in_workspace, 0], points[in_workspace, 1],
          points[in_workspace, 2], '.', alpha=0.2)
plot.show()
