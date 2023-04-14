import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from roboticstoolbox import DHRobot, RevoluteDH

# Define Denavit-Hartenberg parameters for two-link robot arm
L1 = RevoluteDH(d=0.1, a=0, alpha=np.pi/2)
L2 = RevoluteDH(d=0, a=0.5, alpha=0)
robot = DHRobot([L1, L2])

# Define initial joint angles and joint angle increments
q0 = np.zeros(2)
dq = np.array([0.01, 0.01])

# Create figure and axis for animation
fig, ax = plt.subplots()
# Set axis limits
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
# Define update function


def update(frame, robot, q, ax):
    # Calculate new joint angles based on increment
    q = q0 + frame*dq

    # Clear previous plot
    # ax.clear()

    # Plot robot arm with new joint angles
    robot.plot(q=q)

    # Return plot objects to be redrawn
    return ax.collections


# Create animation
animation = FuncAnimation(fig, update, fargs=(
    robot, q0, ax), frames=range(50), interval=50, blit=True)

# Show plot
plt.show()
