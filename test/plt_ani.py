import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from roboticstoolbox import DHRobot, RevoluteDH

# Define the robot arm using DH parameters
dh_params = [
    RevoluteDH(d=0.1, a=0, alpha=np.pi/2),
    RevoluteDH(d=0, a=0.5, alpha=0),
    RevoluteDH(d=0, a=0.5, alpha=0),
    RevoluteDH(d=0, a=0, alpha=np.pi/2),
]
robot = DHRobot(dh_params)

# Create a figure and axes for the animation
fig, ax = plt.subplots()
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)

# Define the initial state of the robot arm
q0 = np.array([0, np.pi/4, np.pi/2, np.pi/6])

# Define a function to update the animation at each time step
def update(frame):
    q = q0 + np.array([np.sin(frame/100), np.cos(frame/100), 0, 0])
    robot.plot(q, fig=fig)
    #ax.set_title(f"Frame {frame}")
    return ax.collections

# Create the animation
anim = FuncAnimation(fig, update, frames=np.arange(0, 1000), interval=10, blit=True, repeat=False)

# Show the animation
plt.show()
