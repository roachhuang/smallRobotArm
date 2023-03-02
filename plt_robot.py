import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from roboticstoolbox import DHRobot, RevoluteDH

a1, a2, a3 = 47.0, 110.0, 26.0
d1, d4, d6 = 133.0, 117.50, 28.0

dh_table = [
    RevoluteDH(d=d1, a=a1, alpha=-np.pi/2),    # joint 1
    RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi/2),             # joint 2
    RevoluteDH(d=0, a=a3, alpha=-np.pi/2),      # joint 3
    RevoluteDH(d=d4, a=0, alpha=np.pi/2),       # joint 4
    RevoluteDH(d=0, a=0, alpha=-np.pi/2),       # joint 5
    RevoluteDH(d=d6, a=0, alpha=0)              # joint 6
]

robot = DHRobot(dh_table)
fig, ax = plt.subplots()
# define the joint angles for a sample pose
# q = np.array([0, np.radians(-78.51), np.radians(73.9), 0, -np.pi/2, 0])

# plot the robot in the specified pose
# robot.plot(q, backend="pyplot", fig=fig)


def update(frame):
    # example of joint angle update
    q = frame / 100 * np.array([np.pi/2, np.pi/4, np.pi/2, 0, 0, 0])
    robot.plot(q, fig=fig, backend="pyplot")
    return ax.collections


ani = FuncAnimation(fig, update, frames=np.arange(100))
plt.show()

