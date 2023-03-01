import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

# Define your DH table parameters
a1, a2, a3, a6 = 47.0, 110.0, 26.0, 50
d1, d4, d6 = 133.0, 117.50, 28.0

dh_table = [
    RevoluteDH(d=d1, a=a1, alpha=-np.pi/2),    # joint 1
    RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi/2),             # joint 2
    RevoluteDH(d=0, a=a3, alpha=-np.pi/2),      # joint 3
    RevoluteDH(d=d4, a=0, alpha=np.pi/2),       # joint 4
    RevoluteDH(d=0, a=0, alpha=-np.pi/2),       # joint 5
    RevoluteDH(d=d6, a=a6, alpha=0)              # joint 6
]

# Create the robot object with the DH parameters
robot = DHRobot(dh_table)

# Print the robot's kinematic model
print(robot)
"""
         ti     q1     q2     q3      q4      q5      q6
0   2.0  11.56  73.55 -24.84  180.00  138.72   58.44
1   2.0  11.56  97.43 -45.92  180.00  141.51   58.44
2   0.0  45.87  -5.92 -33.41  105.86   59.99 -119.22
3   0.0  59.29  19.31 -63.19   95.05   49.31 -116.00
4  21.0  -0.00 -79.45  71.98    0.00   97.47   90.00
"""
# Define the end-effector pose (position and orientation)
T = SE3(164.5,   0.0,    241.0) * \
    SE3.Rz(np.radians(90)) * SE3.Ry(180) * SE3.Rz(np.radians(-90))
# T = SE3(51.98, 0, 218.2) * SE3.Rz(0) * SE3.Ry(0) * SE3.Rz(np.pi)
q = robot.ikine_LM(T)
j = np.degrees(q.q)
print(j)

# folding pose
# j=np.array([21, 51.98, 0, 218.2-52, 90.0, 180.0, -90.0])
# j = np.array([11.56,   73.55,    -24.84,   180.00, 138.72, 58.44])
j = np.array([0, 0, 0, 0, 90, 0])
j = np.radians(j)
T = robot.fkine(q.q)
# Define desired end-effector pose
#T_desired = SE3.Tx(0.5) * SE3.Ty(-0.2) * SE3.Rz(np.pi/2)
# Compute inverse kinematics
# p = robot.fkine([0,-78.51,73.90,0,-90.0,0])

print(T)
