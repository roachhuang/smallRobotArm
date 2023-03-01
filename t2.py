#from spatialmath import DHRobot
#from spatialmath import SE3
from roboticstoolbox import DHRobot, SE3
import numpy as np

# Define the desired end-effector pose
T_desired = SE3.Tx(0.5) * SE3.Ty(-0.2) * SE3.Rz(np.pi/2)
T_desired.t = [0.1, 0.2, 0.3]  # Set the desired position

# Define the robot DH parameters
a = [0, 0.4318, 0.0203, 0, 0, 0]
alpha = [-np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0]
d = [0.1273, 0, 0.1501, 0.4318, 0, 0.066]
theta = [0, 0, 0, 0, 0, 0]

# Define the robot model
robot = DHRobot(
    alpha=alpha,
    a=a,
    d=d,
    theta=theta,
    model='R'
)

# Compute the inverse kinematics
q_desired = robot.ikine(T_desired)

# Compute the forward kinematics
T = robot.fkine(q_desired)
print("Forward Kinematics:\n", T)
print("Inverse Kinematics:\n", q_desired)
