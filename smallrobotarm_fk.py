
"""
    plz give me code in py for the robot arm's ik and fk.
    note the input orientation for ik is in euler zyz angles.
    also verify the ik and fk to make sure they are correct.
    here are its standard dh params: a1=47, a2=110,a3=26, a4=0,a5=0,a6=0.
    d1=133,d2=0,d3=0,d4=117.5,d5=0,d6=28.
    alpha1=-90,alph2=0,alph3=-90,alph4=90,alph5=-90, alph6=0.
    theta1=theta1,theta2=-90+theta2,theta3=theta3,theta4=theta4,theta5=theta5,theta6=theta6.
"""
import numpy as np
from math import sin, cos, atan2, acos, sqrt

# Define the DH parameters
a = [47, 110, 26, 0, 0, 0]
d = [133, 0, 0, 117.5, 0, 28]
alpha = [-90, 0, -90, 90, -90, 0]

# Define the end-effector pose
x, y, z, alpha, beta, gamma = 164.5, 0.0, 141.0, 90.0, 180.0, -90.0

# Convert Euler ZYZ angles to rotation matrix
alpha, beta, gamma = np.radians(alpha), np.radians(beta), np.radians(gamma)
R_z1 = np.array([[cos(alpha), -sin(alpha), 0],
                [sin(alpha), cos(alpha), 0], [0, 0, 1]])
R_y = np.array([[cos(beta), 0, sin(beta)], [
               0, 1, 0], [-sin(beta), 0, cos(beta)]])
R_z2 = np.array([[cos(gamma), -sin(gamma), 0],
                [sin(gamma), cos(gamma), 0], [0, 0, 1]])
R = R_z1.dot(R_y.dot(R_z2))

# Compute the wrist position
P_wrist = np.array([x, y, z]) - d[5] * R[:, 2]

# Compute joint 1 angle
theta1 = atan2(P_wrist[1], P_wrist[0])

# Compute joint 3 angle
D_sq = P_wrist[0]**2 + P_wrist[1]**2 - a[0]**2 - (P_wrist[2] - d[0])**2
theta3 = - \
    acos(D_sq / (2*a[1]*(sqrt(D_sq**2 + 4*a[1]**2*(P_wrist[2] - d[0])**2))))

# Compute joint 2 angle
K1 = a[0] + a[1]*cos(theta3)
K2 = a[1]*sin(theta3)
theta2 = atan2(P_wrist[2] - d[0], sqrt(P_wrist[0] **
               2 + P_wrist[1]**2)) - atan2(K2, K1)

# Compute the rotation matrix from base to wrist
R_0_3 = np.array([[cos(theta1)*cos(theta2+theta3), -sin(theta1), cos(theta1)*sin(theta2+theta3)],
                  [sin(theta1)*cos(theta2+theta3), cos(theta1),
                   sin(theta1)*sin(theta2+theta3)],
                  [-sin(theta2+theta3), 0, cos(theta2+theta3)]])

R_3_6 = np.linalg.inv(R_0_3.dot(R))

# Compute joint 5 angle
theta5 = acos(R_3_6[2, 2])

# Compute joint 4 and 6 angles
if sin(theta5) != 0:
    theta4 = atan2(R_3_6[1, 2], R_3_6[0, 2])
    theta6 = atan2(R_3_6[2, 1], -R_3_6[2, 0])
else:
    theta4 = atan2(R_3_6[0, 1], R_3
