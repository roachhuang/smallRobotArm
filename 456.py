import numpy as np
from math import pi
from scipy.spatial.transform import Rotation as R


def euler_zyz(R):
    """
    Computes the Euler ZYZ angles (in radians) from a 3x3 rotation matrix.

    :param R: a 3x3 rotation matrix
    :return: a tuple containing the Euler ZYZ angles (alpha, beta, gamma)
    """
    if abs(R[2, 2]) != 1:
        beta = np.arccos(R[2, 2])
        alpha = np.arctan2(R[1, 2], R[0, 2])
        gamma = np.arctan2(R[2, 1], -R[2, 0])
    else:
        if R[2, 2] == -1:
            beta = np.pi
            alpha = np.arctan2(-R[0, 1], -R[0, 0])
            gamma = 0
        else:
            beta = 0
            alpha = np.arctan2(R[1, 0], R[1, 1])
            gamma = 0
    return (alpha, beta, gamma)


# define the robot arm's DH table
dh_params = np.array([[0, 0, 0, -pi/2],
                      [-90, -30, 340, 0],
                      [0, 0, 0, -pi/2],
                      [-90, -40, 0, pi/2],
                      [90, 0, 0, -pi/2],
                      [-90, 0, 70, 0]])

# define the desired end-effector pose
R_des = R.from_euler('xyz', [-pi/2, pi/2, -pi/2], degrees=False)
p_des = np.array([350, 0, 350]).reshape((3, 1))

# solve for the joint angles using the Euler ZYZ angles method
d1 = dh_params[0, 2]
a2 = dh_params[1, 1]
a3 = dh_params[2, 1]
d4 = dh_params[3, 2]
d5 = dh_params[4, 2]
d6 = dh_params[5, 2]

# calculate the wrist center position
p_wc = p_des - R_des.apply(np.array([0, 0, d6]).reshape((3, 1)))

# calculate theta1
theta1 = np.arctan2(p_wc[1], p_wc[0])

# calculate theta3
l1 = np.sqrt(p_wc[0]**2 + p_wc[1]**2) - d1
l2 = p_wc[2] - a2
l3 = np.sqrt(l1**2 + l2**2)
alpha = np.arctan2(l2, l1)
beta = np.arccos((a3**2 - d4**2 - l3**2) / (-2 * d4 * l3))
theta3 = pi/2 - alpha - beta

# calculate theta2
phi1 = np.arctan2(l2, l1)
phi2 = np.arcsin(a3 * np.sin(beta) / l3)
theta2 = -(phi1 + phi2) + pi/2

# calculate the wrist rotation matrix
R_03 = dh_transform(0, 0, d1, theta1) @ \
    dh_transform(a2, -pi/2, 0, theta2) @ \
    dh_transform(a3, 0, 0, theta3)

R_36 = np.linalg.inv(R_des) @ R_03
gamma = np.arctan2(-R_36[1, 0], R_36[0, 0])

# calculate theta4
theta4 = euler_zyz(R_des @ dh_transform(0, 0, 0, -
                   (theta1 + theta2))[0:3, 0:3])[1]

# calculate theta5
R_46 = np.linalg.inv(dh_transform(0, 0, d4, theta4) @
                     dh_transform(0, -pi/2, 0, theta3) @ R_36)
theta5 = euler_zyz(R_46)[1]

# calculate theta6
theta6 = gamma - euler_zyz(R_des @ dh_transform(0, 0, -d6, 0))[0]

# print the joint angles
print(theta4, theta5, theta6)
