from math import radians
import numpy as np
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


def dh_transform(alpha, a, d, theta):
    """
    Calculates the DH transformation matrix for a single joint using Craig's convention.

    :param alpha: alpha angle in degrees
    :param a: a distance in millimeters
    :param d: d distance in millimeters
    :param theta: theta angle in degrees
    :return: the DH transformation matrix
    """
    alpha = np.radians(alpha)
    theta = np.radians(theta)
    ct = np.cos(np.radians(theta))
    st = np.sin(np.radians(theta))
    ca = np.cos(np.radians(alpha))
    sa = np.sin(np.radians(alpha))

    T = np.array([[ct, -st*ca, st*sa, a*ct],
                  [st, ct*ca, -ct*sa, a*st],
                  [0, sa, ca, d],
                  [0, 0, 0, 1]])

    return T[0:3, 0:3]


def forward_kinematics(theta):
    """
    Calculates the forward kinematics for a given set of joint angles using Craig's convention.

    :param theta: a list of joint angles in degrees
    :return: the end-effector position and orientation in the base frame
    """
    T_01 = dh_transform(theta[0], 0, 0, 90)
    T_12 = dh_transform(theta[1], 0, 340, -60)
    T_23 = dh_transform(theta[2], 0, 0, 0)
    T_34 = dh_transform(theta[3], 338, 0, -90)
    T_45 = dh_transform(theta[4], 0, 0, 90)
    T_56 = dh_transform(theta[5], 0, 0, -90)
    T = T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56
    return T[:3, 3], T[:3, :3]


def zyz_euler_angles(R):
    """
    Calculates the ZYZ Euler angles for a given rotation matrix.

    :param R: a 3x3 rotation matrix
    :return: the ZYZ Euler angles in degrees
    """
    alpha = np.arctan2(R[2, 1], R[2, 0])
    beta = np.arccos(R[2, 2])
    gamma = np.arctan2(R[1, 2], -R[0, 2])
    return np.degrees([alpha, beta, gamma])


# Example usage
'''
theta = [0, 0, 0, 0, 0, 0]
p, R = forward_kinematics(theta)
print("End-effector position:", p)
print("End-effector orientation:")
print(R)
print("ZYZ Euler angles:", zyz_euler_angles(R))
'''

def inverse_kinematics(p_des, R_des):
    """
    Solves for the joint angles that achieve a desired end-effector position and orientation using Euler ZYZ angles.

    :param p_des: the desired end-effector position in the base frame
    :param R_des: the desired end-effector orientation as a 3x3 rotation matrix in the base frame
    :return: a list of joint angles in degrees that achieve the desired end-effector pose
    """
    d6 = 338  # length of the 6th link
    theta1 = np.arctan2(p_des[1], p_des[0])
    r = np.sqrt(p_des[0] ** 2 + p_des[1] ** 2)
    s = p_des[2] - d6
    D = (r ** 2 + s ** 2 - 340 ** 2 - 338 ** 2) / (2 * 340 * 338)
    theta3 = np.arctan2(np.sqrt(1 - D ** 2), D)
    alpha = np.arctan2(s, r)
    beta = np.arctan2(338 * np.sin(theta3), 340 + 338 * np.cos(theta3))
    gamma = np.arctan2(R_des[2, 0], -R_des[2, 1])
    theta2 = alpha - beta
    theta4 = euler_zyz(R_des @ dh_transform(0, 0, 0, -
                       (theta1 + theta2))[0:3, 0:3])[1]
    theta5 = euler_zyz(R_des @ dh_transform(0, 0, 0, -
                       (theta1 + theta2 + theta4))[0:3, 0:3])[1]
    theta6 = gamma - euler_zyz(R_des @ dh_transform(0, 0, -d6, 0))[0]
    return [np.degrees(theta1), np.degrees(theta2), np.degrees(theta3), np.degrees(theta4), np.degrees(theta5), np.degrees(theta6)]


# example of usage

# Define the desired end-effector pose (position and orientation)
p_des = np.array([0.227, 0.372, 0.1886])  # position in meters
# orientation as a 3x3 rotation matrix
R_des = R.from_euler('xyz', [0, radians(-30), np.pi]).as_matrix()

# Compute the joint angles required to achieve the desired end-effector pose
theta = inverse_kinematics(p_des, R_des)

# Print the computed joint angles in degrees
print("Joint angles (degrees):", theta)
