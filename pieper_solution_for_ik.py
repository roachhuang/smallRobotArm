import numpy as np
from scipy.spatial.transform import Rotation as R

def inverse_kinematics_pieper(T, dh_params):
    """
    Computes the inverse kinematics using Pieper's method for a 6-DOF robotic arm.
    Assumes the last three joints are intersecting at a common point.
    
    :param T: 4x4 numpy array, desired end-effector transformation matrix
    :param dh_params: List of tuples [(theta, d, a, alpha), ...] in standard DH convention
    :return: Possible joint angles solutions
    """
    # Extract the wrist center position
    px, py, pz = T[:3, 3]
    
    # Solve for first three joint angles (position)
    theta1, theta2, theta3 = solve_position(px, py, pz, dh_params)
    
    # Compute the wrist orientation matrix
    R06 = T[:3, :3]
    R03 = compute_forward_rotation(theta1, theta2, theta3, dh_params)
    R36 = np.linalg.inv(R03) @ R06
    
    # Solve for last three joint angles (orientation)
    theta4, theta5, theta6 = solve_orientation(R36)
    
    return theta1, theta2, theta3, theta4, theta5, theta6

def solve_position(px, py, pz, dh_params):
    """Solves for the first three joint angles based on the wrist center position."""
    # Extract DH parameters
    a1, d1 = dh_params[0][2], dh_params[0][1]
    a2, d3 = dh_params[1][2], dh_params[2][1]
    
    # Compute theta1
    theta1 = np.arctan2(py, px)
    
    # Compute theta2 and theta3 using geometric approach
    r = np.sqrt(px**2 + py**2) - a1
    s = pz - d1
    D = (r**2 + s**2 - a2**2 - d3**2) / (2 * a2 * d3)
    
    if np.abs(D) > 1:
        raise ValueError("No real solution for theta3.")
    
    theta3 = np.arctan2(np.sqrt(1 - D**2), D)
    theta2 = np.arctan2(s, r) - np.arctan2(d3 * np.sin(theta3), a2 + d3 * np.cos(theta3))
    
    return theta1, theta2, theta3

def compute_forward_rotation(theta1, theta2, theta3, dh_params):
    """Computes the forward rotation matrix up to the third joint."""
    # Compute transformation matrices
    T01 = transformation_matrix(theta1, *dh_params[0])
    T12 = transformation_matrix(theta2, *dh_params[1])
    T23 = transformation_matrix(theta3, *dh_params[2])
    
    T03 = T01 @ T12 @ T23
    return T03[:3, :3]

def solve_orientation(R36):
    """Solves for the last three joint angles from the wrist orientation matrix."""
    theta5 = np.arccos(R36[2, 2])
    
    if np.sin(theta5) != 0:
        theta4 = np.arctan2(R36[1, 2], R36[0, 2])
        theta6 = np.arctan2(R36[2, 1], -R36[2, 0])
    else:
        theta4 = 0
        theta6 = np.arctan2(-R36[0, 1], R36[1, 1])
    
    return theta4, theta5, theta6

def transformation_matrix(theta, d, a, alpha):
    """Computes a transformation matrix using DH parameters."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])