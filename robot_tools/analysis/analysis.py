"""Kinematics Analysis Module

This module provides analysis tools for robot kinematics including
eigenvalue analysis and approach pose computation.
"""

import numpy as np
from spatialmath import SE3

def eigen_analysis(robot, q):
    """Perform eigenvalue analysis of robot Jacobian and inertia matrices."""
    J = robot.jacob0(q)
    M = robot.inertia(q)
    
    # SVD for kinematics
    U, S, Vt = np.linalg.svd(J)
    
    # Eigen-decomposition for dynamics
    eigvals_M, eigvecs_M = np.linalg.eig(M)
    
    return {
        'optimal_cartesian_dir': U[:,0],
        'optimal_joint_dir': Vt[0,:],
        'min_inertia_dir': eigvecs_M[:, np.argmin(eigvals_M)],
        'manipulability': np.prod(S)
    }

def compute_approach_pose(T_cup, approach_vec_cup, offset=50):
    """Compute an approach pose with a specified offset from the target."""
    R_cup = T_cup[0:3, 0:3]
    p_cup = T_cup[0:3, 3]

    approach_vec_base = R_cup @ approach_vec_cup
    p_tcp = p_cup + offset * approach_vec_base
    z_tcp = -approach_vec_base

    world_up = np.array([0, 0, 1])
    x_tcp = np.cross(world_up, z_tcp)
    if np.linalg.norm(x_tcp) < 1e-3:
        world_up = np.array([0, 1, 0])
        x_tcp = np.cross(world_up, z_tcp)
    x_tcp /= np.linalg.norm(x_tcp)
    y_tcp = np.cross(z_tcp, x_tcp)

    R_tcp = np.column_stack((x_tcp, y_tcp, z_tcp))
    T_tcp = np.eye(4)
    T_tcp[0:3, 0:3] = R_tcp
    T_tcp[0:3, 3] = p_tcp
    
    return T_tcp