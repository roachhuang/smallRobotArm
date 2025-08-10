"""Velocity Optimizer Module

SVD-based velocity optimization for smooth robot motion.
"""

import numpy as np
from numpy import ndarray
from typing import Dict


class VelOptimizer:
    """SVD-based motion optimizer for velocity optimization."""
    
    def __init__(self, robot, controller=None, max_q_dot_rad=None):
        self.robot = robot
        self.controller = controller
    
    def optimize_cartesian_velocity(self, xdot: ndarray, svd_data: Dict) -> ndarray:
        """Optimize Cartesian velocity using manipulability ellipsoid."""
        U, S = svd_data['U'], svd_data['S']
        
        v_ellipsoid = U.T @ xdot
        s_normalized = S / np.max(S)
        v_scaled = v_ellipsoid * s_normalized
        
        return U @ v_scaled
    
    def optimize_joint_velocity(self, qdot: ndarray, svd_data: Dict) -> ndarray:
        """Optimize joint velocity with singularity avoidance."""
        Vt, S = svd_data['Vt'], svd_data['S']
        
        joint_effectiveness = np.sum(np.abs(Vt), axis=0)
        joint_effectiveness /= np.max(joint_effectiveness)
        
        manipulability = np.prod(S)
        manip_factor = np.tanh(manipulability * 100)
        
        return qdot * joint_effectiveness * manip_factor