"""Optimal Motion Direction Module - Refactored

Efficient SVD-based motion optimization with caching and velocity constraints.
Integrates seamlessly with VelocityController and time-optimal trajectory planning.
"""

import numpy as np
from typing import Dict, Optional


class OptimalMotionAnalyzer:
    """Efficient SVD-based motion optimizer with caching and velocity limits."""
    
    def __init__(self, robot, controller=None, max_q_dot_rad=None):
        self.robot = robot
        self.controller = controller
        self.max_q_dot_rad = max_q_dot_rad or np.radians(30)
    
    def optimize_cartesian_velocity(self, desired_velocity: np.ndarray, 
                                  svd_data: Dict) -> np.ndarray:
        """Optimize Cartesian velocity using manipulability ellipsoid."""
        U, S = svd_data['U'], svd_data['S']
        
        # Transform to manipulability ellipsoid coordinates
        v_ellipsoid = U.T @ desired_velocity
        
        # Scale by inverse singular values (easier directions get higher weight)
        s_normalized = S / np.max(S)
        v_scaled = v_ellipsoid * s_normalized
        
        # Transform back to Cartesian space
        return U @ v_scaled
    
    def optimize_joint_velocity(self, desired_joint_velocity: np.ndarray,
                              q_rad: Optional[np.ndarray] = None,
                              svd_data: Optional[Dict] = None) -> np.ndarray:
        """Optimize joint velocity with singularity avoidance and limits."""
        if svd_data is None:
            if q_rad is None:
                q_rad = np.radians(self.controller.current_angles)
            svd_data = self._compute_svd(q_rad)
        
        Vt, S = svd_data['Vt'], svd_data['S']
        
        # Project onto most effective joint directions
        joint_effectiveness = np.sum(np.abs(Vt), axis=0)  # Sum across all task directions
        joint_effectiveness /= np.max(joint_effectiveness)
        
        # Singularity avoidance factor
        manip_factor = np.tanh(svd_data['manipulability'] * 100)  # Smooth scaling
        
        # Apply optimization
        q_dot_opt = desired_joint_velocity * joint_effectiveness * manip_factor
        
        return q_dot_opt
    



