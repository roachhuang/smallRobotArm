"""Path Optimization Module

Path optimizer for robot trajectory planning with singularity avoidance.
"""

import numpy as np
from numpy import ndarray
from typing import List


class PathOptimizer:
    """Path optimizer for robot trajectory planning."""
    
    def __init__(self, robot):
        self.robot = robot
    
    def eigen_analysis(self, q: ndarray) -> dict:
        """Perform eigenvalue analysis of robot Jacobian and inertia matrices."""
        J = self.robot.jacob0(q)
        M = self.robot.inertia(q)
        
        U, S, Vt = np.linalg.svd(J)
        eigvals_M, eigvecs_M = np.linalg.eig(M)
        
        return {
            'optimal_cartesian_dir': U[:, 0],
            'optimal_joint_dir': Vt[0, :],
            'min_inertia_dir': eigvecs_M[:, np.argmin(eigvals_M)],
            'manipulability': np.prod(S),
            'singular_values': S
        }
    
    def adjust_for_manipulability(self, q_rad: ndarray, min_manipulability: float = 0.1) -> ndarray:
        """Adjust joint angles to improve manipulability near singularities."""
        best_q = q_rad.copy()
        best_manip = self.eigen_analysis(q_rad)['manipulability']
        
        if best_manip > min_manipulability:
            return best_q
            
        for joint_idx in [1, 2, 4]:  # joints 2,3,5 (0-indexed)
            for delta in [-0.1, 0.1]:  # ±5.7 degrees
                q_test = q_rad.copy()
                q_test[joint_idx] += delta
                manip = self.eigen_analysis(q_test)['manipulability']
                if manip > best_manip:
                    best_manip = manip
                    best_q = q_test
        return best_q
    
    def optimize_manipulability_path(self, waypoints_rad: ndarray, 
                                   min_manipulability: float = 0.1) -> ndarray:
        """Optimize joint path to avoid singularities."""
        optimized_path = []
        
        for i, waypoint in enumerate(waypoints_rad):
            analysis = self.eigen_analysis(waypoint)
            manip = analysis['manipulability']
            
            if manip < min_manipulability:
                print(f"Waypoint {i}: Low manipulability {manip:.3f}, optimizing...")
                optimized_waypoint = self.adjust_for_manipulability(waypoint, min_manipulability)
                new_manip = self.eigen_analysis(optimized_waypoint)['manipulability']
                print(f"  Improved from {manip:.3f} to {new_manip:.3f}")
                optimized_path.append(optimized_waypoint)
            else:
                optimized_path.append(waypoint)
                
        return np.array(optimized_path)
    
    def align_path_to_vector(self, original_path: List[ndarray], 
                           easy_direction: ndarray, strength: float = 0.7) -> List[ndarray]:
        """Adjust path to favor movement in the energy-efficient direction."""
        adjusted_path = []
        easy_direction = easy_direction / np.linalg.norm(easy_direction)

        N = len(original_path)
        for i, point in enumerate(original_path):
            if i == 0:
                adjusted_path.append(point)
                continue
                
            progress = i / (N - 1) if N > 1 else 0.0
            adjustment_factor = strength * np.exp(-10 * (progress - 0.5)**2)
            delta = original_path[i] - original_path[i - 1]
            magnitude = np.linalg.norm(delta)
            adjusted_point = point + adjustment_factor * easy_direction * magnitude
            adjusted_path.append(adjusted_point)

        return adjusted_path