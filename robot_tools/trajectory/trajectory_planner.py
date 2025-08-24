"""Trajectory Planner Module

Main trajectory planning class with interpolation and path generation.
"""

import numpy as np
from numpy import ndarray
from spatialmath import SE3
from typing import List, Callable


class TrajectoryPlanner:
    """Main trajectory planning class for interpolation and path generation."""
    
    def __init__(self, robot=None):
        self.robot = robot
    
    def generate_linear_path_in_js(self, start: ndarray, end: ndarray, steps: int) -> List[ndarray]:
        """Generate joint-space straight-line path from start to end."""
        start = np.array(start)
        end = np.array(end)
        if steps < 2:
            raise ValueError("Number of steps must be at least 2")
        return [start + (end - start) * i / (steps - 1) for i in range(steps)]
    
    def generate_linear_path_in_cs(self, start_pose: SE3, end_pose: SE3, steps: int) -> List[SE3]:
        """Generate linear path between poses in Cartesian space."""
        return [start_pose.interp(end_pose, i / (steps - 1)) for i in range(steps)]
    
    def interpolate_poses(self, start_pose: SE3, end_pose: SE3, num_poses: int = 5) -> List[ndarray]:
        """
        Interpolate poses and return as 4x4 matrices (backward compatibility).
        
        """
        poses = self.generate_linear_path_in_cs(start_pose, end_pose, num_poses + 2)[1:-1]
        return [pose.A for pose in poses]
    
    def smooth_pose_path(self, path: List[SE3], alpha: float = 0.3) -> List[SE3]:
        """Smooth poses in cartesian space using low-pass filtering in SE3 space.
        Takes List[SE3] as input (SE3 = 4x4 transformation matrices)
        """
        smoothed = [path[0]]
        for i in range(1, len(path)):
            smooth_pose = smoothed[-1].interp(path[i], alpha)
            smoothed.append(smooth_pose)
        return smoothed
    
# Backward compatibility alias
Interp = TrajectoryPlanner