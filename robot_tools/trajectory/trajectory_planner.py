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
    
    def interpolate_poses(self, start_pose: SE3, end_pose: SE3, num_poses: int = 5) -> List[ndarray]:
        """Interpolate between two SE3 poses."""
        s_values = np.linspace(0, 1, num_poses + 2)[1:-1]
        se3_poses = [start_pose.interp(end_pose, s) for s in s_values]
        return [pose.A for pose in se3_poses]
    
    def generate_linear_path_in_js(self, start: ndarray, end: ndarray, steps: int) -> List[ndarray]:
        """Generate joint-space straight-line path from start to end."""
        start = np.array(start)
        end = np.array(end)
        return [start + (end - start) * t / (steps - 1) for t in range(steps)]
    
    def generate_linear_pose_path(self, start_pose: SE3, end_pose: SE3, steps: int) -> List[SE3]:
        """Generate linear path between poses in Cartesian space."""
        return [start_pose.interp(end_pose, s / (steps - 1)) for s in range(steps)]
    
    def smooth_pose_path(self, path: List[SE3], alpha: float = 0.3) -> List[SE3]:
        """Smooth a pose path using low-pass filtering in SE3 space."""
        smoothed = [path[0]]
        for i in range(1, len(path)):
            smooth_pose = smoothed[-1].interp(path[i], alpha)
            smoothed.append(smooth_pose)
        return smoothed
    
    def plan_simple_path(self, start_angles: ndarray, goal_angles: ndarray, 
                        obstacles=None, steps: int = 20) -> List[ndarray]:
        """Simple path planning - straight line in joint space."""
        return self.generate_linear_path_in_js(start_angles, goal_angles, steps)


# Backward compatibility alias
Interp = TrajectoryPlanner