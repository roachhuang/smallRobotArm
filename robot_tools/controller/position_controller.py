"""Position Controller Module

This module provides position control functionality for the small robot arm,
including point-to-point motion, trajectory following, and path optimization.

Classes:
    PositionController: Position controller for the robot arm
"""

import time
import numpy as np
from numpy import ndarray
from .base_controller import BaseController


class PositionController(BaseController):
    """Position controller for the small robot arm.
    
    This class provides position control methods including point-to-point motion,
    trajectory following, and path optimization with manipulability considerations.
    """
    
    def __init__(self, robot):
        super().__init__(robot)
        
        # Position controller parameters
        self.position_tolerance = 0.1  # degrees
        self.max_step_size = 5.0  # degrees per step
    
    def move_to_pose(self, target_pose, optimize_path=True):
        """Move to a target Cartesian pose.
        
        Args:
            target_pose: 6D pose [x, y, z, rx, ry, rz]
            optimize_path: Whether to optimize the path for manipulability
        """
        # Convert pose to transformation matrix
        T_target = self.robot.convert_p_dc_to_T06(target_pose)
        
        # Solve inverse kinematics
        target_angles = self.robot.ik(T_target)
        
        # Move to joint angles
        self.move_to_joint_angles(target_angles, optimize_path=optimize_path)
    
    def move_to_joint_angles(self, target_angles, optimize_path=True):
        """Move to target joint angles with optional path optimization.
        
        Args:
            target_angles: Target joint angles in degrees
            optimize_path: Whether to optimize the path for manipulability
        """
        # Calculate path
        path = self._generate_joint_path(self.current_angles, target_angles)
        
        if optimize_path:
            # Convert to radians for optimization
            path_rad = [np.radians(angles) for angles in path]
            
            # Optimize for manipulability
            path_rad_opt = self.optimize_manipulability_path(path_rad, min_manipulability=0.05)
            
            # Optimize for energy efficiency
            if len(path_rad_opt) > 0:
                M = self.robot.inertia(path_rad_opt[-1])
                eigvals, eigvecs = np.linalg.eig(M)
                easy_direction = eigvecs[:, np.argmin(eigvals)]
                path_rad_final = self.align_path_to_vector(path_rad_opt, easy_direction, strength=0.3)
            else:
                path_rad_final = path_rad_opt
            
            # Convert back to degrees
            path = [np.degrees(angles) for angles in path_rad_final]
        
        # Execute path
        for angles in path:
            self.move_to_angles(angles, header='g', ack=True)
    
    def _generate_joint_path(self, start_angles, target_angles, max_steps=None):
        """Generate a linear interpolated path between joint angles."""
        start = np.array(start_angles)
        target = np.array(target_angles)
        
        # Calculate required steps based on maximum joint movement
        diff = np.abs(target - start)
        max_diff = np.max(diff)
        
        if max_steps is None:
            n_steps = max(5, int(max_diff / self.max_step_size))
        else:
            n_steps = max_steps
        
        # Generate linear interpolation
        path = []
        for i in range(n_steps + 1):
            alpha = i / n_steps
            interpolated = start + alpha * (target - start)
            path.append(tuple(interpolated))
        
        return path
    
    def follow_trajectory(self, waypoints, optimize_path=True):
        """Follow a trajectory defined by multiple waypoints.
        
        Args:
            waypoints: List of 6D poses [x, y, z, rx, ry, rz]
            optimize_path: Whether to optimize each segment for manipulability
        """
        for i, waypoint in enumerate(waypoints):
            print(f"Moving to waypoint {i+1}/{len(waypoints)}")
            self.move_to_pose(waypoint, optimize_path=optimize_path)
    
    def follow_joint_trajectory(self, joint_waypoints, optimize_path=True):
        """Follow a trajectory defined by joint angle waypoints.
        
        Args:
            joint_waypoints: List of joint angle tuples in degrees
            optimize_path: Whether to optimize each segment for manipulability
        """
        for i, waypoint in enumerate(joint_waypoints):
            print(f"Moving to joint waypoint {i+1}/{len(joint_waypoints)}")
            self.move_to_joint_angles(waypoint, optimize_path=optimize_path)
    
    def is_at_target(self, target_angles):
        """Check if robot is at target position within tolerance."""
        current = np.array(self.current_angles)
        target = np.array(target_angles)
        error = np.abs(target - current)
        return np.all(error < self.position_tolerance)