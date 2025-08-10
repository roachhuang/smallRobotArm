"""Velocity Controller Module

This module provides velocity control functionality for the small robot arm,
including joint-space and Cartesian-space velocity control with smoothing.

Classes:
    VelocityController: Velocity controller for the robot arm
"""

import time
from typing import Callable, Dict, List
import numpy as np
from numpy import ndarray

# from robot_tools.trajectory.path_optimizer import PathOptimizer
from .base_controller import BaseController

class VelocityController(BaseController):
    """Velocity controller for the small robot arm.
    
    This class provides velocity control methods including joint-space and
    Cartesian-space velocity control with advanced smoothing and singularity handling.
    """
    
    def __init__(self, robot):
        super().__init__(robot)
        self.joint_limits: Dict[str, List[float]] = {
        'vel': [np.radians(30)] * 6,  # Max velocity: 25 deg/s per joint
        'acc': [2.0] * 6,             # Max acceleration: 2.0 rad/s²
        'jerk': [10] * 6,             # Max jerk: 10 rad/s³ (higher = snappier motion)
        }
    
        # Joint Velocity Controller parameters
        self.max_q_dot_rad = np.radians(30)*6  # Max joint velocity
        # self.max_q_dot_rad = np.radians(30) * 6  # Max joint velocity
        self._q_dot_prev = np.zeros(6)  # Internal state for smoothing
        
        # End-Effector Velocity Controller parameters
        self.dls_epsilon = 0.01  # Damping factor for DLS
        self.manipulability_threshold = 0.01  # Minimum manipulability threshold
    
    def _smooth_q_dot(self, q_dot_raw: ndarray):
        """Smooth joint velocity using time-optimal scaling and adaptive EMA.

        Args:
            q_dot_raw (ndarray): Raw joint velocities in radians/s.

        Returns:
            ndarray: Smoothed joint velocities in radians/s.
        """
        
        # Numerical safety: handle NaN or invalid inputs
        if np.any(np.isnan(q_dot_raw)) or np.any(np.isnan(self.max_q_dot_rad)):
            return self._q_dot_prev  # Return previous velocity to avoid crashes

        # Time-optimal scaling: scale velocity if any joint exceeds limit
        max_ratios = np.abs(q_dot_raw / np.atleast_1d(self.max_q_dot_rad))
        scaling = 1.0 / np.max(max_ratios) if np.any(max_ratios > 1.0) else 1.0
        q_dot_scaled = q_dot_raw * scaling

        # Adaptive smoothing: higher speeds -> lower alpha -> more smoothing
        max_q_dot = np.max(np.atleast_1d(self.max_q_dot_rad))
        speed_ratio = np.linalg.norm(q_dot_scaled) / max_q_dot if max_q_dot > 1e-6 else 0.0\
        # NEW: Sigmoid fn
        alpha = np.clip(0.5 / (1 + np.exp(5 * (speed_ratio - 0.5))), 0.1, 0.9)
        alpha_scaled = 1 - (1 - alpha) ** (self.dt / 0.005)  # Adjust for dt

        # Exponential smoothing (EMA filter)
        q_dot = alpha_scaled * q_dot_scaled + (1 - alpha_scaled) * self._q_dot_prev
        self._q_dot_prev = q_dot
        return q_dot
        
    
    def joint_space_vel_ctrl(self, q_dot_func: Callable[[float], ndarray], duration: float):
        """Apply joint velocity control.

        Args:
            q_dot_func (callable): Function that returns 6D joint velocity at time t.
            duration (float): Duration to run control.
        """
        q_rad = np.radians(self.current_angles)
        n_steps = int(duration / self.dt)
        self._q_dot_prev = np.zeros(6)
        start_time = time.perf_counter()
        
        for i in range(n_steps):
            t = i * self.dt
            try:
                q_dot_raw = q_dot_func(t)            
                q_dot = self._smooth_q_dot(q_dot_raw)
                
                # Integrate velocity to get new joint positions
                q_rad += q_dot * self.dt           
                
                self.move_to_angles(np.degrees(q_rad), header='g', ack=True)

                # Time sync
                next_time = start_time + (i + 1) * self.dt
                time.sleep(max(0, next_time - time.perf_counter()))
            except Exception as e:
                print(f"Error in joint space velocity control at step {i}: {e}")
                break    
    
    def cartesian_space_vel_ctrl(self, x_dot_func: Callable[[float], ndarray], duration: float):
        """Velocity control with proper error handling and smoothing.
        
        Args:
            x_dot_func (callable): Function returning 6D velocity vector in (mm/s, rad/s)
            duration (float): Duration to run control
        """
        # analyzer = OptimalMotionAnalyzer(self.robot, self.max_q_dot_rad)
        
        n_steps = int(duration / self.dt)
        self._q_dot_prev = np.zeros(6)
        start_time = time.perf_counter()
        
        for i in range(n_steps):
            t = i * self.dt
            try:
                # close loop 
                q_rad = np.radians(self.current_angles)
                J = self.robot.jacob0(q_rad)
                
                # Damped Least Squares Inverse for singularity avoidance
                JT = J.T
                U, S, Vt = np.linalg.svd(J)
                svd_data = {'U': U, 'S': S}
               
                '''
                manipulability = np.prod(S)
                if manipulability < self.manipulability_threshold:
                    # Add null-space motion or increase damping
                    print(f"Manipulability Measure: {manipulability:.4f}")
                
                # 3. Principal Directions of Manipulability (U matrix columns)
                # U[:, 0] is the direction of easiest motion (largest singular value)
                # U[:, -1] is the direction of hardest motion (smallest singular value)
                print(f"Direction of Easiest Motion (Task Space): {U[:, 0]}")
                print(f"Direction of Hardest Motion (Task Space): {U[:, -1]}")
                '''
                sigma_min = np.min(S)
                epsilon = self.dls_epsilon
                lambda_sq = (epsilon**2) if sigma_min > epsilon else (epsilon**2 + (1 - (sigma_min / epsilon)**2))
                J_dls = JT @ np.linalg.inv(J @ JT + lambda_sq * np.eye(6))
                
                x_dot = x_dot_func(t)
                # Option 1: Use optimizer (recommended), but this will reshpae circular motion!!!
                # q_dot_raw = analyzer.optimize_cartesian_velocity(x_dot, svd_data=svd_data)
                # Option 2: Use DLS (original method)
                q_dot_raw = J_dls @ x_dot
                q_dot = self._smooth_q_dot(q_dot_raw)
                q_rad += q_dot * self.dt           
                self.move_to_angles(np.degrees(q_rad), header='g', ack=True)
                
                # Fixed time synchronization
                next_time = start_time + (i + 1) * self.dt
                time.sleep(max(0, next_time - time.perf_counter()))
            except Exception as e:
                print(f"Error in cartesian space velocity control at step {i}: {e}")
                break
            
    def get_optimal_directions(self, U, Vt):
        return {
            'easiest_cartesian_motion': U[:, 0],      # Direction requiring least joint effort
            'hardest_cartesian_motion': U[:, -1],     # Direction requiring most joint effort
            'most_effective_joint': Vt[0, :],        # Joint combination with max end-effector effect
            'least_effective_joint': Vt[-1, :]       # Joint combination with min end-effector effect
        }
        
    def force_analysis(self, desired_force, q):
        J = self.robot.compute_jacobian(q)
        U, S, Vt = np.linalg.svd(J)
        
        # Required joint torques for desired end-effector force
        tau_required = J.T @ desired_force
        
        # Force amplification in each direction
        force_amplification = 1.0 / S  # Higher singular values = easier force generation
        
        return {
            'joint_torques': tau_required,
            'force_difficulty': force_amplification,
            'hardest_force_direction': U[:, -1]  # Direction hardest to generate force
        }

    def adaptive_control_gains(self, S):       
        # Higher gains for well-conditioned directions
        position_gains = S * 100  # Scale with singular values
        velocity_gains = np.sqrt(S) * 20
        
        return np.diag(position_gains), np.diag(velocity_gains)

    def analyze_singularities(self, S, Vt):
        # Detect near-singularities
        condition_number = S[0] / S[-1]
        if condition_number > 100:
            return "Near singularity - avoid this pose"
        
        # Find null space (lost DOF directions)
        null_directions = Vt[S < 0.01]  # Joint directions that don't move end-effector
        return null_directions
