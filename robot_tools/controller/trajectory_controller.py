"""Trajectory Controller for Robot Arm

Executes smooth trajectories with coordinated multi-axis motion,
S-curve profiles, and time-optimal planning for stepper motors.
"""

import numpy as np
from typing import List, Tuple, Callable, Optional
from .base_controller import BaseController

'''
from robot_tools.controller import TrajectoryController

controller = TrajectoryController(robot)

# Multi-point trajectory
waypoints = [[0,0,0,0,0,0], [10,20,30,0,0,0], [20,40,60,0,0,0]]
durations = [2.0, 3.0]
controller.execute_trajectory(waypoints, durations)

# S-curve motion
controller.set_profile_type('scurve')

# Time-optimal planning
durations = controller.plan_time_optimal_trajectory(waypoints)
'''


class TrajectoryController(BaseController):
    """Advanced trajectory controller with S-curve profiles and coordination."""
    
    def __init__(self, robot):
        super().__init__(robot)
        
        # Trajectory parameters
        self.max_velocity = np.radians([30, 25, 35, 45, 45, 60])  # rad/s per joint
        self.max_acceleration = np.array([2.0, 1.5, 2.5, 3.0, 3.0, 4.0])  # rad/s²
        self.max_jerk = np.array([10, 8, 12, 15, 15, 20])  # rad/s³
        
        # Motion profile type
        self.profile_type = 'scurve'  # 'linear', 'trapezoidal', 'scurve'
    
    def generate_scurve_profile(self, q_start: np.ndarray, q_end: np.ndarray, 
                               duration: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Generate S-curve trajectory profile."""
        steps = int(duration / self.dt)
        t = np.linspace(0, duration, steps)
        
        # S-curve parameters (7-segment profile)
        T = duration
        profiles = []
        
        for i in range(6):  # For each joint
            displacement = q_end[i] - q_start[i]
            
            # Simple S-curve: smooth acceleration and deceleration
            # Using polynomial approach for simplicity
            s = np.zeros(steps)
            s_dot = np.zeros(steps)
            s_ddot = np.zeros(steps)
            
            for j, time in enumerate(t):
                tau = time / T  # Normalized time [0, 1]
                
                if tau <= 0.5:
                    # Acceleration phase
                    s[j] = 2 * tau**3
                    s_dot[j] = 6 * tau**2 / T
                    s_ddot[j] = 12 * tau / T**2
                else:
                    # Deceleration phase
                    tau_d = 1 - tau
                    s[j] = 1 - 2 * tau_d**3
                    s_dot[j] = 6 * tau_d**2 / T
                    s_ddot[j] = -12 * tau_d / T**2
            
            profiles.append((s, s_dot, s_ddot))
        
        # Combine profiles for all joints
        positions = np.zeros((steps, 6))
        velocities = np.zeros((steps, 6))
        accelerations = np.zeros((steps, 6))
        
        for i in range(6):
            s, s_dot, s_ddot = profiles[i]
            displacement = q_end[i] - q_start[i]
            
            positions[:, i] = q_start[i] + displacement * s
            velocities[:, i] = displacement * s_dot
            accelerations[:, i] = displacement * s_ddot
        
        return t, positions, velocities, accelerations
    
    def generate_trapezoidal_profile(self, q_start: np.ndarray, q_end: np.ndarray,
                                   max_vel: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Generate trapezoidal velocity profile."""
        displacement = q_end - q_start
        max_displacement = np.max(np.abs(displacement))
        
        # Calculate timing
        t_accel = max_vel / self.max_acceleration[0]
        t_total = max_displacement / max_vel + t_accel
        
        steps = int(t_total / self.dt)
        t = np.linspace(0, t_total, steps)
        
        positions = np.zeros((steps, 6))
        velocities = np.zeros((steps, 6))
        
        for i in range(steps):
            time = t[i]
            if time <= t_accel:
                # Acceleration phase
                progress = 0.5 * (time / t_accel)**2
                vel_factor = time / t_accel
            elif time <= t_total - t_accel:
                # Constant velocity phase
                progress = 0.5 + (time - t_accel) / (t_total - 2*t_accel)
                vel_factor = 1.0
            else:
                # Deceleration phase
                t_remain = t_total - time
                progress = 1.0 - 0.5 * (t_remain / t_accel)**2
                vel_factor = t_remain / t_accel
            
            positions[i] = q_start + displacement * progress
            velocities[i] = displacement * vel_factor / t_total
        
        return t, positions, velocities
    
    def execute_trajectory(self, waypoints: List[np.ndarray], durations: List[float]) -> None:
        """Execute multi-segment trajectory through waypoints."""
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")
        
        if len(durations) != len(waypoints) - 1:
            raise ValueError("Need duration for each segment")
        
        print(f"Executing trajectory with {len(waypoints)} waypoints")
        
        for i in range(len(waypoints) - 1):
            q_start = np.radians(waypoints[i])
            q_end = np.radians(waypoints[i + 1])
            duration = durations[i]
            
            print(f"Segment {i+1}/{len(waypoints)-1}: {duration:.2f}s")
            
            if self.profile_type == 'scurve':
                t, positions, velocities, accelerations = self.generate_scurve_profile(
                    q_start, q_end, duration)
                
                # Execute with smooth motion
                for j, pos in enumerate(positions):
                    self.move_to_angles(tuple(np.degrees(pos)), header="g", ack=False)
                    self.current_angles = tuple(np.degrees(pos))
                    
            elif self.profile_type == 'trapezoidal':
                t, positions, velocities = self.generate_trapezoidal_profile(
                    q_start, q_end, np.min(self.max_velocity))
                
                for j, pos in enumerate(positions):
                    self.move_to_angles(tuple(np.degrees(pos)), header="g", ack=False)
                    self.current_angles = tuple(np.degrees(pos))
            
            else:  # Linear
                steps = int(duration / self.dt)
                for j in range(steps):
                    alpha = j / (steps - 1)
                    pos = q_start + alpha * (q_end - q_start)
                    self.move_to_angles(tuple(np.degrees(pos)), header="g", ack=False)
                    self.current_angles = tuple(np.degrees(pos))
    
    def execute_cartesian_trajectory(self, poses: List[np.ndarray], duration: float) -> None:
        """Execute Cartesian space trajectory."""
        # Convert poses to joint space
        joint_waypoints = []
        
        for pose in poses:
            # Assuming pose is 6D: [x, y, z, rx, ry, rz]
            if hasattr(self.robot, 'ik'):
                joints = self.robot.ik(pose)
                joint_waypoints.append(joints)
            else:
                raise NotImplementedError("Robot must have ik() method for Cartesian trajectories")
        
        # Execute as joint space trajectory
        segment_duration = duration / (len(joint_waypoints) - 1)
        durations = [segment_duration] * (len(joint_waypoints) - 1)
        
        self.execute_trajectory(joint_waypoints, durations)
    
    def execute_velocity_trajectory(self, velocity_func: Callable[[float], np.ndarray], 
                                  duration: float, q_start: Optional[np.ndarray] = None) -> None:
        """Execute trajectory defined by velocity function."""
        if q_start is None:
            q_start = np.radians(self.current_angles)
        
        steps = int(duration / self.dt)
        q_current = q_start.copy()
        
        print(f"Executing velocity trajectory for {duration:.2f}s")
        
        for i in range(steps):
            t = i * self.dt
            q_dot = velocity_func(t)
            
            # Integrate velocity to get position
            q_current += q_dot * self.dt
            
            # Send to robot
            self.move_to_angles(tuple(np.degrees(q_current)), header="g", ack=False)
            self.current_angles = tuple(np.degrees(q_current))
    
    def set_motion_limits(self, max_vel: np.ndarray = None, max_acc: np.ndarray = None, 
                         max_jerk: np.ndarray = None) -> None:
        """Update motion limits for trajectory planning."""
        if max_vel is not None:
            self.max_velocity = max_vel
        if max_acc is not None:
            self.max_acceleration = max_acc
        if max_jerk is not None:
            self.max_jerk = max_jerk
    
    def set_profile_type(self, profile_type: str) -> None:
        """Set motion profile type: 'linear', 'trapezoidal', 'scurve'."""
        if profile_type not in ['linear', 'trapezoidal', 'scurve']:
            raise ValueError("Profile type must be 'linear', 'trapezoidal', or 'scurve'")
        self.profile_type = profile_type
        print(f"Motion profile set to: {profile_type}")
    
    def plan_time_optimal_trajectory(self, waypoints: List[np.ndarray]) -> List[float]:
        """Plan time-optimal trajectory durations between waypoints."""
        durations = []
        
        for i in range(len(waypoints) - 1):
            q_start = np.radians(waypoints[i])
            q_end = np.radians(waypoints[i + 1])
            
            # Calculate time based on maximum joint displacement and velocity limits
            joint_displacements = np.abs(q_end - q_start)
            joint_times = joint_displacements / self.max_velocity
            
            # Use the longest time (limiting joint)
            segment_duration = np.max(joint_times)
            durations.append(max(segment_duration, 0.5))  # Minimum 0.5s per segment
        
        return durations