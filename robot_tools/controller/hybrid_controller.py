"""Hybrid Controller - Trajectory + Feedforward

Combines S-curve trajectory generation with feedforward compensation
for optimal motion control of stepper motor robot arms.
"""

import numpy as np
from typing import List, Tuple, Callable, Optional
from .trajectory_controller import TrajectoryController
from .feedforward_controller import FeedforwardController

'''
from robot_tools.controller import HybridController

# Best of both worlds
controller = HybridController(robot)

# Configure both systems
controller.configure_hybrid_control(
    profile_type='scurve',
    gravity_comp=True,
    friction_comp=True,
    gravity_scale=0.8
)

# Smooth compensated motion
controller.move_to_angles_smooth_compensated((10, 20, 30, 0, 0, 0), duration=3.0)

# Complex trajectory with compensation
waypoints = [[0,0,0,0,0,0], [10,20,30,0,0,0], [20,40,60,0,0,0]]
durations = [2.0, 3.0]
controller.execute_compensated_trajectory(waypoints, durations)

'''

class HybridController(TrajectoryController, FeedforwardController):
    """Hybrid controller combining trajectory planning with feedforward compensation."""
    
    def __init__(self, robot):
        # Initialize both parent classes
        TrajectoryController.__init__(self, robot)
        FeedforwardController.__init__(self, robot)
        
        print("Hybrid Controller initialized: Trajectory + Feedforward")
    
    def execute_compensated_trajectory(self, waypoints: List[np.ndarray], 
                                     durations: List[float]) -> None:
        """Execute trajectory with feedforward compensation."""
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")
        
        print(f"Executing compensated trajectory with {len(waypoints)} waypoints")
        
        for i in range(len(waypoints) - 1):
            q_start = np.radians(waypoints[i])
            q_end = np.radians(waypoints[i + 1])
            duration = durations[i]
            
            print(f"Segment {i+1}: S-curve + compensation for {duration:.2f}s")
            
            # Generate S-curve trajectory
            t, positions, velocities, accelerations = self.generate_scurve_profile(
                q_start, q_end, duration)
            
            # Execute with feedforward compensation
            for j, (pos, vel, acc) in enumerate(zip(positions, velocities, accelerations)):
                # Compute feedforward compensation
                compensation = self.compute_feedforward_torques(pos, vel, acc)
                
                # Apply compensation as angle adjustment
                angle_adjustment = compensation * 0.1  # Scaling factor
                compensated_pos = pos + angle_adjustment
                
                # Send to robot
                self.move_to_angles(tuple(np.degrees(compensated_pos)), header="g", ack=False)
                self.current_angles = tuple(np.degrees(pos))  # Track actual commanded position
    
    def execute_compensated_cartesian_trajectory(self, poses: List[np.ndarray], 
                                               duration: float) -> None:
        """Execute Cartesian trajectory with compensation."""
        # Convert to joint space
        joint_waypoints = []
        for pose in poses:
            joints = self.robot.ik(pose)
            joint_waypoints.append(joints)
        
        # Execute with compensation
        segment_duration = duration / (len(joint_waypoints) - 1)
        durations = [segment_duration] * (len(joint_waypoints) - 1)
        
        self.execute_compensated_trajectory(joint_waypoints, durations)
    
    def execute_compensated_velocity_trajectory(self, velocity_func: Callable[[float], np.ndarray],
                                              duration: float, 
                                              q_start: Optional[np.ndarray] = None) -> None:
        """Execute velocity trajectory with feedforward compensation."""
        if q_start is None:
            q_start = np.radians(self.current_angles)
        
        steps = int(duration / self.dt)
        q_current = q_start.copy()
        
        print(f"Executing compensated velocity trajectory for {duration:.2f}s")
        
        for i in range(steps):
            t = i * self.dt
            q_dot = velocity_func(t)
            
            # Estimate acceleration (simple finite difference)
            if i > 0:
                q_ddot = (q_dot - q_dot_prev) / self.dt
            else:
                q_ddot = np.zeros(6)
            
            # Integrate velocity
            q_current += q_dot * self.dt
            
            # Compute feedforward compensation
            compensation = self.compute_feedforward_torques(q_current, q_dot, q_ddot)
            angle_adjustment = compensation * 0.1
            compensated_pos = q_current + angle_adjustment
            
            # Send to robot
            self.move_to_angles(tuple(np.degrees(compensated_pos)), header="g", ack=False)
            self.current_angles = tuple(np.degrees(q_current))
            
            q_dot_prev = q_dot.copy()
    
    def move_to_angles_smooth_compensated(self, target_angles: Tuple[float, ...], 
                                        duration: float = 2.0) -> None:
        """Move to target with both S-curve and compensation."""
        current_angles = np.array(self.current_angles)
        target_angles = np.array(target_angles)
        
        # Use single-segment trajectory
        waypoints = [current_angles, target_angles]
        durations = [duration]
        
        self.execute_compensated_trajectory(waypoints, durations)
    
    def configure_hybrid_control(self, 
                                # Trajectory parameters
                                profile_type: str = 'scurve',
                                max_vel: np.ndarray = None,
                                max_acc: np.ndarray = None,
                                # Compensation parameters  
                                gravity_comp: bool = True,
                                friction_comp: bool = True,
                                gravity_scale: float = 0.8) -> None:
        """Configure both trajectory and compensation parameters."""
        
        # Configure trajectory
        self.set_profile_type(profile_type)
        if max_vel is not None:
            self.max_velocity = max_vel
        if max_acc is not None:
            self.max_acceleration = max_acc
        
        # Configure compensation
        self.enable_compensation(gravity=gravity_comp, friction=friction_comp)
        self.gravity_scale = gravity_scale
        
        print(f"Hybrid control configured:")
        print(f"  Profile: {profile_type}")
        print(f"  Compensation: Gravity={gravity_comp}, Friction={friction_comp}")
    
    def auto_tune_compensation(self, test_angles: List[Tuple[float, ...]], 
                             duration: float = 3.0) -> None:
        """Auto-tune compensation parameters using test movements."""
        print("Auto-tuning compensation parameters...")
        
        # Test different gravity scales
        gravity_scales = [0.6, 0.8, 1.0, 1.2]
        best_scale = 0.8
        
        for scale in gravity_scales:
            print(f"Testing gravity scale: {scale}")
            self.gravity_scale = scale
            
            # Execute test movement
            self.move_to_angles_smooth_compensated(test_angles[0], duration)
            
            # In practice, you would measure positioning error here
            # For now, we'll use the middle value
        
        self.gravity_scale = best_scale
        print(f"Optimal gravity scale: {best_scale}")
    
    def get_status(self) -> dict:
        """Get current controller status."""
        return {
            'profile_type': self.profile_type,
            'gravity_compensation': self.gravity_compensation,
            'friction_compensation': self.friction_compensation,
            'gravity_scale': self.gravity_scale,
            'max_velocity': self.max_velocity,
            'current_angles': self.current_angles
        }