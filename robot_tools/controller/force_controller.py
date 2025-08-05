"""Force Controller Module

This module provides force control functionality for the small robot arm,
including impedance control, compliant motion, and force-guided manipulation.

Classes:
    ForceController: Force-based control methods
"""

import time
import numpy as np
from numpy import ndarray
from .base_controller import BaseController


class ForceController(BaseController):
    """Force controller for the small robot arm.
    
    This class provides force control methods including impedance control,
    compliant motion, and force-guided manipulation tasks.
    """
    
    def __init__(self, robot):
        super().__init__(robot)
        
        # Force control parameters
        self.force_sensor_available = False  # Set to True when force sensor added
        self.max_force = 10.0  # Maximum allowed force (N)
        self.force_threshold = 2.0  # Contact detection threshold (N)
        
        # Impedance control parameters
        self.K_p = np.diag([1000, 1000, 1000, 100, 100, 100])  # Position stiffness
        self.K_d = np.diag([50, 50, 50, 10, 10, 10])  # Damping
        self.M_d = np.diag([1, 1, 1, 0.1, 0.1, 0.1])  # Desired inertia
        
        # Compliant motion parameters
        self.compliance_gain = 0.1  # How much to yield to forces
        self.force_filter_alpha = 0.3  # Force measurement filtering
        self._force_filtered = np.zeros(6)
    
    def read_force_sensor(self):
        """Read force/torque from sensor.
        
        Returns:
            6D force/torque vector [Fx, Fy, Fz, Mx, My, Mz]
        """
        if not self.force_sensor_available:
            # Simulate force reading or return zeros
            return np.zeros(6)
        
        # TODO: Implement actual force sensor reading
        # Example: return self.force_sensor.read()
        return np.zeros(6)
    
    def filter_force(self, force_raw):
        """Apply low-pass filter to force measurements."""
        self._force_filtered = (self.force_filter_alpha * force_raw + 
                               (1 - self.force_filter_alpha) * self._force_filtered)
        return self._force_filtered
    
    def impedance_control(self, x_desired, x_dot_desired, x_ddot_desired=None):
        """Impedance control for compliant motion.
        
        Args:
            x_desired: Desired Cartesian pose [x, y, z, rx, ry, rz]
            x_dot_desired: Desired Cartesian velocity
            x_ddot_desired: Desired Cartesian acceleration (optional)
            
        Returns:
            Joint torques for impedance behavior
        """
        # Get current pose and velocity
        q_current = np.radians(self.current_angles)
        T_current = self.robot.fk(np.degrees(q_current))
        x_current = self.robot.T2Pose(T_current)
        
        # Read external forces
        f_external = self.filter_force(self.read_force_sensor())
        
        # Calculate position and velocity errors
        x_error = np.array(x_desired) - np.array(x_current)
        # x_dot_current would need velocity estimation
        x_dot_current = np.zeros(6)  # Placeholder
        x_dot_error = x_dot_desired - x_dot_current
        
        # Impedance force calculation
        if x_ddot_desired is None:
            x_ddot_desired = np.zeros(6)
            
        f_desired = (self.M_d @ x_ddot_desired + 
                    self.K_d @ x_dot_error + 
                    self.K_p @ x_error)
        
        # Accommodate external forces
        f_command = f_desired - f_external
        
        # Convert to joint torques using Jacobian transpose
        J = self.robot.compute_jacobian(q_current)
        tau = J.T @ f_command
        
        return tau
    
    def compliant_motion(self, target_pose, max_force=None):
        """Move to target with force compliance.
        
        Args:
            target_pose: Target Cartesian pose
            max_force: Maximum force before stopping
        """
        if max_force is None:
            max_force = self.max_force
            
        # Move towards target while monitoring forces
        while not self._at_target_pose(target_pose):
            force = self.filter_force(self.read_force_sensor())
            force_magnitude = np.linalg.norm(force[:3])  # Linear forces only
            
            if force_magnitude > max_force:
                print(f"Force limit exceeded: {force_magnitude:.2f}N > {max_force}N")
                break
            
            # Calculate compliant motion
            compliance_offset = self.compliance_gain * force[:3]  # Only position compliance
            adjusted_target = np.array(target_pose)
            adjusted_target[:3] += compliance_offset
            
            # Move with impedance control
            tau = self.impedance_control(adjusted_target, np.zeros(6))
            self._apply_joint_torques(tau)
            
            time.sleep(self.dt)
    
    def force_guided_insertion(self, insertion_direction, target_force=5.0):
        """Force-guided insertion task.
        
        Args:
            insertion_direction: 3D unit vector for insertion direction
            target_force: Desired contact force during insertion
        """
        insertion_dir = np.array(insertion_direction) / np.linalg.norm(insertion_direction)
        
        # Phase 1: Approach until contact
        print("Phase 1: Approaching until contact...")
        contact_detected = False
        
        while not contact_detected:
            force = self.filter_force(self.read_force_sensor())
            force_magnitude = np.linalg.norm(force[:3])
            
            if force_magnitude > self.force_threshold:
                contact_detected = True
                print(f"Contact detected: {force_magnitude:.2f}N")
                break
            
            # Move slowly in insertion direction
            velocity = 5.0 * insertion_dir  # 5 mm/s
            self._move_with_velocity(velocity)
            time.sleep(self.dt)
        
        # Phase 2: Force-controlled insertion
        print(f"Phase 2: Force-controlled insertion at {target_force}N...")
        
        for _ in range(int(5.0 / self.dt)):  # 5 second insertion
            force = self.filter_force(self.read_force_sensor())
            force_in_direction = np.dot(force[:3], insertion_dir)
            
            # Force control law
            force_error = target_force - force_in_direction
            velocity_magnitude = 2.0 * force_error  # Simple proportional control
            velocity = velocity_magnitude * insertion_dir
            
            self._move_with_velocity(velocity)
            time.sleep(self.dt)
    
    def compliant_grasp(self, target_pose, grasp_force=3.0):
        """Perform compliant grasping with force feedback.
        
        Args:
            target_pose: Grasp pose
            grasp_force: Desired grasp force
        """
        # Move to pre-grasp pose
        pre_grasp = np.array(target_pose)
        pre_grasp[2] += 50  # 50mm above target
        self.move_to_pose(pre_grasp)
        
        # Compliant approach
        print("Compliant approach to object...")
        self.compliant_motion(target_pose, max_force=grasp_force)
        
        # Force-controlled grasping
        print(f"Grasping with {grasp_force}N force...")
        self._close_gripper_with_force_control(grasp_force)
    
    def _at_target_pose(self, target_pose, tolerance=5.0):
        """Check if at target pose within tolerance."""
        q_current = np.radians(self.current_angles)
        T_current = self.robot.fk(np.degrees(q_current))
        current_pose = self.robot.T2Pose(T_current)
        
        position_error = np.linalg.norm(np.array(target_pose[:3]) - np.array(current_pose[:3]))
        return position_error < tolerance
    
    def _apply_joint_torques(self, tau):
        """Apply joint torques (placeholder for torque control)."""
        # Convert torques to position commands (simplified)
        # In real implementation, this would send torque commands to motors
        q_current = np.radians(self.current_angles)
        
        # Simple torque-to-position conversion (placeholder)
        dt_torque = 0.001  # Small time step for torque integration
        q_new = q_current + dt_torque * tau / 100  # Simplified dynamics
        
        self.move_to_angles(np.degrees(q_new))
    
    def _move_with_velocity(self, cartesian_velocity):
        """Move with specified Cartesian velocity."""
        q_current = np.radians(self.current_angles)
        J = self.robot.compute_jacobian(q_current)
        
        # Convert Cartesian velocity to joint velocity
        try:
            q_dot = np.linalg.pinv(J) @ np.concatenate([cartesian_velocity, [0, 0, 0]])
            q_new = q_current + q_dot * self.dt
            self.move_to_angles(np.degrees(q_new))
        except np.linalg.LinAlgError:
            print("Jacobian singular, skipping velocity command")
    
    def _close_gripper_with_force_control(self, target_force):
        """Close gripper with force control."""
        # Placeholder for force-controlled gripper closing
        # In real implementation, monitor gripper force and stop when target reached
        print(f"Closing gripper with {target_force}N force control...")
        self.grab()  # Simple on/off for now
    
    def set_compliance_parameters(self, stiffness=None, damping=None):
        """Set impedance control parameters.
        
        Args:
            stiffness: 6x6 stiffness matrix or 6D diagonal values
            damping: 6x6 damping matrix or 6D diagonal values
        """
        if stiffness is not None:
            if np.isscalar(stiffness) or len(stiffness) == 6:
                self.K_p = np.diag(stiffness)
            else:
                self.K_p = np.array(stiffness)
        
        if damping is not None:
            if np.isscalar(damping) or len(damping) == 6:
                self.K_d = np.diag(damping)
            else:
                self.K_d = np.array(damping)