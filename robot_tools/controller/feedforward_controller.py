"""Feedforward Controller for Open-Loop Stepper Motor Robot Arm

Compensates for known dynamics without requiring feedback sensors.
Suitable for stepper motor systems with predictable behavior.
"""

import numpy as np
from typing import Tuple, Optional
from .base_controller import BaseController


class FeedforwardController(BaseController):
    """Feedforward controller for open-loop stepper motor robot arm."""
    
    def __init__(self, robot):
        super().__init__(robot)
        
        # Compensation parameters (tune for your robot)
        self.gravity_compensation = True
        self.friction_compensation = True
        self.inertia_compensation = False  # Usually not needed for steppers
        
        # Friction model parameters (Coulomb + viscous)
        self.coulomb_friction = np.array([0.1, 0.15, 0.1, 0.05, 0.05, 0.02])  # Nm
        self.viscous_friction = np.array([0.01, 0.02, 0.01, 0.005, 0.005, 0.002])  # Nm·s/rad
        
        # Gravity compensation scaling factor
        self.gravity_scale = 0.8  # Reduce if overcompensating
    
    def compensate_gravity(self, q_rad: np.ndarray) -> np.ndarray:
        """Compute gravity compensation torques."""
        if not self.gravity_compensation:
            return np.zeros(6)
        
        try:
            # Use robot model to compute gravity torques
            gravity_torques = self.robot.gravload(q_rad)
            return gravity_torques * self.gravity_scale
        except:
            # Fallback: simplified gravity model for vertical joints
            g = 9.81  # m/s²
            compensation = np.zeros(6)
            
            # Approximate gravity effects on major joints (joints 1,2,3)
            compensation[1] = -2.0 * np.sin(q_rad[1])  # Shoulder pitch
            compensation[2] = -1.0 * np.sin(q_rad[2])  # Elbow
            
            return compensation
    
    def compensate_friction(self, q_dot_rad: np.ndarray) -> np.ndarray:
        """Compute friction compensation torques."""
        if not self.friction_compensation:
            return np.zeros(6)
        
        # Coulomb friction (direction-dependent constant friction)
        coulomb = self.coulomb_friction * np.sign(q_dot_rad)
        
        # Viscous friction (velocity-proportional)
        viscous = self.viscous_friction * q_dot_rad
        
        return coulomb + viscous
    
    def compensate_inertia(self, q_rad: np.ndarray, q_ddot_rad: np.ndarray) -> np.ndarray:
        """Compute inertia compensation torques."""
        if not self.inertia_compensation:
            return np.zeros(6)
        
        try:
            # Full inertia matrix compensation
            M = self.robot.inertia(q_rad)
            return M @ q_ddot_rad
        except:
            # Simplified inertia compensation for major joints
            inertia_approx = np.array([0.5, 1.0, 0.3, 0.1, 0.1, 0.05])  # kg·m²
            return inertia_approx * q_ddot_rad
    
    def compute_feedforward_torques(self, q_rad: np.ndarray, 
                                  q_dot_rad: Optional[np.ndarray] = None,
                                  q_ddot_rad: Optional[np.ndarray] = None) -> np.ndarray:
        """Compute total feedforward compensation torques."""
        if q_dot_rad is None:
            q_dot_rad = np.zeros(6)
        if q_ddot_rad is None:
            q_ddot_rad = np.zeros(6)
        
        # Sum all compensation terms
        gravity_comp = self.compensate_gravity(q_rad)
        friction_comp = self.compensate_friction(q_dot_rad)
        inertia_comp = self.compensate_inertia(q_rad, q_ddot_rad)
        
        total_compensation = gravity_comp + friction_comp + inertia_comp
        
        return total_compensation
    
    def move_to_angles_with_compensation(self, target_angles: Tuple[float, ...], 
                                       duration: float = 2.0) -> None:
        """Move to target angles with feedforward compensation."""
        target_rad = np.radians(target_angles)
        current_rad = np.radians(self.current_angles)
        
        # Simple trajectory: linear interpolation with smooth velocity profile
        steps = int(duration / self.dt)
        t = np.linspace(0, 1, steps)
        
        # S-curve velocity profile for smooth motion
        s = 3*t**2 - 2*t**3  # Smooth interpolation
        s_dot = 6*t - 6*t**2  # Velocity profile
        s_ddot = 6 - 12*t     # Acceleration profile
        
        for i, (pos, vel, acc) in enumerate(zip(s, s_dot, s_ddot)):
            # Interpolated position, velocity, acceleration
            q = current_rad + pos * (target_rad - current_rad)
            q_dot = (vel / duration) * (target_rad - current_rad)
            q_ddot = (acc / duration**2) * (target_rad - current_rad)
            
            # Compute feedforward compensation
            compensation = self.compute_feedforward_torques(q, q_dot, q_ddot)
            
            # Convert compensation to angle adjustments (simplified)
            # In practice, this would be sent to motor drivers as current/torque commands
            angle_adjustment = compensation * 0.1  # Scaling factor (tune this)
            compensated_angles = np.degrees(q + angle_adjustment)
            
            # Send to robot
            self.move_to_angles(tuple(compensated_angles), header="g", ack=False)
            
            # Update current position
            self.current_angles = tuple(np.degrees(q))
    
    def set_compensation_parameters(self, gravity_scale: float = None,
                                  coulomb_friction: np.ndarray = None,
                                  viscous_friction: np.ndarray = None) -> None:
        """Update compensation parameters for tuning."""
        if gravity_scale is not None:
            self.gravity_scale = gravity_scale
        if coulomb_friction is not None:
            self.coulomb_friction = coulomb_friction
        if viscous_friction is not None:
            self.viscous_friction = viscous_friction
    
    def enable_compensation(self, gravity: bool = True, friction: bool = True, 
                          inertia: bool = False) -> None:
        """Enable/disable specific compensation types."""
        self.gravity_compensation = gravity
        self.friction_compensation = friction
        self.inertia_compensation = inertia
        
        print(f"Compensation enabled - Gravity: {gravity}, Friction: {friction}, Inertia: {inertia}")