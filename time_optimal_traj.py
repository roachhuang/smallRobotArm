"""Time-Optimal Trajectory Planning Module

This module demonstrates time-optimal trajectory planning for a 6-DOF robot arm.
It converts Cartesian waypoints to joint space, optimizes for manipulability,
and executes velocity-controlled motion with timing constraints.

Features:
    - Cartesian to joint space trajectory conversion
    - Manipulability-based path optimization
    - Time-optimal velocity profile generation
    - Real-time trajectory execution with velocity control

Example:
    python time_optimal_traj.py

Author: Robot Control Team
Date: 2024
"""

from time import sleep
import logging
import numpy as np
import sys
from typing import List, Tuple, Dict, Any

from roboticstoolbox import DHRobot   
from robot_tools.kinematics import SmallRbtArm, std_dh_params, std_dh_tbl
from robot_tools.controller import VelocityController
from robot_tools.misc.signal_handler import setup_signal_handler
import matplotlib.pyplot as plt
from robot_tools.trajectory.path_optimizer import PathOptimizer

def main() -> None:
    """Main function for time-optimal trajectory planning demonstration.
    Executes a complete trajectory planning pipeline:
    1. Initialize robot and controller
    2. Define Cartesian waypoints with timestamps
    3. Convert to joint space trajectories
    4. Optimize path for manipulability
    5. Generate velocity profiles
    6. Execute motion with real-time control
    
    Returns:
        None
        
    Raises:
        SystemExit: On completion or error
    """
    # Configure logging and numpy display
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(precision=2, suppress=True)
  
    # Initialize robot models and controller
    smRobot = DHRobot(std_dh_tbl, name="smallRobotArm")
    smallRobotArm = SmallRbtArm(std_dh_params)
    controller = VelocityController(smRobot)
    path_optimizer = PathOptimizer(smRobot)
    
    # Setup signal handling for graceful shutdown
    setup_signal_handler(controller)                    
   
    # Hardware initialization sequence
    sleep(1)  # Required delay after signal initialization
    controller.enable()  # Enable robot hardware
    sleep(1)
    
    # Define trajectory timing and waypoints
    timestamps: List[float] = [0, 8, 10, 24]  # Time points in seconds
    dt_list: List[float] = [timestamps[i] - timestamps[i-1] 
                           for i in range(1, len(timestamps))]  # [8, 12, 4] seconds
    
    # Cartesian waypoints: [x, y, z, roll, pitch, yaw] in desk coordinate frame
    # Units: mm for position, degrees for orientation
    cartesian_path: np.ndarray = np.array([
        (-200, 100, 60, 0.0, 0, 35.0),    # Start position
        (-200, 100, 90, 0.0, 0, 35.0),    # Lift up
        (-295, 170, 350, 0, -60, 0.0),    # Move and rotate
        (-295, 270, 350, 0, -60, 0.0),    # Final position
    ], dtype=np.float64)

    # Move to initial position
    initial_pose = tuple(cartesian_path[0])
    T_06_initial: np.ndarray = smallRobotArm.convert_p_dc_to_T06(initial_pose)
    initial_joints_deg = smallRobotArm.ik(T_06_initial)    
    controller.move_to_angles(initial_joints_deg)
    input("Press Enter to start time-optimal trajectory execution...")
    
    # Convert Cartesian waypoints to joint space
    joint_path_deg: List[Tuple[float, ...]] = [
        smallRobotArm.ik(smallRobotArm.convert_p_dc_to_T06(tuple(pose))) 
        for pose in cartesian_path
    ]
    joint_path_rad = np.array([np.radians(joints) for joints in joint_path_deg])
    
    # Optimize trajectory for manipulability (avoid singularities)
    print("\nOptimizing trajectory for manipulability...")
   
    joint_path_optimized = path_optimizer.optimize_manipulability_path(
        joint_path_rad, min_manipulability=0.05
    )
    
    # Display optimization results
    print("Manipulability improvement:")
    for i, (original, optimized) in enumerate(zip(joint_path_rad, joint_path_optimized)):
        orig_manip: float = path_optimizer.eigen_analysis(original)['manipulability']
        opt_manip: float = path_optimizer.eigen_analysis(optimized)['manipulability']
        print(f"  Waypoint {i}: {orig_manip:.4f} → {opt_manip:.4f}")
    
    joint_path_final = joint_path_optimized
    
    # 3. Generate velocity profiles (rad/s) for each trajectory segment
    segment_velocities: List[np.ndarray] = []
    for i in range(1, len(joint_path_final)):
        joint_displacement: np.ndarray = joint_path_final[i] - joint_path_final[i-1]
        segment_duration: float = dt_list[i-1]
        segment_velocity: np.ndarray = joint_displacement / segment_duration
        segment_velocities.append(segment_velocity)
    
    velocity_profiles = np.array(segment_velocities)  # Shape: (n_segments, 6)
    
    # Display trajectory information
    print(f"\nTrajectory Summary:")
    print(f"  Timestamps: {timestamps}")
    print(f"  Segment durations: {dt_list}")
    print(f"  Velocity profiles shape: {velocity_profiles.shape}")
    print(f"  Max joint velocities: {np.max(np.abs(velocity_profiles), axis=0)}")
    
    # Visualize velocity profiles
    _plot_velocity_profiles(velocity_profiles)
    
    # Execute trajectory with simple velocity profiles (waypoints already optimized)
    print("\nExecuting time-optimal trajectory...")
    for seg_idx, seg_duration in enumerate(dt_list):
        segment_velocity = velocity_profiles[seg_idx]  # rad/s (simple constant velocity)
        print(f" Segment {seg_idx+1}: max(|vel|)={np.max(np.abs(segment_velocity)):.3f} rad/s for {seg_duration:.2f}s")
        velocity_func = lambda t, vel=segment_velocity: vel
        controller.joint_space_vel_ctrl(q_dot_func=velocity_func, duration=seg_duration)
        
    print("Trajectory execution completed.")
    controller.go_home()

def _plot_velocity_profiles(velocity_profiles: np.ndarray) -> None:
    """Plot joint velocity profiles for visualization.
    
    Args:
        velocity_profiles: Array of shape (n_segments, 6) containing
                         velocity values for each joint in each segment
    """
    plt.figure(figsize=(10, 6))
    plt.plot(velocity_profiles)
    plt.title("Joint Velocity Profiles")
    plt.xlabel("Trajectory Segment")
    plt.ylabel("Velocity (rad/s)")
    plt.legend([f'Joint {i+1}' for i in range(6)])
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()
    
if __name__ == "__main__":
    sys.exit(main())

