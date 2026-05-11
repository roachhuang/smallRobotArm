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
from typing import List, Tuple, Dict, Any, Callable

from roboticstoolbox import DHRobot   
from robot_tools.kinematics import SmallRbtArm, std_dh_params, std_dh_tbl
from robot_tools.controller import VelocityController
from robot_tools.misc.signal_handler import setup_signal_handler
import matplotlib.pyplot as plt
from robot_tools.trajectory.path_optimizer import PathOptimizer


def _quintic_scurve_velocity_profile(
    q_start: np.ndarray,
    q_end: np.ndarray,
    duration: float,
) -> Callable[[float], np.ndarray]:
    """Return a smooth joint-velocity profile for one segment.

    Uses the quintic smoothstep:
        s(u) = 10u^3 - 15u^4 + 6u^5
    so the segment starts/ends with zero velocity and zero acceleration.
    """
    if duration <= 0:
        raise ValueError("Segment duration must be positive.")

    displacement = q_end - q_start

    def velocity_func(t: float) -> np.ndarray:
        u = np.clip(t / duration, 0.0, 1.0)
        s_dot = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / duration
        return displacement * s_dot

    return velocity_func


def _sample_velocity_profile(
    velocity_func: Callable[[float], np.ndarray],
    duration: float,
    dt: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Sample a velocity callback for visualization."""
    n_steps = max(2, int(np.ceil(duration / dt)) + 1)
    t = np.linspace(0.0, duration, n_steps)
    q_dot = np.array([velocity_func(ti) for ti in t])
    return t, q_dot

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
    timestamps: List[float] = [0, 2, 20, 24]  # Time points in seconds
    # timestamps: List[float] = [0, 8, 10, 24]  # Time points in seconds
    dt_list: List[float] = [timestamps[i] - timestamps[i-1] 
                           for i in range(1, len(timestamps))]  # [8, 12, 4] seconds
    
    # Cartesian waypoints: [x, y, z, roll, pitch, yaw] in desk coordinate frame
    # Units: mm for position, degrees for orientation
    # cartesian_path: np.ndarray = np.array([
    #     (-200, 100, 60, 0.0, 0, 35.0),    # Start position
    #     (-200, 100, 90, 0.0, 0, 35.0),    # Lift up
    #     (-295, 170, 350, 0, -60, 0.0),    # Move and rotate
    #     (-295, 270, 350, 0, -60, 0.0),    # Final position
    # ], dtype=np.float64)

    cartesian_path = np.array(
        [
            (-200, 160, 60, 0.0, 0.0, 35.0),
            (-200, 160, 90, 0.0, 0.0, 35.0),
            (-295, 270.0, 350.0, 0.0, -60.0, 0.0),
            (-295, 350.0, 350.0, 0.0, -60.0, 0.0),
        ],
        dtype=np.float64,
    )

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
    
    # 3. Generate smooth S-curve velocity profiles (rad/s) for each trajectory segment.
    segment_profiles: List[Callable[[float], np.ndarray]] = []
    sampled_profiles: List[np.ndarray] = []
    for i in range(1, len(joint_path_final)):
        segment_duration: float = dt_list[i-1]
        profile = _quintic_scurve_velocity_profile(
            joint_path_final[i - 1],
            joint_path_final[i],
            segment_duration,
        )
        _, q_dot_samples = _sample_velocity_profile(profile, segment_duration, controller.dt)
        segment_profiles.append(profile)
        sampled_profiles.append(q_dot_samples)

    all_velocity_samples = np.vstack(sampled_profiles)
    
    # Display trajectory information
    print(f"\nTrajectory Summary:")
    print(f"  Timestamps: {timestamps}")
    print(f"  Segment durations: {dt_list}")
    print(f"  Sampled velocity profile shape: {all_velocity_samples.shape}")
    print(f"  Max joint velocities: {np.max(np.abs(all_velocity_samples), axis=0)}")
    
    # Visualize velocity profiles
    _plot_velocity_profiles(sampled_profiles, dt_list, controller.dt)
    
    # Execute trajectory with simple velocity profiles (waypoints already optimized)
    print("\nExecuting time-optimal trajectory...")
    for seg_idx, seg_duration in enumerate(dt_list):
        segment_q_dot = sampled_profiles[seg_idx]
        print(
            f" Segment {seg_idx+1}: "
            f"max(|vel|)={np.max(np.abs(segment_q_dot)):.3f} rad/s for {seg_duration:.2f}s"
        )
        controller.joint_space_vel_ctrl(
            q_dot_func=segment_profiles[seg_idx],
            duration=seg_duration,
        )
        
    print("Trajectory execution completed.")
    controller.go_home()

def _plot_velocity_profiles(
    sampled_profiles: List[np.ndarray],
    segment_durations: List[float],
    dt: float,
) -> None:
    """Plot sampled joint velocity profiles for visualization."""
    plt.figure(figsize=(10, 6))
    t_offset = 0.0
    for segment_idx, (segment_profile, duration) in enumerate(
        zip(sampled_profiles, segment_durations),
        start=1,
    ):
        t_segment = np.linspace(0.0, duration, len(segment_profile)) + t_offset
        for joint_idx in range(segment_profile.shape[1]):
            label = f'Joint {joint_idx+1}' if segment_idx == 1 else None
            plt.plot(t_segment, segment_profile[:, joint_idx], label=label)
        t_offset += duration

    plt.title("Joint Velocity Profiles (S-Curve)")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()
    
if __name__ == "__main__":
    sys.exit(main())
