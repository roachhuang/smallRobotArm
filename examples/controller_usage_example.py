"""Controller Usage Example

This example demonstrates how to use the separated position and velocity controllers.
"""

import numpy as np
from robot_tools.kinematics import SmallRbtArm, std_dh_params
from robot_tools.controller import VelocityController, PositionController, RobotController

def main():
    # Create robot model
    robot = SmallRbtArm(std_dh_params)
    
    # Option 1: Use separate controllers
    print("=== Using Separate Controllers ===")
    
    # Velocity control
    vel_controller = VelocityController(robot)
    vel_controller.enable()
    
    # Constant velocity example
    constant_velocity = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad/s
    vel_controller.joint_space_vel_ctrl(
        q_dot_func=lambda t: constant_velocity,
        duration=2.0
    )
    
    # Position control
    pos_controller = PositionController(robot)
    
    # Move to specific joint angles
    target_angles = (10, -45, 30, 0, -90, 0)
    pos_controller.move_to_joint_angles(target_angles, optimize_path=True)
    
    # Move to Cartesian pose
    target_pose = [200, 100, 150, 0, 90, 0]  # [x, y, z, rx, ry, rz]
    pos_controller.move_to_pose(target_pose, optimize_path=True)
    
    pos_controller.go_home()
    
    # Option 2: Use combined controller (backward compatibility)
    print("\n=== Using Combined Controller (Legacy) ===")
    
    combined_controller = RobotController(robot)
    combined_controller.enable()
    
    # Can use both position and velocity methods
    combined_controller.joint_space_vel_ctrl(
        q_dot_func=lambda t: constant_velocity,
        duration=1.0
    )
    
    combined_controller.move_to_joint_angles(target_angles, optimize_path=True)
    combined_controller.go_home()

if __name__ == "__main__":
    main()