"""Robot Controller Module (Legacy)

This module provides backward compatibility for the original RobotController.
New code should use VelocityController and PositionController directly.

Classes:
    RobotController: Combined controller for backward compatibility
"""

from .velocity_controller import VelocityController
from .position_controller import PositionController
import numpy as np

class RobotController(VelocityController, PositionController):
    """Combined controller for backward compatibility.
    
    This class combines velocity and position control functionality.
    For new projects, consider using VelocityController and PositionController directly.
    """
    
    def __init__(self, robot):
        # Initialize both parent classes
        VelocityController.__init__(self, robot)
        PositionController.__init__(self, robot)        
        
    def compute_approach_pose(self, T_cup, approach_vec_cup, offset=50):
        """
        Compute an approach pose with a specified offset from the target.
        
        Args:
            T_cup: 4x4 transformation matrix of the cup/target
            approach_vec_cup: 3D vector defining approach direction in cup frame
            offset: distance in mm to offset from the target
        
        Returns:
            4x4 transformation matrix for the approach pose
        """
        # Extract rotation and position from target transformation
        R_cup = T_cup[0:3, 0:3]
        p_cup = T_cup[0:3, 3]

        # Transform approach vector to base frame
        approach_vec_base = R_cup @ approach_vec_cup
        
        # Calculate position with offset
        p_tcp = p_cup + offset * approach_vec_base
        
        # Calculate orientation (z-axis aligned with approach vector)
        z_tcp = -approach_vec_base  # Negative because we approach opposite to vector
        
        # Find perpendicular vectors to form coordinate frame
        world_up = np.array([0, 0, 1])
        x_tcp = np.cross(world_up, z_tcp)
        if np.linalg.norm(x_tcp) < 1e-3:
            # Handle singularity if approach is parallel to world up
            world_up = np.array([0, 1, 0])
            x_tcp = np.cross(world_up, z_tcp)
        x_tcp /= np.linalg.norm(x_tcp)
        y_tcp = np.cross(z_tcp, x_tcp)

        # Construct rotation matrix and transformation
        R_tcp = np.column_stack((x_tcp, y_tcp, z_tcp))
        T_tcp = np.eye(4)
        T_tcp[0:3, 0:3] = R_tcp
        T_tcp[0:3, 3] = p_tcp
        
        return T_tcp
    
    # def simple_combined_pick(self, target_pose, grasp_candidates):
    #     """Simplest MIT + approach pose combination"""
        
    #     # 1. MIT: Select best grasp by reachability only
    #     best_grasp = None
    #     for grasp in grasp_candidates:
    #         joints = SmallRbtArm.ik(self.convert_p_dc_to_T06(grasp['pose']))
    #         if joints is not None:  # Just check if reachable
    #             best_grasp = grasp
    #             break
        
    #     # 2. Your approach: Generate approach pose
    #     approach_pose = self.compute_approach_pose(
    #         SE3.Trans(best_grasp['pose'][:3]) * SE3.RPY(best_grasp['pose'][3:], order="zyx", unit="deg"),
    #         best_grasp['approach_vector'], 
    #         offset=50
    #     )
        
    #     # 3. Execute: approach → grasp → retreat → place
    #     self.move_to_angles(self.robot.ik(self.convert_p_dc_to_T06(self.T2Pose(approach_pose))))
    #     self.move_to_angles(self.robot.ik(self.convert_p_dc_to_T06(best_grasp['pose'])))
    #     self.grab()
    #     self.move_to_angles(self.robot.ik(self.convert_p_dc_to_T06(self.T2Pose(approach_pose))))
    #     self.move_to_angles(self.robot.ik(self.convert_p_dc_to_T06(target_pose)))
    #     self.drop()
        
    def grab(self):
        """Activate the gripper to grab an object."""
        self.conn.ser.write(b"eOn\n")

    def drop(self):
        """Deactivate the gripper to release an object."""
        self.conn.ser.write(b"eOff\n")

    def enable(self):
        """Enable all motors on the robot arm."""
        # motors are disabled in arduino's setup()
        self.conn.ser.write(b"en\n")       

    def calibrate(self):
        """Calibrate the robot by homing all axes."""
        self.conn.ser.write(b"g28\n")  # G28 is the command to home the robot arm
        
    def disable(self):
        """Disable all motors on the robot arm."""
        self.conn.ser.write(b"dis\n")

    def move_to_angles(self, j: tuple, header='g', ack=True) -> None:
        """Move the robot to specified joint angles.
        
        Args:
            j (tuple): Target joint angles in degrees
            header (str): Command header for Arduino protocol
            ack (bool): Whether to wait for acknowledgment from Arduino
        """
        # return super().moveTo(end_effector_pose)
        # limited_j = self.limit_joint_angles(j)
        cmd = {"header": header, "joint_angle": j, "ack": ack}
        self.conn.send2Arduino(cmd)
        # naively assume motors moved accordinly. remove this line if motors have encoder.
        self.current_angles = j
    
    # def optimize_manipulability_path(self, waypoints):
    #     optimized_path = []
    #     for waypoint in waypoints:
    #         # Use your existing eigen_analysis
    #         analysis = self.eigen_analysis(self.robot, waypoint)
    #         if analysis['manipulability'] < 0.1:  # Near singularity
    #             # Adjust joint angles to improve manipulability
    #             waypoint = self.adjust_for_manipulability(waypoint)
    #         optimized_path.append(waypoint)
    #     return optimized_path

    def impedance_control(self, x_desired, x_dot_desired, f_external):
        # Your existing position control + force accommodation
        x_error = x_desired - self.get_current_pose()
        f_desired = self.K_p @ x_error + self.K_d @ x_dot_desired
        f_command = f_desired - f_external  # Accommodate external forces
        return self.force_to_joint_torques(f_command)

    def closed_loop_control(self, target_angles):
        while not self.at_target(target_angles):
            current = self.read_encoders()
            error = target_angles - current
            self.send_velocity_command(self.pid_controller(error))

    def compliant_grasp(self, target_pose):
        self.move_to_pose(target_pose)
        while self.force_reading() < self.grasp_threshold:
            self.close_gripper_slowly()
        # Perfect grasp force achieved

    def visual_servo_control(self):
        while not self.object_centered():
            object_pos = self.camera.detect_object()
            error = self.target_pos - object_pos
            velocity = self.visual_jacobian @ error
            self.cartesian_space_vel_ctrl(lambda t: velocity, dt=0.02)

    
    def go_init(self):
        """Move the robot to its initial position."""
        self.move_to_angles(j=self.robot_init_angles)   
       
    def go_home(self):
        """Move the robot to its home/rest position and disconnect.
        
        This is typically called before shutting down the system.
        """
        print('gohome')
        self.move_to_angles(j=self.robot_rest_angles)
        # self.current_angles = self.robot_rest_angles
        # self.disable()
        # a way to terminate thread
        self.conn.disconnect()
    
