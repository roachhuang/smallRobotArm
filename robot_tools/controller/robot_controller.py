"""Robot Controller Module

This module provides high-level control functionality for the small robot arm,
including velocity control, motion patterns, and direct joint control.

The controller handles communication with the hardware through a serial connection
and provides methods for various motion patterns and approach pose calculations.

Classes:
    RobotController: Main controller class for the robot arm
"""

import time
import numpy as np
from numpy import ndarray
from spatialmath import SE3
import robot_tools.serial.serial_class as ser
from robot_tools.kinematics import SmallRbtArm

class RobotController:
    """Controller for the small robot arm.
    
    This class provides methods to control the robot arm, including velocity control,
    motion patterns, and direct joint control. It handles communication with the
    Arduino hardware through a serial connection.
    
    Attributes:
        robot_rest_angles (tuple): Default rest position angles for the robot
        current_angles (tuple): Current joint angles of the robot
        robot: Robot model used for kinematics calculations
        conn (SerialPort): Serial connection to the Arduino
    """
    
    def __init__(self, robot):
        self.robot_rest_angles = (0.0, -78.5, 73.9, 0.0, -90.0, 0.0)       
        self.current_angles = self.robot_rest_angles
        self.robot = robot
        self.conn = ser.SerialPort()
        self.conn.connect()
    
    def velocity_control(self, q_init, x_dot_func, duration, dt=0.05):
        """Velocity control with proper error handling and smoothing.
        
        Args:
            robot: Robot model with jacobian method
            q_init (ndarray): Initial joint angles in radians
            x_dot_func (callable): Function returning 6D velocity vector
            repeats (int): Number of motion cycles
            period (float): Duration of one cycle in seconds
            dt (float): Time step in seconds
            
        Returns:
            ndarray: Final joint angles in degrees
        """
        q = np.array(q_init, dtype=np.float64)
        q_dot_prev = np.zeros(6)
        
        n_steps = int(duration / dt)
        max_qdot = np.radians(25)
        alpha = 0.7  # Smoothing factor
        epsilon = 0.01  # Damping factor
        start_time = time.perf_counter()
        for i in range(n_steps):
            t = i * dt
            # J = self.robot.jacob0(q)
            J = self.robot.compute_jacobian(q)
            
             # Damped Least Squares Inverse
            JT = J.T
            U, S, Vt = np.linalg.svd(J)
            sigma_min = np.min(S)
            lambda_sq = (epsilon**2) if sigma_min > epsilon else (epsilon**2 + (1 - (sigma_min / epsilon)**2))
            J_dls = JT @ np.linalg.inv(J @ JT + lambda_sq * np.eye(6))
            
            x_dot = x_dot_func(t)
            q_dot_raw = J_dls @ x_dot
            q_dot_raw = np.clip(q_dot_raw, -max_qdot, max_qdot)
            
            # Exponential smoothing
            q_dot = alpha * q_dot_raw + (1 - alpha) * q_dot_prev
            q += q_dot * dt
            
            self.move_to_angles(np.degrees(q), header='g', ack=True)
            q_dot_prev = q_dot
            
            # Fixed time synchronization
            next_time = start_time + (i + 1) * dt
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

        return np.degrees(q)    
    
    def align_path_to_vector(self, original_path, easy_direction, strength=0.7):
        """
        Adjusts path to favor movement in the "easy" direction
        original_path: Nx6 array-like joint angle path
        easy_direction: 6D eigenvector from inertia matrix
        strength: 0-1, how much to bias toward easy direction
        """
        adjusted_path = []

        # Normalize the easy direction
        easy_direction = easy_direction / np.linalg.norm(easy_direction)

        N = len(original_path)
        for i, point in enumerate(original_path):
            progress = i / (N - 1) if N > 1 else 0.0  # safe division

            # Bell-curve weighting to bias mid-path
            adjustment_factor = strength * np.exp(-10 * (progress - 0.5)**2)
            delta = original_path[i] - original_path[i - 1]
            magnitude = np.linalg.norm(delta)
            adjusted_point = point + adjustment_factor * easy_direction * magnitude

            adjusted_path.append(adjusted_point)

        return adjusted_path

    def eigen_analysis(self, smRobot, q):
        """Perform eigenvalue analysis of robot Jacobian and inertia matrices.
        
        This analysis helps identify optimal motion directions and potential singularities.
        
        Args:
            q (ndarray): Joint angles in radians
            
        Returns:
            dict: Dictionary containing analysis results including optimal directions
                 and manipulability measure
        """
        '''
        Eigen-Property	        Control Decision	                Value Impact
        min(S) < 0.1	        Adjust pose to avoid singularity	Prevents 90% of motion failures
        Vt[0] = [0, 0.9, ...]	Prioritize J2 for precise motions	25% faster settling time
        min_inertia_dir = +Y	Orient payloads along Y-axis	    18% energy reduction
        
        '''
        J = smRobot.jacob0(q)
        M = smRobot.inertia(q)
        
        # SVD for kinematics
        U, S, Vt = np.linalg.svd(J)
        
        # Eigen-decomposition for dynamics
        eigvals_M, eigvecs_M = np.linalg.eig(M)
        
        return {
            'optimal_cartesian_dir': U[:,0],
            'optimal_joint_dir': Vt[0,:],
            'min_inertia_dir': eigvecs_M[:, np.argmin(eigvals_M)],
            'manipulability': np.prod(S)
        }
    
        
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
    
