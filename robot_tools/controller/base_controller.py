"""Base Controller Module

This module provides the base controller functionality for the small robot arm,
including hardware interface, basic motion commands, and analysis methods.

Classes:
    BaseController: Base controller class with common functionality
"""

# import time
import numpy as np
from numpy import ndarray
# from spatialmath import SE3
import robot_tools.serial.serial_class as ser
from roboticstoolbox import DHRobot

class BaseController:
    """Base controller for the small robot arm.
    
    This class provides common functionality for all controllers including
    hardware interface, basic motion commands, and analysis methods.
    
    Attributes:
        robot_rest_angles (tuple): Default rest position angles for the robot
        current_angles (tuple): Current joint angles of the robot
        robot: Robot model used for kinematics calculations
        conn (SerialPort): Serial connection to the Arduino
    """
    
    def __init__(self, robot):
        self.robot_init_angles = (0, 0, 0, 0, 0, 0)
        self.robot_rest_angles = (0.0, -78.5, 73.9, 0.0, -90.0, 0.0)       
        self.current_angles = self.robot_rest_angles
        # industrial robots typically 100-10000hz (0.001-0.01s)
        # send2Arduino takes about 6-20ms, so 0.02 is appropiate
        self.dt = 0.02  # Default dt for the main control loop
        
        self.robot = robot
        self.conn = ser.SerialPort()  # h/w interface
        self.conn.connect()
    
    # Path optimization methods moved to robot_tools.trajectory.path_optimizer
    # Use PathOptimizer class for trajectory optimization functionality
    
    def compute_approach_pose(self, T_cup, approach_vec_cup, offset=50):
        """Compute an approach pose with a specified offset from the target."""
        R_cup = T_cup[0:3, 0:3]
        p_cup = T_cup[0:3, 3]
        approach_vec_base = R_cup @ approach_vec_cup
        p_tcp = p_cup + offset * approach_vec_base
        z_tcp = -approach_vec_base
        
        world_up = np.array([0, 0, 1])
        x_tcp = np.cross(world_up, z_tcp)
        if np.linalg.norm(x_tcp) < 1e-3:
            world_up = np.array([0, 1, 0])
            x_tcp = np.cross(world_up, z_tcp)
        x_tcp /= np.linalg.norm(x_tcp)
        y_tcp = np.cross(z_tcp, x_tcp)

        R_tcp = np.column_stack((x_tcp, y_tcp, z_tcp))
        T_tcp = np.eye(4)
        T_tcp[0:3, 0:3] = R_tcp
        T_tcp[0:3, 3] = p_tcp
        
        return T_tcp
    
    def move_to_angles(self, j: tuple, header='g', ack=True) -> None:
        """Move the robot to specified joint angles."""
        cmd = {"header": header, "joint_angle": j, "ack": ack}
        self.conn.send2Arduino(cmd)
        self.current_angles = j
    
    def grab(self):
        """Activate the gripper to grab an object."""
        self.conn.ser.write(b"eOn\n")

    def drop(self):
        """Deactivate the gripper to release an object."""
        self.conn.ser.write(b"eOff\n")

    def enable(self):
        """Enable all motors on the robot arm."""
        self.conn.ser.write(b"en\n")       

    def calibrate(self):
        """Calibrate the robot by homing all axes."""
        self.conn.ser.write(b"g28\n")
        
    def disable(self):
        """Disable all motors on the robot arm."""
        self.conn.ser.write(b"dis\n")
    
    def go_init(self):
        """Move the robot to its initial position."""
        self.move_to_angles(j=self.robot_init_angles)   
       
    def go_home(self):
        """Move the robot to its home/rest position and disconnect."""
        print('gohome')
        self.move_to_angles(j=self.robot_rest_angles)
        self.conn.disconnect()