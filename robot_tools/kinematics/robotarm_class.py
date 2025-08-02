"""Small Robot Arm Kinematics Module

This module implements the forward and inverse kinematics for the small robot arm
based on the standard Denavit-Hartenberg parameters.

Classes:
    SmallRbtArm: Kinematics implementation for the specific robot arm
"""

import numpy as np
from numpy import ndarray
from spatialmath import SE3

from .robotarm_independent_class import RobotArm

class SmallRbtArm(RobotArm):    
    """Small Robot Arm kinematics implementation.
    
    This class implements the specific kinematics for the small robot arm,
    including forward and inverse kinematics, joint limits, and coordinate
    transformations between different reference frames.
    
    Attributes:
        dhTbl (ndarray): DH parameters table
        max_qlimits (tuple): Maximum joint angle limits in degrees
        min_qlimits (tuple): Minimum joint angle limits in degrees
        th_offset (tuple): Joint angle offsets in radians
        controller: Reference to the robot controller
        T_wd (ndarray): Transformation from world to desk frame
        T_w0_inv (ndarray): Inverse transformation from world to robot base frame
        T_6c_inv (ndarray): Inverse transformation from joint 6 to cup/tool frame
    """
    def __init__(self, std_dh_tbl: ndarray):
        """Initialize the small robot arm with DH parameters.
        
        Args:
            std_dh_tbl (ndarray): Standard DH parameters table
        """
        super().__init__()        
        self.dhTbl = std_dh_tbl  
        self.dof=6
        self.max_qlimits = ( 130, 130.0, 73.9,  50, 120, 180) 
        self.min_qlimits = (-130, -78.5, -66, -30, -90, -180)
        self.th_offsets = (0.0, -np.pi / 2, 0.0, 0.0, 0.0, 0.0)
        self.controller = None
      
        self.T_wd = np.array([[1, 0, 0, 440], [0, 1, 0, -75], [0, 0, 1, 0], [0, 0, 0, 1]])
        robot_base_height = 0.0
        T_w0 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, robot_base_height], [0, 0, 0, 1]])
        self.T_w0_inv = np.linalg.inv(T_w0)
        
        tool_length = 36.0    # the tool attached to axis 6
        T_6c = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, tool_length], [0, 0, 0, 1]])
        self.T_6c_inv = np.linalg.inv(T_6c)
      
        # 4x4 transformation matrix from end-effector frame to cup frame
        # tool frame. this is for generating T_6E (end-effector/gripper to {6})
        # t_6E = smallRobotArm.pose2T([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])
        # hand made T_6E: gripper's len + x:180,y:-90, z:0. see their coordiation system.
    
    def compute_jacobian(self, q):
        """Compute Jacobian matrix from DH parameters.
        
        Args:
            dh_params: Nx3 array [alpha, a, d] for each joint
            q: Joint angles in radians (6x1)
            
        Returns:
            6x6 Jacobian matrix
        """
        n = len(q)
        J = np.zeros((6, n))
        
        # Compute transformation matrices T_0^i for each joint
        T = np.eye(4)
        T_list = [T.copy()]  # T_0^0 = I
        
        for i in range(n):
            alpha, a, d = self.dhTbl[i]
            theta = q[i] + self.th_offsets[i]  # Add offset
            
            # Standard DH transformation matrix
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha), np.sin(alpha)
            
            Ti = np.array([
                [ct, -st*ca, st*sa, a*ct],
                [st, ct*ca, -ct*sa, a*st],
                [0, sa, ca, d],
                [0, 0, 0, 1]
            ])
            
            T = T @ Ti
            T_list.append(T.copy())  # T_0^(i+1)
        
        # End-effector position
        p_n = T_list[-1][:3, 3]
        
        # Compute Jacobian columns
        for i in range(n):
            # z-axis and origin of frame i
            z_i = T_list[i][:3, 2]  # z-axis of frame i
            p_i = T_list[i][:3, 3]  # origin of frame i
            
            # Jacobian column for joint i
            J[:3, i] = np.cross(z_i, p_n - p_i)  # Linear velocity
            J[3:, i] = z_i                        # Angular velocity
        
        return J
    
    def convert_p_dc_to_T06(self, p:tuple)->ndarray:
        """
        Convert a desk/cup pose to the robot's joint-space transformation matrix T0_6.

        Args:
            p (tuple): A 6-element tuple containing (x, y, z, roll, pitch, yaw) in mm and degrees.

        Returns:
            ndarray: 4x4 transformation matrix representing the pose in the robot's joint frame.
        """
        T_dc = SE3.Trans(p[0:3]) * SE3.RPY(p[3:6], order="zyx", unit="deg")
        T_wc = self.T_wd @ T_dc.A
        T_06 = self.T_w0_inv @ T_wc @ self.T_6c_inv
        return T_06        
        
    def limit_joint_angles(self, angles):
        """Limit joint angles to specified max/min values.
        
        Args:
            angles (list or tuple): Joint angles to be limited
            
        Returns:
            list: Joint angles clipped to the allowed range
            
        Raises:
            ValueError: If the input angles list length doesn't match limits length
        """
        if len(angles) != len(self.max_qlimits) or len(angles) != len(self.min_qlimits):
            raise ValueError("Angle and qlimit lists must have the same length.")
        # note that j4 can move only when j3 < 63.9
        return [
            max(min(a, max_val), min_val)
            for a, max_val, min_val in zip(angles, self.max_qlimits, self.min_qlimits)
        ]
        
    # @hlp.timer
    def ik(self, T_06: ndarray) -> tuple | None:
        """
        in general, the T taken in is the desired pose of the end-effector frame relative to the world frame)
        
        arg:
            pose: end-effector pose in cartension space.
            position is retrieve from T_06.
            orientation(j4,5,6) are in deg
        return:
            4 decimaled joints angles in degrees.
        """
        (_, r1, d1) = self.dhTbl[0, :]
        r2 = self.dhTbl[1, 1]
        (_, r3, d3) = self.dhTbl[2, :]
        d4 = self.dhTbl[3, 2]
        d6 = self.dhTbl[5, 2]

        Jik = np.zeros((6,), dtype=np.float64)        

        # T_06 = T_0E @ self.T_6E_inv
        wrist_position = T_06[:3, 3] - d6 * T_06[:3, 2]

        try:
            Jik[0] = np.arctan2(wrist_position[1], wrist_position[0]) - np.arctan2(
                d3, np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
            )

            arccos_input_2 = (
                r2**2
                + (wrist_position[2] - d1) * (wrist_position[2] - d1)
                + (
                    np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
                    - r1
                )
                * (
                    np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
                    - r1
                )
                - (r3**2 + d4**2)
            ) / (
                2.0
                * r2
                * np.sqrt(
                    (wrist_position[2] - d1) * (wrist_position[2] - d1)
                    + (
                        np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
                        - r1
                    )
                    * (
                        np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
                        - r1
                    )
                )
            )

            arccos_input_2 = np.clip(arccos_input_2, -1.0, 1.0)

            Jik[1] = (
                np.pi / 2.0
                - np.arccos(arccos_input_2)
                - np.arctan2(
                    (wrist_position[2] - d1),
                    (
                        np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
                        - r1
                    ),
                )
            )

            arccos_input_3 = (
                r2**2
                + r3**2
                + d4**2
                - (wrist_position[2] - d1) * (wrist_position[2] - d1)
                - (
                    np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
                    - r1
                )
                * (
                    np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
                    - r1
                )
            ) / (2 * r2 * np.sqrt(r3**2 + d4**2))

            arccos_input_3 = np.clip(arccos_input_3, -1.0, 1.0)

            Jik[2] = np.pi - np.arccos(arccos_input_3) - np.arctan2(d4, r3)

            T01 = self.get_ti2i_1(1, Jik[0] + self.th_offsets[0])
            T12 = self.get_ti2i_1(2, Jik[1] + self.th_offsets[1])
            T23 = self.get_ti2i_1(3, Jik[2] + self.th_offsets[2])

            T03 = T01 @ T12 @ T23
            inv_T03 = np.linalg.inv(T03)
            T36 = inv_T03 @ T_06

            Jik[3] = np.arctan2(-T36[1][2], -T36[0][2])
            Jik[4] = np.arctan2(np.sqrt(T36[0][2] ** 2 + T36[1][2] ** 2), T36[2][2])
            Jik[5] = np.arctan2(-T36[2][1], T36[2][0])

            Jik = np.degrees(Jik)
            Jik = np.round(Jik, decimals=4)

            return tuple(Jik)

        except ValueError:
            print("Warning: arccos domain error in Joint calculations.")
            return None

    # todo: fk taks care of qs wrt t06 instead of t0-cup. don't do the transformation in fk.
    # @hlp.timer
    def fk(self, Jfk:tuple) -> ndarray:
        """
        arg:
            Jfk(in deg) - joints value for the calculation of the forward kinematics
        return:
            Xfk - pos value for the calculation of the forward kinematics
        """
        # Denavit-Hartenberg matrix
        # theta = np.zeros((6,), dtype=np.float64)
        # theta=[Jfk(1); -90+Jfk(2); Jfk(3); Jfk(4); Jfk(5); Jfk(6)];
        theta = Jfk + np.degrees(self.th_offsets)

        # alfa = self.dhTbl[0:6, 0]
        theta = np.deg2rad(theta)
        # theta = np.deg2rad(Jfk)
        # alfa = np.deg2rad(alfa)

        # world frame, there is a 25mm offset in z direction btw world frame and frame0
        Twf = self.pose2T([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], seq='ZYZ')
        # even with std_dh_tbl, we still need to give last link's offset for end-effector.
        Tft = self.pose2T([0.0, 0.0, 50.0, 180.0, -90.0, 0.0], seq='ZYZ')
        # tool frame
        # Tft = self.pose2T([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # DH homogeneous transformation matrix
        # float T01[16], T12[16], T23[16], T34[16], T45[16], T56[16];
        # T 1 到 0 (1在下, 0在上).
        # #1, frame1 相對于frame0 的空間幾何關係; #2,frame1 下表達的vector可轉囘frame0下來表達.
        # #3, 從 frame0 來看 frame1.
        T_01 = self.get_ti2i_1(1, theta[0])
        # T 2 到 1
        T_12 = self.get_ti2i_1(2, theta[1])
        T_23 = self.get_ti2i_1(3, theta[2])
        T_34 = self.get_ti2i_1(4, theta[3])
        T_45 = self.get_ti2i_1(5, theta[4])
        T_56 = self.get_ti2i_1(6, theta[5])
        # T = Twf @ T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56 @ Tft
        T = T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56        
        return T
    
        # # print('t: ', np.around(T, 2))
        # Xfk = np.zeros((6,), dtype=np.float64)
        # # get position from transformation matrix
        # position = T[0:3, 3]

        # Xfk[4] = np.arctan2(np.sqrt(T[2, 0] ** 2 + T[2, 1] ** 2), T[2, 2])
        # Xfk[3] = np.arctan2(T[1, 2] / np.sin(Xfk[4]), T[0, 2] / np.sin(Xfk[4]))
        # Xfk[5] = np.arctan2(T[2, 1] / np.sin(Xfk[4]), -T[2, 0] / np.sin(Xfk[4]))                     
                
        # # convert to degree
        # orientation = np.degrees(Xfk[3:6])
        # return (position, orientation)
