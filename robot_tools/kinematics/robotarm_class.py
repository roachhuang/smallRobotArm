"""Small Robot Arm Kinematics Module

This module implements the forward and inverse kinematics for the small robot arm
based on the standard Denavit-Hartenberg parameters.

Classes:
    SmallRbtArm: Kinematics implementation for the specific robot arm
"""

import numpy as np
import modern_robotics as mr
from numpy import ndarray
from spatialmath import SE3
from .robotarm_independent_class import RobotArm
from . import jacobians as jk

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
         //设置电机1/电机4/电机6初始状态为非0状态 免于使用负数参数 
        //例如:底盘一轴电机初始步进值为10000 moveTO(0)顺转90度 moveTO(10000)归位  moveTO(20000) 逆转90度
        
        //轴    脉冲范围(极小l~零位o~极大m)    ROS范围(Rl~Rm)      Studio<-->real      脉冲与转角对应公式             细分a   步距角b  传动比c   极大
        //J1    0~10240~20480              -120~120               0      0        y = x/240*20480+10240          32     1.8    4.8       m = 360/b*a*c/360*(abs(Rm-Rl))
        //J2    0~5582~14115               -78.5~120              0      -90      y = (x+78.5)/198.5*14115       32     1.8    4.0       m = 360/b*a*c/360*(abs(Rm-Rl)) 
        //J3    0~6569~22569               73.9~-180              0      90       y = (-x+73.9)/253.9*22569      32     1.8    5.0       m = 360/b*a*c/360*(abs(Rm-Rl))
        //J4    0~8960~17920               -180~180               0      0        y = x/360*17920+8960           32     1.8    2.8       m = 360/b*a*c/360*(abs(Rm-Rl))
        //J5    0~8400~10080               -45~225                0      -90      y = (x+45)/270*10080           32     1.8    2.1       m = 360/b*a*c/360*(abs(Rm-Rl))
        //J6    0~3200~6400                -180~180               0      0        y = x/360*6400+3200            32     1.8    1.0       m = 360/b*a*c/360*(abs(Rm-Rl))

        """
        super().__init__()        
        self.dhTbl = std_dh_tbl  
        self.dof=6
        self.max_qlimits = ( 130, 130.0, 73.9,  50, 120, 180) 
        self.min_qlimits = (-130, -78.5, -66, -30, -90, -180)
        self.th_offsets = (0.0, -np.pi / 2, 0.0, 0.0, 0.0, 0.0)
        # self.controller = None
      
        self.T_wd = np.array([[1, 0, 0, 440], [0, 1, 0, -75], [0, 0, 1, 0], [0, 0, 0, 1]])
        robot_base_height = 35.0
        T_w0 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, robot_base_height], [0, 0, 0, 1]])
        self.T_w0_inv = np.linalg.inv(T_w0)
        
        tool_length = 25.0 # 36.0    # the tool attached to axis 6
        T_6c = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, tool_length], [0, 0, 0, 1]])
        self.T_6c_inv = np.linalg.inv(T_6c)
      
        # 4x4 transformation matrix from end-effector frame to cup frame
        # tool frame. this is for generating T_6E (end-effector/gripper to {6})
        # t_6E = smallRobotArm.pose2T([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])
        # hand made T_6E: gripper's len + x:180,y:-90, z:0. see their coordiation system.
        
        # 1. Zero Configuration Matrix (all theta are zero)
        self.M = np.array([
            [0,  0, 1, 192.5],
            [0, -1, 0, 0],
            [1,  0, 0, 269.0],
            [0,  0, 0, 1],
        ])
        # 2. Points on the joint axes (q)
        d1=133
        d4=117.5
        d6=28
        r1=47
        r2=110
        r3=26

        q1=np.array([0,0,0])
        q2=np.array([r1,0,d1])
        q3=np.array([r1,0,d1+r2])
        q4=np.array([r1,0,d1+r2+r3])
        q5=np.array([r1+d4,0,d1+r2+r3])
        # q5=np.array([164.5,0,269])
        q6=np.array([r1+d4+d6,0,d1+r2+r3])
        # q6=np.array([192.5,0,269])
        # 3. Direction of axes (CRITICAL: Double check your physical arm's tilt axes!)
        # since the screw axis has rotation, w is a unit vector. (screw axis wrt {s})
        w1=np.array([0, 0, 1])
        w2=np.array([0, 1, 0])
        w3=np.array([0, 1, 0])
        w4=np.array([1, 0, 0])
        w5=np.array([0, 1, 0])
        w6=np.array([1, 0, 0])
        # 4. Linear Velocity Components (Notice the negative sign: -w x q)
        v1=-np.cross(w1,q1)
        v2=-np.cross(w2,q2)
        v3=-np.cross(w3,q3)
        v4=-np.cross(w4,q4)
        v5=-np.cross(w5,q5)
        v6=-np.cross(w6,q6)
        # 5. Combine into 6x1 Screw Axis vectors (flattened) in the『{s}
        s1=np.hstack([w1,v1])
        s2=np.hstack([w2,v2])
        s3=np.hstack([w3,v3])
        s4=np.hstack([w4,v4])
        s5=np.hstack([w5,v5])
        s6=np.hstack([w6,v6])
        # Stack them into an elegant 6 x 6 matrix where each column is a screw axis
        self.Slist = np.column_stack((s1, s2, s3, s4, s5, s6))

        # Body-frame screw axes, MR Eq. 5.20: Blist = [Ad_{M^-1}] Slist
        self.Blist = mr.Adjoint(mr.TransInv(self.M)) @ self.Slist

    def inertia(self, q):
        # Rough estimates: larger joints = higher inertia
        joint_masses = [1.5, 1.8, 0.8, 0.5, 0.4, 0.2]  # kg
        """Configuration-dependent fake inertia."""
        # Simple example: inertia increases when arm is extended
        extension_factor = 1 + 0.5 * np.abs(q[1])  # Based on joint 2 angle
        base_inertia = np.diag(joint_masses)
        return base_inertia * extension_factor
        
    def jacobian_space(self, q: ndarray) -> ndarray:
        """Space Jacobian J_s(q), MR Eq. 5.11. Rows [omega; v], frame {s}."""
        return jk.jacobian_space(self.Slist, q)

    def jacobian_body(self, q: ndarray) -> ndarray:
        """Body Jacobian J_b(q), MR Eq. 5.18. Rows [omega; v], frame {b}."""
        return jk.jacobian_body(self.Blist, q)

    def jacobian_geometric(self, q: ndarray) -> ndarray:
        """Geometric Jacobian: MR block order [omega; v], EE-point referenced."""
        p_ee = self.poe_fk(q)[0:3, 3]
        return jk.spatial_to_geometric(self.jacobian_space(q), p_ee)

    def compute_jacobian(self, q: ndarray) -> ndarray:
        """Jacobian contract for the force controllers: geometric, EE-point wrench [m; f]."""
        return self.jacobian_geometric(q)

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

            # T01 = self.get_ti2i_1(1, Jik[0] + self.th_offsets[0])
            # T12 = self.get_ti2i_1(2, Jik[1] + self.th_offsets[1])
            # T23 = self.get_ti2i_1(3, Jik[2] + self.th_offsets[2])
            # T03 = T01 @ T12 @ T23
            T_03 = self.get_t_0n(Jik[0:3], 3)
            inv_T03 = np.linalg.inv(T_03)
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

    def poe_fk(self, theta_list:list) -> ndarray:
        """
        Calculates Forward Kinematics using Product of Exponentials.
        theta_list: array or list of 6 joint angles in radians [theta1, theta2, ..., theta6]
        """

        # 6. Calculate T(theta) using Modern Robotics library
        # This automatically computes: e^([s1]*th1) * e^([s2]*th2) * ... * M
        T_theta = mr.FKinSpace(self.M, self.Slist, theta_list)
    
        return T_theta
    
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
        #1, frame1 相對于frame0 的空間幾何關係; #2,frame1 下表達的vector可轉囘frame0下來表達.
        #3, 從 frame0 來看 frame1.
       
        # T = Twf @ T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56 @ Tft
        return self.get_t_0n(theta[0:6], 6)
    
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
