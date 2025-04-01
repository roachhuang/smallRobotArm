# the reason for importing cos and sin from sympy is Rot case (can't import them from math, plz note!!!)
# from sympy import symbols  # , nsimplify, Matrix
import numpy as np
from numpy import ndarray
# import helpers as hlp
from numpy import ndarray
from spatialmath.base import trinterp
from spatialmath import SE3
# from scipy.spatial.transform import Rotation as R
import serial_class as ser
from abc import ABC, abstractmethod

# robot arm independent
class RobotArm(ABC):
    def __init__(self):
        # init arm independent params
        pass                 
    
    def T2Pose(self, T, seq="xyz", degrees=True) -> tuple:
        """
        Converts a 4x4 transformation matrix to a pose (position and orientation).

        Args:
            T: 4x4 NumPy array representing the transformation matrix.
            euler_seq: Euler angle sequence (e.g., "xyz", "zyz").
            degrees: If True, returns Euler angles in degrees; otherwise, in radians.

        Returns:
            A tuple: (position, euler_angles)
            position: a 3 element numpy array.
            euler_angles: a 3 element numpy array.
        """
        # position = T[:3, 3]  # Extract position (translation)
        # rotation_matrix = T[:3, :3]  # Extract rotation matrix

        # rotation = R.from_matrix(rotation_matrix)
        # euler_angles = rotation.as_euler(seq=seq, degrees=degrees)

        # return (position, euler_angles)
    
        SE3_T = SE3(T)
        position = SE3_T.t  # Extract position as a list
        zyz_euler = SE3_T.eul(
            unit="deg"
        )  # Extract ZYZ Euler angles in radians

        return (np.round(position, 4), np.round(zyz_euler, 4))


    def pose2T(self, pose: tuple, seq="xyz") -> ndarray:
        """
        Args: position (x,y,z) + 3 rotation angles in ZYZ order in degree
        Returns: transformation matrix from pose

        The three rotations can either be in a global frame of reference (extrinsic) or in a body centred frame of reference (intrinsic),
        which is attached to, and moves with, the object under rotation [1].
        Robots with spherical wrists (where the last three joint axes intersect at a point)
        a.k.a. a4=a5=a6=0 (std dh tbl) often use "ZYZ" to represent the orientation of the wrist.
        NOTE: uppercase 'ZYZ' for intrinsic rotation; lowercase->fixed angles sequence
        """
        # avoid naming roll, pitch and yaw with zyz coz of misleading
        # x, y, z, psi, theta, phi = pose
        # r = R.from_euler(seq=seq, angles=[psi, theta, phi], degrees=True)

        # Translation matrix
        # translation_matrix = np.eye(4)
        # translation_matrix[:3, 3] = [x, y, z]

        # Homogeneous transformation matrix
        # T = np.eye(4)
        # T[:3, :3] = r.as_matrix()  # rotation_matrix
        # T[:3, 3] = [x, y, z]

        # Alternatively, you can multiply the translation and rotation matrix.
        # T = translation_matrix @ np.eye(4)
        # T[:3,:3] = rotation_matrix

        # return T
    
        x, y, z = pose[:3]
        alpha, beta, gamma = pose[3:6]
        # SO3.eul always zyz
        T = SE3(x, y, z) * SE3.Eul(alpha, beta, gamma, unit='deg')
        return T.A

    def get_ti2i_1(self, i, theta=None) -> ndarray:
        """
        todo: when theta will be none? examin intput theta's procision by checking its number of decimal points.
        Creates a DH transformation matrix using NumPy.

        Args:
            th: Theta (joint angle or variable). muse be in rad coz np.sin/cos... take rad.
            alfa: Alpha (twist angle).
            ai: ai (link length).
            di: di (link offset).
            theta: If None, returns a rounded matrix (approximating symbolic).
            tolerance: Tolerance for rounding (used if theta is None).

        Returns:
            NumPy array representing the DH transformation matrix.
        """
        # fill in dh tbl wrt robot arms' dh params

        # array idx starts frm 0, so i-1
        # alfa, ai, di, th = symbols("alfa, ai, di, th")
        alfa, ai, di = self.dhTbl[i - 1, :]
        # th = f"q{i}" if theta is None else theta
        th = theta

        # the reason for using sympy's Matrix is that we need to apply it with sympy's simplify func
        # to eliminate sth likes 1.xxxxxe-14 * sin(qx)
        # Ti_2_i-1=Tzi-1(thetai)TZr(di)TXq(ai)TXp(alphai)
        # this matrix is transformation matrix for std dh table
        # m = Matrix(
        #     [
        #         [cos(th), -sin(th) * cos(alfa), sin(th) * sin(alfa), ai * cos(th)],
        #         [sin(th), cos(th) * cos(alfa), -cos(th) * sin(alfa), ai * sin(th)],
        #         [0, sin(alfa), cos(alfa), di],
        #         [0, 0, 0, 1],
        #     ]
        # )

        """ standard DH table's T_i_to_i-1 coz smallrobotarm is uing std DH tbl.
            the m matrix will be difference if using craig dh table. see ntu 3-3
            Tzi-1(th_i)@Tzr(di)@Txq(ai)@Txp(alpha_i)
        """
        m = np.array(
            [
                [
                    np.cos(th),
                    -np.sin(th) * np.cos(alfa),
                    np.sin(th) * np.sin(alfa),
                    ai * np.cos(th),
                ],
                [
                    np.sin(th),
                    np.cos(th) * np.cos(alfa),
                    -np.cos(th) * np.sin(alfa),
                    ai * np.sin(th),
                ],
                [0, np.sin(alfa), np.cos(alfa), di],
                [0, 0, 0, 1],
            ]
        )
        # todo: find out when theta will be None and why?
        if theta is None:
            # m = nsimplify(m, tolerance=1e-10, rational=True)
            # # print(f't{i}-{i-1}: {m}')
            # return np.array(m)
            # ------------------------------------------------------------
            # For symbolic-like behavior (approximation with rationals)
            # NumPy doesn't have a direct equivalent to nsimplify with rational=True
            # This part requires more advanced techniques or a different library if you need exact rational approximations.
            # Here we approximate with floats and round to a certain tolerance.
            decimals = int(-np.log10(1e-8))  # calculates the number of decimals needed
            # 6-10 decimals is common for robotics, we use 8 here.
            m = np.round(m, decimals)  # approximates the symbolic nsimplify tolerance.
            return m
        else:
            # Matrix objects have a numpy method that returns a numpy.ndarray
            # float32 maintains only approximately 7 decimal digits fo precision internally.
            return m.astype(np.float64)
        
    def interpolate_poses(self, start_pose: ndarray, end_pose: ndarray, num_poses=10):
        """Generates smooth poses between two poses."""
        # Ensure inputs are SE3 objects
        # if not isinstance(start_pose, SE3) or not isinstance(end_pose, SE3):
        #     raise ValueError("start_pose and end_pose must be SE3 objects")

        # Generate interpolation values
        s_values = np.linspace(0, 1, num_poses + 2)[1:-1]  # Get 'in-between' values.

        # Interpolate poses
        poses = []
        for s in s_values:
            print(f"Interpolating with s={s}")
            pose = trinterp(start_pose, end_pose, s)  # Use .A matrices
            poses.append(SE3(pose))  # Convert back to SE3
        return poses    
    # interface
    @abstractmethod
    def ik(self, T_06:ndarray)->tuple:
        raise NotImplementedError("Subclasses must implement ik()")

    @abstractmethod
    def fk(self, Jfk):
        pass

    # @abstractmethod
    # def moveTo(self, end_effector_pose):
    #     pass


# robot arm dependent
class SmallRbtArm(RobotArm):    
    def __init__(self, std_dh_tbl: ndarray):
        # super().__init__(std_dh_tbl)        
        self.dhTbl = std_dh_tbl  
        self.robot_rest_angles = (0.0, -78.5, 73.9, 0.0, -90.0, 0.0)       
        
        self.max_qlimits = ( 130, 130.0, 73.9,  50, 120, 180) 
        self.min_qlimits = (-130, -78.5, -66, -30, -90, -180)
        self.th_offset = (0.0, -np.pi / 2, 0.0, 0.0, 0.0, 0.0)
        self.conn = ser.SerialPort()
        self.conn.connect()
        self.cup_height = 50
        # 4x4 transformation matrix from end-effector frame to cup frame
        self.T_EC=np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 50/2], [0, 0, 0, 1]])
        self.T_EC_inv=np.linalg.inv(self.T_EC)
        # tool frame. this is for generating T_6E (end-effector/gripper to {6})
        # 50mm is the distance btw frame6 to end-effector
        # t_6E = smallRobotArm.pose2T([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])
        # hand made T_6E: gripper's len + x:180,y:-90, z:0. see their coordiation system.
        # self.T_6E = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 50.0], [0, 0, 0, 1]])
        # self.T_6E_inv = np.linalg.inv(self.T_6E)
        self.T_6C = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, -50/2], [0, 0, 0, 1]])
        self.T_6C_inv = np.linalg.inv(self.T_6C)
        
         # hand made T_6E: gripper's len + x:180,y:-90, z:0. see their coordiation system.
        tool_length = 50.0    # the tool attached to axis 6
        self.T_6E = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, tool_length], [0, 0, 0, 1]])
        self.T_6E_inv = np.linalg.inv(self.T_6E)
        
    def limit_joint_angles(self, angles):
        """Limits joint angles to specified max/min values."""
        if len(angles) != len(self.max_qlimits) or len(angles) != len(self.min_qlimits):
            raise ValueError("Angle and qlimit lists must have the same length.")
        # note that j4 can move only when j3 < 63.9
        return [
            max(min(a, max_val), min_val)
            for a, max_val, min_val in zip(angles, self.max_qlimits, self.min_qlimits)
        ]

    def grab(self):
        self.conn.ser.write(b"eOn\n")

    def drop(self):
        self.conn.ser.write(b"eOff\n")

    def enable(self):
        # motors are disabled in arduino's setup()
        self.conn.ser.write(b"en\n")
        # sleep(.5)
        # self.conn.ser.write(b"rst\n")
        # sleep(.5)

    def disable(self):
        self.conn.ser.write(b"dis\n")

    def move_to_angles(self, j: tuple) -> None:
        # return super().moveTo(end_effector_pose)
        limited_j = self.limit_joint_angles(j)
        cmd = {"header": "g", "joint_angle": limited_j, "ack": True}
        self.conn.send2Arduino(cmd)
        
    def go_home(self):
        self.move_to_angles(self.robot_rest_angles)
        self.disable()
        # a way to terminate thread
        self.conn.disconnect()
        
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

            T01 = self.get_ti2i_1(1, Jik[0] + self.th_offset[0])
            T12 = self.get_ti2i_1(2, Jik[1] + self.th_offset[1])
            T23 = self.get_ti2i_1(3, Jik[2] + self.th_offset[2])

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
        theta = Jfk + np.degrees(self.th_offset)

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
