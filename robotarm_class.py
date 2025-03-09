# the reason for importing cos and sin from sympy is Rot case (can't import them from math, plz note!!!)
# from sympy import symbols  # , nsimplify, Matrix
import numpy as np
import helpers as hlp

# from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
import serial_class as ser
from abc import ABC, abstractmethod


class RobotArm(ABC):
    def __init__(self, std_dh_tbl: np.array):
        self.dhTbl = std_dh_tbl
        self.max_limits = (130,180,180,25,280,180) # 70,)
        self.min_limits = (-125, -180, -180, -25, -60, -180)
        
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
        position = T[:3, 3]  # Extract position (translation)
        rotation_matrix = T[:3, :3]  # Extract rotation matrix

        rotation = R.from_matrix(rotation_matrix)
        euler_angles = rotation.as_euler(seq=seq, degrees=degrees)

        return (position, euler_angles)

    def pose2T(self, pose: tuple, seq="xyz") -> np.array:
        """
        Args: position (x,y,z) + 3 rotation angles in ZYZ order
        Returns: transformation matrix from pose

        The three rotations can either be in a global frame of reference (extrinsic) or in a body centred frame of reference (intrinsic),
        which is attached to, and moves with, the object under rotation [1].
        Robots with spherical wrists (where the last three joint axes intersect at a point)
        a.k.a. a4=a5=a6=0 (std dh tbl) often use "zyz" to represent the orientation of the wrist.
        NOTE: uppercase 'ZYZ' for intrinsic rotation; lowercase->fixed angles sequence
        """
        # avoid naming roll, pitch and yaw with zyz coz of misleading
        x, y, z, psi, theta, phi = pose
        r = R.from_euler(seq=seq, angles=[psi, theta, phi], degrees=True)

        # Translation matrix
        translation_matrix = np.eye(4)
        translation_matrix[:3, 3] = [x, y, z]

        # Homogeneous transformation matrix
        T = np.eye(4)
        T[:3, :3] = r.as_matrix()  # rotation_matrix
        T[:3, 3] = [x, y, z]

        # Alternatively, you can multiply the translation and rotation matrix.
        # T = translation_matrix @ np.eye(4)
        # T[:3,:3] = rotation_matrix

        return T

    def get_ti2i_1(self, i, theta=None) -> np.array:
        """
        todo: when theta will be none? examin intput theta's procision by checking its number of decimal points.
        Creates a DH transformation matrix using NumPy.

        Args:
            th: Theta (joint angle or variable).
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

    @abstractmethod
    def ik(self):
        pass

    @abstractmethod
    def fk(self, Jfk):
        pass

    # @abstractmethod
    # def moveTo(self, end_effector_pose):
    #     pass


class SmallRbtArm(RobotArm):
    def __init__(self, std_dh_tbl: np.array):        
        super().__init__(std_dh_tbl)
        self.conn = ser.SerialPort()
        self.conn.connect()
        
    def limit_joint_angles(self, angles):
        """Limits joint angles to specified max/min values."""
        if len(angles) != len(self.max_limits) or len(angles) != len(self.min_limits):
            raise ValueError("Angle and limit lists must have the same length.")
        return [max(min(a, max_val), min_val) for a, max_val, min_val in zip(angles, self.max_limits, self.min_limits)]
            
    def grab(self):
        self.conn.ser.write(b"eOn\n")

    def drop(self):
        self.conn.ser.write(b"eOff\n")

    def enable(self):
        # motors are disabled in arduino's setup()
        self.conn.ser.write(b"en\n")
        # sleep(.5)
        self.conn.ser.write(b"rst\n")
        # sleep(.5)

    def disable(self):
        self.conn.ser.write(b"dis\n")

    def move_to_pose(self, T: np.array)->None:
        # return super().moveTo(end_effector_pose)
        j = self.ik(T)
        print("q:", j)
        j_array = np.array(j)
        if np.isnan(j_array).any():
            return
        limit_j=self.limit_joint_angles(j)
        cmd = {"header": "j", "joint_angle": limit_j, "ack": True}
        self.conn.send2Arduino(cmd)

    def move_to_angles(self, j: tuple) -> None:
        # return super().moveTo(end_effector_pose)
        cmd = {"header": "j", "joint_angle": j, "ack": True}
        self.conn.send2Arduino(cmd)

    @hlp.timer
    def ik(self, T_06: np.array) -> list:
        """
        arg:
            pose: end-effector pose in cartension space.
            position is retrieve from T_06.
            orientation(j4,5,6) are in deg
        return:
            2 decimaled joints angles in degrees.
        """
        (_, r1, d1) = self.dhTbl[0, :]
        r2 = self.dhTbl[1, 1]
        (_, r3, d3) = self.dhTbl[2, :]
        d4 = self.dhTbl[3, 2]
        d6 = self.dhTbl[5, 2]

        Jik = np.zeros((6,), dtype=np.float64)
        th_offset = np.array([0.0, np.pi / 2, 0.0, 0.0, 0.0, 0.0])

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

            T01 = self.get_ti2i_1(1, Jik[0] + th_offset[0])
            T12 = self.get_ti2i_1(2, Jik[1] + th_offset[1])
            T23 = self.get_ti2i_1(3, Jik[2] + th_offset[2])

            T03 = T01 @ T12 @ T23
            inv_T03 = np.linalg.inv(T03)
            T36 = inv_T03 @ T_06

            Jik[3] = np.arctan2(-T36[1][2], -T36[0][2])
            Jik[4] = np.arctan2(np.sqrt(T36[0][2] ** 2 + T36[1][2] ** 2), T36[2][2])
            Jik[5] = np.arctan2(-T36[2][1], T36[2][0])

            Jik = np.degrees(Jik)
            Jik = np.round_(Jik, decimals=2)
            return Jik

        except ValueError:
            print("Warning: arccos domain error in Joint calculations.")
            return None

    # todo: fk taks care of qs wrt t06 instead of t0-cup. don't do the transformation in fk.
    @hlp.timer
    def fk(self, Jfk) -> np.array:
        """
        Jfk(in deg) - joints value for the calculation of the forward kinematics
        output: Xfk - pos value for the calculation of the forward kinematics
        """
        # Denavit-Hartenberg matrix
        th_offset = np.array([0.0, -90.0, 0.0, 0.0, 0.0, 0.0])
        theta = np.zeros((6,), dtype=np.float64)
        # theta=[Jfk(1); -90+Jfk(2); Jfk(3); Jfk(4); Jfk(5); Jfk(6)];
        theta = np.add(Jfk, th_offset)

        # alfa = self.dhTbl[0:6, 0]
        # from deg to rad
        theta = np.deg2rad(theta)
        # theta = np.deg2rad(Jfk)
        # alfa = np.deg2rad(alfa)

        # work frame
        # Xwf = Point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # tool frame
        # Xtf = Point(0.0, 0.0, 50.0, 180.0, -90.0, 0.0)
        # work frame transformation matrix

        # Twf = Xwf.pos2tran()  # Twf=pos2tran(Xwf);
        # tool frame transformation matrix
        # Tft = Xtf.pos2tran()  # Ttf=pos2tran(Xtf);

        Twf = self.pose2T([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # std_dh_tbl seems no need to give last link's offset for end-effector. todo:to be confirm.
        # Tft = self.pose2T([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])
        Tft = self.pose2T([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])

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
        T = Twf @ T_01 @ T_12 @ T_23 @ T_34 @ T_45 @ T_56 @ Tft
        # print('t: ', np.around(T, 2))
        Xfk = np.zeros((6,), dtype=np.float64)
        # get position from transformation matrix
        (Xfk[0], Xfk[1], Xfk[2]) = T[0:3, 3]

        Xfk[4] = np.arctan2(np.sqrt(T[2, 0] ** 2 + T[2, 1] ** 2), T[2, 2])
        Xfk[3] = np.arctan2(T[1, 2] / np.sin(Xfk[4]), T[0, 2] / np.sin(Xfk[4]))
        Xfk[5] = np.arctan2(T[2, 1] / np.sin(Xfk[4]), -T[2, 0] / np.sin(Xfk[4]))
        # convert to degree
        Xfk[3:6] = np.degrees(Xfk[3:6])
        return Xfk

    # def ik(self, pose: list) -> list:
    #     """
    #     arg:
    #         pose: end-effector pose in cartension space.
    #         position is retrieve from T_06.
    #         orientation(j4,5,6) are in deg
    #     return:
    #         2 decimaled joints angles in degrees.
    #     """
    #     (_, r1, d1) = self.dhTbl[0, :]
    #     r2 = self.dhTbl[1, 1]
    #     (_, r3, d3) = self.dhTbl[2, :]
    #     d4 = self.dhTbl[3, 2]
    #     d6 = self.dhTbl[5, 2]

    #     Jik = np.zeros((6,), dtype=np.float64)
    #     # DH table
    #     # th_offset = np.array([0.0, np.radians(-90), 0.0, 0.0, 0.0, 0.0])
    #     th_offset = np.array([0.0, np.pi / 2, 0.0, 0.0, 0.0, 0.0])

    #     # work frame
    #     # Xwf = Point([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #     # work frame transformation matrix
    #     # Twf = Xwf.pos2tran()
    #     Twf = self.pose2T([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #     # tool frame. end_effector is 50mm align with z6
    #     # Xtf = Point([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])
    #     # tool frame transformation matrix (wrt last joint frame!)
    #     # Ttf = Xtf.pos2tran()

    #     Ttf = self.pose2T([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])

    #     # endEffectorPose = Point([
    #     #    Xik[0], Xik[1], Xik[2], Xik[3], Xik[4], Xik[5]])
    #     # tc_0 transformation matrix
    #     # Twt = endEffectorPose.pos2tran()  # Twt=pos2tran(Xik)
    #     # Twt = T0c
    #     # map tool frame to world frame
    #     Twt = self.pose2T(pose)

    #     # find T06
    #     # inTwf[16], inTtf[16], Tw6[16], T06[16]
    #     invTwf = np.linalg.inv(Twf)  # Tw0 is not take into account now...
    #     invTtf = np.linalg.inv(Ttf)
    #     # Tcw=T0w*T60*Tc6 => T60=invT0w*Tcw*invTc6
    #     # Tw6 = Twt@invTtf
    #     # T06 = invTwf @ Twt @ invTtf

    #     T06=self.pose2T(pose)

    #     # print('T06: ', np.around(T06,2))

    #     # positon of the spherical wrist, and use analytical IK formulas to calculate the first three joint angles based on wrist_position and std_dh params.
    #     wrist_position = [0.0, 0.0, 0.0]
    #     wrist_position = T06[:3, 3] - d6 * T06[:3, 2]
    #     # wrist_position[0] = T06[0][3] - d6 * T06[0][2]
    #     # wrist_position[1] = T06[1][3] - d6 * T06[1][2]
    #     # wrist_position[2] = T06[2][3] - d6 * T06[2][2]

    #     # joints variable
    #     # Jik=zeros(6,1);
    #     # first joint
    #     Jik[0] = np.arctan2(wrist_position[1], wrist_position[0]) - np.arctan2(
    #         d3, np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
    #     )

    #     # second joint
    #     Jik[1] = (
    #         np.pi / 2.0
    #         - np.arccos(
    #             (
    #                 r2**2
    #                 + (wrist_position[2] - d1) * (wrist_position[2] - d1)
    #                 + (
    #                     np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
    #                     - r1
    #                 )
    #                 * (
    #                     np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
    #                     - r1
    #                 )
    #                 - (r3**2 + d4**2)
    #             )
    #             / (
    #                 2.0
    #                 * r2
    #                 * np.sqrt(
    #                     (wrist_position[2] - d1) * (wrist_position[2] - d1)
    #                     + (
    #                         np.sqrt(
    #                             wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2
    #                         )
    #                         - r1
    #                     )
    #                     * (
    #                         np.sqrt(
    #                             wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2
    #                         )
    #                         - r1
    #                     )
    #                 )
    #             )
    #         )
    #         - np.arctan2(
    #             (wrist_position[2] - d1),
    #             (np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2) - r1),
    #         )
    #     )
    #     # Jik(1) = pi/2-acos((r(2) ^ 2+(wrist_position(3)-d(1)) ^ 2+(sqrt(wrist_position(1) ^ 2+wrist_position(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2-(r(3) ^ 2+d(4) ^ 2))/(2*r(2)*sqrt((wrist_position(3)-d(1)) ^ 2+(sqrt(wrist_position(1) ^ 2+wrist_position(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2)))-atan((wrist_position(3)-d(1))/(sqrt(wrist_position(1) ^ 2+wrist_position(2) ^ 2-d(3) ^ 2)-r(1)))

    #     # third joint
    #     Jik[2] = (
    #         np.pi
    #         - np.arccos(
    #             (
    #                 r2**2
    #                 + r3**2
    #                 + d4**2
    #                 - (wrist_position[2] - d1) * (wrist_position[2] - d1)
    #                 - (
    #                     np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
    #                     - r1
    #                 )
    #                 * (
    #                     np.sqrt(wrist_position[0] ** 2 + wrist_position[1] ** 2 - d3**2)
    #                     - r1
    #                 )
    #             )
    #             / (2 * r2 * np.sqrt(r3**2 + d4**2))
    #         )
    #         - np.arctan2(d4, r3)
    #     )
    #     # Jik(2) = pi-acos((r(2) ^ 2+r(3) ^ 2+d(4) ^ 2-(wrist_position(3)-d(1)) ^ 2-(sqrt(wrist_position(1) ^ 2+wrist_position(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2)/(2*r(2)*sqrt(r(3) ^ 2+d(4) ^ 2)))-atan(d(4)/r(3))

    #     # last three joints
    #     T01 = T12 = T23 = T03 = inv_T03 = T36 = np.zeros((4, 4), dtype=np.float64)

    #     T01 = self.get_ti2i_1(1, Jik[0] + th_offset[0])
    #     # todo: note here is a -90 coz of dh table. figure out why
    #     T12 = self.get_ti2i_1(2, Jik[1] + th_offset[1])
    #     T23 = self.get_ti2i_1(3, Jik[2] + th_offset[2])

    #     T03 = T01 @ T12 @ T23
    #     inv_T03 = np.linalg.inv(T03)
    #     # pose of frame6 measured from frame3
    #     T36 = inv_T03 @ T06
    #     # use use trigonometric functions to extract the last three join angles from T36.
    #     # forth joint
    #     Jik[3] = np.arctan2(-T36[1][2], -T36[0][2])

    #     # fifth joint
    #     Jik[4] = np.arctan2(np.sqrt(T36[0][2] ** 2 + T36[1][2] ** 2), T36[2][2])

    #     # sixth joints
    #     Jik[5] = np.arctan2(-T36[2][1], T36[2][0])
    #     # rad to deg
    #     Jik = np.degrees(Jik)
    #     Jik = np.round_(Jik, decimals=2)
    #     return Jik
