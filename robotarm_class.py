# the reason for importing cos and sin from sympy is Rot case (can't import them from math, plz note!!!)
from sympy import symbols  # , nsimplify, Matrix
import numpy as np
import helpers as hlp

# from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
import serial_class as ser
from abc import ABC, abstractmethod

"""
def pose2T(pose):
    p = tuple(pose[:3])
    phi, theta, psi = tuple(np.radians(pose[3:]))
    # Creates a rotation matrix using
    # spatialmath's SE3 class and Euler angle rotations (Rz, Ry, Rz).
    eul = SE3.Rz(phi) * SE3.Ry(theta) * SE3.Rz(psi)
    T = np.eye(4)  # identity matrix, placeholder
    # inserts the rotation matrix into the top-left 3x3 portion of T.
    T[:3, :3] = eul.A[:3, :3]
    # inserts the translation vector into the top-right 3x1 portion of T
    T[:3, 3] = p
    return T
"""

"""
Euler Sequence:
Note that the original code used a ZYZ euler sequence, and my code uses XYZ. If your data is based on ZYZ, you will need to change the 'xyz' inside of the R.from_euler function to 'zyz'.
Rotate around the initial Z-axis.
Rotate around the new Y-axis (which has been rotated).
Rotate around the newest Z-axis (which has been rotated twice).
zyz
"""


def pose2T(pose):
    """Converts a pose (x, y, z, roll, pitch, yaw) to a 4x4 homogeneous transformation matrix."""
    x, y, z, roll, pitch, yaw = pose

    # Convert angles to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    # Create rotation matrix using scipy.spatial.transform.Rotation
    r = R.from_euler("xyz", [roll, pitch, yaw])
    rotation_matrix = r.as_matrix()

    # Create homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = [x, y, z]

    return T


"""
This notation can sometimes be used to emphasize that the rotations are happening in a specific order, and can be used to describe either intrinsic or extrinsic rotations, but often when people write Z-Y-Z they are talking about extrinsic rotations.
Extrinsic rotations mean.

    Rotate around the initial Z-axis.
    Rotate around the initial Y-axis.
    Rotate around the initial Z-axis.

So the rotations are all happening around the same global axis.
Uses the Euler angle sequence 'z-y-z'
"""
def create_T_scipy(pose):
    """Creates SE3 transformation matrix from pose using scipy."""

    x, y, z, roll, pitch, yaw = pose

    # Translation matrix
    translation_matrix = np.eye(4)
    translation_matrix[:3, 3] = [x, y, z]

    # Rotation matrices
    r_z1 = R.from_euler("z", np.radians(roll))
    r_y = R.from_euler("y", np.radians(pitch))
    r_z2 = R.from_euler("z", np.radians(yaw))

    rotation_matrix = (r_z1 * r_y * r_z2).as_matrix()

    # Homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = [x, y, z]

    # Alternatively, you can multiply the translation and rotation matrix.
    # T = translation_matrix @ np.eye(4)
    # T[:3,:3] = rotation_matrix

    return T


def euler_zyz_from_matrix(matrix):
    R = matrix[:3, :3]
    phi = np.arctan2(R[1, 0], R[0, 0])
    theta = np.arccos(R[2, 0])
    psi = np.arctan2(R[2, 1], R[2, 2])
    return phi, theta, psi


class RobotArm(ABC):
    def __init__(self, std_dh_tbl: np.ndarray):
        self.dhTbl = std_dh_tbl
        # self.ser = com.init_ser()

    def get_ti2i_1(self, i, theta=None) -> np.ndarray:
        """
        todo: examin intpu theta's procision by checking its number of decimal points.
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
        th=theta

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

        if theta is None:
            # m = nsimplify(m, tolerance=1e-10, rational=True)
            # # print(f't{i}-{i-1}: {m}')
            # return np.array(m)
            # ------------------------------------------------------------
            # For symbolic-like behavior (approximation with rationals)
            # NumPy doesn't have a direct equivalent to nsimplify with rational=True
            # This part requires more advanced techniques or a different library if you need exact rational approximations.
            # Here we approximate with floats and round to a certain tolerance.
            decimals = int(-np.log10(1e-8)) # calculates the number of decimals needed
            # 6-10 decimals is common for robotics, we use 8 here.
            m = np.round(
                m, decimals
            )  # approximates the symbolic nsimplify tolerance.
            return m
        else:
            # Matrix objects have a numpy method that returns a numpy.ndarray
            # float32 maintains only approximately 7 decimal digits fo precision internally.
            return m.astype(np.float64)
        # return m

    @abstractmethod
    def ik(self):
        pass

    @abstractmethod
    def fk(self, Jfk):
        pass

    @abstractmethod
    def moveTo(self, end_effector_pose):
        pass


class SmallRbtArm(RobotArm):
    def __init__(self, std_dh_tbl: np.ndarray):
        super().__init__(std_dh_tbl)
        self.conn = ser.SerialPort()
        self.conn.connect()

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

    def moveTo(self, end_effector_pose):
        # return super().moveTo(end_effector_pose)
        j = self.ik(end_effector_pose)
        cmd = {"header": "j", "joint_angle": j, "ack": True}
        self.conn.send2Arduino(cmd)

    @hlp.timer
    def ik(self, Xik: list):
        """
        Xik: end-effector pose in cartension space, orientation(j4,5,6) are in deg
        return: 2 decimaled joints angles in degrees.
        """
        (_, r1, d1) = self.dhTbl[0, :]
        r2 = self.dhTbl[1, 1]
        (_, r3, d3) = self.dhTbl[2, :]
        d4 = self.dhTbl[3, 2]
        d6 = self.dhTbl[5, 2]

        Jik = np.zeros((6,), dtype=float)
        # DH table
        th_offset = np.array([0.0, np.radians(-90), 0.0, 0.0, 0.0, 0.0])

        # work frame
        # Xwf = Point([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # work frame transformation matrix
        # Twf = Xwf.pos2tran()
        Twf = pose2T([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # tool frame. end_effector is 50mm align with z6
        # Xtf = Point([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])
        # tool frame transformation matrix (wrt last joint frame!)
        # Ttf = Xtf.pos2tran()
        Ttf = pose2T([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])
        # endEffectorPose = Point([
        #    Xik[0], Xik[1], Xik[2], Xik[3], Xik[4], Xik[5]])
        # tc_0 transformation matrix
        # Twt = endEffectorPose.pos2tran()  # Twt=pos2tran(Xik)
        # Twt = T0c
        Twt = pose2T(Xik)

        # find T06
        # inTwf[16], inTtf[16], Tw6[16], T06[16]
        invTwf = np.linalg.inv(Twf)  # Tw0 is not take into account now...
        invTtf = np.linalg.inv(Ttf)
        # Tcw=T0w*T60*Tc6 => T60=invT0w*Tcw*invTc6
        # Tw6 = Twt@invTtf
        T06 = invTwf @ Twt @ invTtf

        # print('T06: ', np.around(T06,2))

        # positon of the spherical wrist
        Xsw = [0.0, 0.0, 0.0]
        # Xsw=T06(1:3,4)-d(6)*T06(1:3,3);
        Xsw[0] = T06[0][3] - d6 * T06[0][2]
        Xsw[1] = T06[1][3] - d6 * T06[1][2]
        Xsw[2] = T06[2][3] - d6 * T06[2][2]

        # joints variable
        # Jik=zeros(6,1);
        # first joint
        Jik[0] = np.arctan2(Xsw[1], Xsw[0]) - np.arctan2(
            d3, np.sqrt(Xsw[0] ** 2 + Xsw[1] ** 2 - d3**2)
        )

        # second joint
        Jik[1] = (
            np.pi / 2.0
            - np.arccos(
                (
                    r2**2
                    + (Xsw[2] - d1) * (Xsw[2] - d1)
                    + (np.sqrt(Xsw[0] ** 2 + Xsw[1] ** 2 - d3**2) - r1)
                    * (np.sqrt(Xsw[0] ** 2 + Xsw[1] ** 2 - d3**2) - r1)
                    - (r3**2 + d4**2)
                )
                / (
                    2.0
                    * r2
                    * np.sqrt(
                        (Xsw[2] - d1) * (Xsw[2] - d1)
                        + (np.sqrt(Xsw[0] ** 2 + Xsw[1] ** 2 - d3**2) - r1)
                        * (np.sqrt(Xsw[0] ** 2 + Xsw[1] ** 2 - d3**2) - r1)
                    )
                )
            )
            - np.arctan2((Xsw[2] - d1), (np.sqrt(Xsw[0] ** 2 + Xsw[1] ** 2 - d3**2) - r1))
        )
        # Jik(1) = pi/2-acos((r(2) ^ 2+(Xsw(3)-d(1)) ^ 2+(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2-(r(3) ^ 2+d(4) ^ 2))/(2*r(2)*sqrt((Xsw(3)-d(1)) ^ 2+(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)))

        # third joint
        Jik[2] = (
            np.pi
            - np.arccos(
                (
                    r2**2
                    + r3**2
                    + d4**2
                    - (Xsw[2] - d1) * (Xsw[2] - d1)
                    - (np.sqrt(Xsw[0] ** 2 + Xsw[1] ** 2 - d3**2) - r1)
                    * (np.sqrt(Xsw[0] ** 2 + Xsw[1] ** 2 - d3**2) - r1)
                )
                / (2 * r2 * np.sqrt(r3**2 + d4**2))
            )
            - np.arctan2(d4, r3)
        )
        # Jik(2) = pi-acos((r(2) ^ 2+r(3) ^ 2+d(4) ^ 2-(Xsw(3)-d(1)) ^ 2-(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2)/(2*r(2)*sqrt(r(3) ^ 2+d(4) ^ 2)))-atan(d(4)/r(3))

        # last three joints
        T01 = T12 = T23 = T03 = inv_T03 = T36 = np.zeros((4, 4), dtype=float)

        T01 = self.get_ti2i_1(1, Jik[0] + th_offset[0])
        # note here is a -90 coz of dh table. figure out why
        T12 = self.get_ti2i_1(2, Jik[1] + th_offset[1])
        T23 = self.get_ti2i_1(3, Jik[2] + th_offset[2])

        T03 = T01 @ T12 @ T23
        inv_T03 = np.linalg.inv(T03)
        # pose of frame6 measured from frame3
        T36 = inv_T03 @ T06

        # forth joint
        Jik[3] = np.arctan2(-T36[1][2], -T36[0][2])

        # fifth joint
        Jik[4] = np.arctan2(np.sqrt(T36[0][2] ** 2 + T36[1][2] ** 2), T36[2][2])

        # sixth joints
        Jik[5] = np.arctan2(-T36[2][1], T36[2][0])
        # rad to deg
        Jik = np.degrees(Jik)
        Jik = np.round_(Jik, decimals=2)
        return Jik

    @hlp.timer
    def fk(self, Jfk):
        """
        Jfk(in deg) - joints value for the calculation of the forward kinematics
        output: Xfk - pos value for the calculation of the forward kinematics
        """
        # Denavit-Hartenberg matrix
        th_temp = np.array([0.0, -90.0, 0.0, 0.0, 0.0, 0.0])
        theta = np.zeros((6,), dtype=float)
        # theta=[Jfk(1); -90+Jfk(2); Jfk(3); Jfk(4); Jfk(5); Jfk(6)];
        theta = np.add(Jfk, th_temp)

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

        Twf = pose2T([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        Tft = pose2T([0.0, 0.0, 50.0, 180.0, -90.0, 0.0])

        # DH homogeneous transformation matrix
        # float T01[16], T12[16], T23[16], T34[16], T45[16], T56[16];
        # T 1 到 0 (1在下, 0在上).
        # #1, frame1 相對于frame0 的空間幾何關係; #2,frame1 下表達的vector可轉囘frame0下來表達.
        # #3, 從 frame0 來看 frame1.
        T10 = self.get_ti2i_1(1, theta[0])
        # T 2 到 1
        T21 = self.get_ti2i_1(2, theta[1])
        T32 = self.get_ti2i_1(3, theta[2])
        T43 = self.get_ti2i_1(4, theta[3])
        T54 = self.get_ti2i_1(5, theta[4])
        T65 = self.get_ti2i_1(6, theta[5])
        T = Twf @ T10 @ T21 @ T32 @ T43 @ T54 @ T65 @ Tft
        # print('t: ', np.around(T, 2))
        Xfk = np.zeros((6,), dtype=float)
        # calculate pos from transformation matrix
        (Xfk[0], Xfk[1], Xfk[2]) = T[0:3, 3]

        Xfk[4] = np.arctan2(np.sqrt(T[2, 0] ** 2 + T[2, 1] ** 2), T[2, 2])
        Xfk[3] = np.arctan2(T[1, 2] / np.sin(Xfk[4]), T[0, 2] / np.sin(Xfk[4]))
        Xfk[5] = np.arctan2(T[2, 1] / np.sin(Xfk[4]), -T[2, 0] / np.sin(Xfk[4]))
        Xfk[3:6] = np.degrees(Xfk[3:6])
        return Xfk
