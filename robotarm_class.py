# import pandas as pd
from math import pi, cos, acos, sin, atan2, sqrt, radians
# the reason for importing cos and sin from sympy is Rot case (can't import them from math, plz note!!!)
from sympy import symbols, nsimplify, Matrix
import numpy as np
import helpers as hlp
from spatialmath import SE3
from abc import ABC, abstractmethod


def pose2T(pose):
    p = tuple(pose[:3])
    phi, theta, psi = tuple(np.radians(pose[3:]))
    eul = SE3.Rz(phi) * SE3.Ry(theta) * SE3.Rz(psi)
    T = np.eye(4)   # identity matrix, placeholder
    T[:3, :3] = eul.A[:3, :3]
    T[:3, 3] = p
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
        
    def get_ti2i_1(self, i, theta=None) -> np.ndarray:
        ''' return transfermation matrix from dh tbl'''
        # fill in dh tbl wrt robot arms' dh params

        # array idx starts frm 0, so i-1
        alfa, ai, di, th = symbols('alfa, ai, di, th')
        alfa, ai, di = self.dhTbl[i - 1, :]
        if theta is None:
            th = f'q{i}'
        else:
            th = theta

        # the reason for using sympy's Matrix is that we need to apply it with sympy's simplify func
        # to eliminate sth likes 1.xxxxxe-14 * sin(qx)
        # Ti_2_i-1=Tzi-1(thetai)TZr(di)TXq(ai)TXp(alphai)
        # this matrix is transformation matrix for std dh table
        m = Matrix([[cos(th), -sin(th)*cos(alfa), sin(th)*sin(alfa), ai*cos(th)],
                    [sin(th), cos(th)*cos(alfa), -
                     cos(th)*sin(alfa), ai*sin(th)],
                    [0, sin(alfa), cos(alfa), di],
                    [0, 0, 0, 1]])

        if theta is None:
            m = nsimplify(m, tolerance=1e-10, rational=True)
            # print(f't{i}-{i-1}: {m}')
            return np.array(m)
        else:
            # Matrix objects have a numpy method that returns a numpy.ndarray
            return np.array(m).astype(np.float64)
        #return m

    @abstractmethod      
    def ik(self): pass    

    @abstractmethod    
    def fk(self, Jfk): pass

    @abstractmethod       
    def moveTo(self, end_effector_pose): pass

class SmallRbtArm(RobotArm):
    def __init__(self, std_dh_tbl: np.ndarray):
        super().__init__(std_dh_tbl)

    @hlp.timer  
    def ik(self, Xik):
        '''
        Xik: end-effector pose in cartension space, orientation(j4,5,6) are in deg
        return: 2 decimaled joints angles in degrees.
        '''
        (_, r1, d1) = self.dhTbl[0, :]
        r2 = self.dhTbl[1, 1]
        (_, r3, d3) = self.dhTbl[2, :]
        d4 = self.dhTbl[3, 2]
        d6 = self.dhTbl[5, 2]

        Jik = np.zeros((6,), dtype=float)
        # DH table
        th_offset = np.array([0.0, radians(-90), 0.0, 0.0, 0.0, 0.0])

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
        invTwf = np.linalg.inv(Twf) # Tw0 is not take into account now...
        invTtf = np.linalg.inv(Ttf)
        # Tcw=T0w*T60*Tc6 => T60=invT0w*Tcw*invTc6
        # Tw6 = Twt@invTtf
        T06 = invTwf@Twt@invTtf

        # print('T06: ', np.around(T06,2))

        # positon of the spherical wrist
        Xsw = [0.0, 0.0, 0.0]
        # Xsw=T06(1:3,4)-d(6)*T06(1:3,3);
        Xsw[0] = T06[0][3]-d6*T06[0][2]
        Xsw[1] = T06[1][3]-d6*T06[1][2]
        Xsw[2] = T06[2][3]-d6*T06[2][2]

        # joints variable
        # Jik=zeros(6,1);
        # first joint
        Jik[0] = atan2(Xsw[1], Xsw[0]) - \
            atan2(d3, sqrt(Xsw[0]**2+Xsw[1]**2-d3**2))

        # second joint
        Jik[1] = pi/2.0 - acos((r2**2 + (Xsw[2] - d1) * (Xsw[2] - d1) + (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1) * (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1) - (r3**2 + d4**2)) / (2.0 * r2 * sqrt(
            (Xsw[2] - d1) * (Xsw[2] - d1) + (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1) * (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1)))) - atan2((Xsw[2] - d1), (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1))
        # Jik(1) = pi/2-acos((r(2) ^ 2+(Xsw(3)-d(1)) ^ 2+(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2-(r(3) ^ 2+d(4) ^ 2))/(2*r(2)*sqrt((Xsw(3)-d(1)) ^ 2+(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)))

        # third joint
        Jik[2] = pi - acos((r2**2 + r3**2 + d4**2 - (Xsw[2] - d1) * (Xsw[2] - d1) - (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) -
                                                                                    r1) * (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1)) / (2 * r2 * sqrt(r3**2 + d4**2))) - atan2(d4, r3)
        # Jik(2) = pi-acos((r(2) ^ 2+r(3) ^ 2+d(4) ^ 2-(Xsw(3)-d(1)) ^ 2-(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2)/(2*r(2)*sqrt(r(3) ^ 2+d(4) ^ 2)))-atan(d(4)/r(3))

        # last three joints
        T01 = T12 = T23 = T03 = inv_T03 = T36 = np.zeros(
            (4, 4), dtype=float)

        T01 = self.get_ti2i_1(1, Jik[0]+th_offset[0])
        # note here is a -90 coz of dh table. figure out why
        T12 = self.get_ti2i_1(2, Jik[1]+th_offset[1])
        T23 = self.get_ti2i_1(3, Jik[2]+th_offset[2])

        T03 = T01@T12@T23
        inv_T03 = np.linalg.inv(T03)
        # pose of frame6 measured from frame3
        T36 = inv_T03@T06

        # forth joint
        Jik[3] = atan2(-T36[1][2], -T36[0][2])

        # fifth joint
        Jik[4] = atan2(sqrt(T36[0][2]**2 + T36[1][2]**2), T36[2][2])

        # sixth joints
        Jik[5] = atan2(-T36[2][1], T36[2][0])
        # rad to deg
        Jik = np.degrees(Jik)
        Jik = np.round_(Jik, decimals=2)
        return Jik

    @hlp.timer
    def fk(self, Jfk):
        '''
        Jfk(in deg) - joints value for the calculation of the forward kinematics
        output: Xfk - pos value for the calculation of the forward kinematics
        '''
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
        T = Twf@T10@T21@T32@T43@T54@T65@Tft
        # print('t: ', np.around(T, 2))
        Xfk = np.zeros((6,), dtype=float)
        # calculate pos from transformation matrix
        (Xfk[0], Xfk[1], Xfk[2]) = T[0:3, 3]

        Xfk[4] = atan2(sqrt(T[2, 0]**2 + T[2, 1]**2), T[2, 2])
        Xfk[3] = atan2(T[1, 2]/sin(Xfk[4]), T[0, 2]/sin(Xfk[4]))
        Xfk[5] = atan2(T[2, 1]/sin(Xfk[4]), -T[2, 0]/sin(Xfk[4]))
        Xfk[3:6] = np.degrees(Xfk[3:6])
        return Xfk
    
    
    def send2Arduino(ser, header: str, j, bWaitAck: bool):
        global event_ok2send

        """send robot cmd to arduino

        Args:
            ser (_type_): _description_
            header (str): cmd type
            j (float): theta in deg for 6 axes
            bWaitAck (bool): wait for ack from arduino or not
        """
        # msg = f'{header}{j[0]:.2f},{j[1]:.2f},{j[2]:.2f},{j[3]:.2f},{j[4]:.2f},{j[5]:.2f}\n'
        msg = '{}{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n'.format(header, *j,)
        ser.write(msg.encode('utf-8'))
        event_ok2send.clear()
        print(msg)
        if bWaitAck is True:
            # wait till the event is set in rcvThread.
            event_ok2send.wait()
        # while event_ack.is_set() and bWaitAck is True:
        #    pass

    def moveTo(self, end_effector_pose):
        #return super().moveTo(end_effector_pose)
        j = self.ik(end_effector_pose)
        self.send2Arduino('j', j, bWaitAck=True)
