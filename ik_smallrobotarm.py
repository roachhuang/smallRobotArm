
"""
to do:
    1. send ik result to arduino from py
    2. arduino Xx[0]=xfi[0]+l
    3. trajectory plan
    4. ....
"""

import numpy as np
from numpy.linalg import inv
from math import pi, cos, acos, sin, atan2, sqrt, radians
import std_dh_tbl as stdDh


def InverseK(Xik):
    ''' Xik: points in cartension space
        return: 2 decimaled joints angles in degrees.
    '''
    (alp1, r1, d1) = stdDh.dh_tbl[0, :]
    (alp2, r2, d2) = stdDh.dh_tbl[1, :]
    (alp3, r3, d3) = stdDh.dh_tbl[2, :]
    (alp4, r4, d4) = stdDh.dh_tbl[3, :]
    (alp5, r5, d5) = stdDh.dh_tbl[4, :]
    (alp6, r6, d6) = stdDh.dh_tbl[5, :]

    # r1, r2, r3 = 47.0, 110.0, 26.0
    # d1, d3, d4, d6 = 133.0, 0.0, 117.50, 28.0
    r = [r1, r2, r3, 0.0, 0.0, 0.0]  # r=[47; 110; 26; 0; 0; 0]
    d = [d1, 0.0, d3, d4, 0.0, d6]  # d=[133; 0; 7; 117.5; 0; 28]

    Jik = np.zeros((6,), dtype=float)
    # inverse kinematics
    # input: Xik - pos value for the calculation of the inverse kinematics
    # output: Jfk - joints value for the calculation of the inversed kinematics

    # from deg to rad
    # Xik(4:6)=Xik(4:6)*math.pi/180;
    # deg to rad
    Xik[3] = radians(Xik[3])
    Xik[4] = radians(Xik[4])
    Xik[5] = radians(Xik[5])

    # work frame
    Xwf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Xwf=[0; 0; 0; 0; 0; 0]

    # tool frame
    Xtf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Xtf=[0; 0; 0; 0; 0; 0]

    # work frame transformation matrix
    # Twf[16]
    Twf = pos2tran(Xwf)  # Twf=pos2tran(Xwf)

    # tool frame transformation matrix
    # Ttf[16]
    Ttf = pos2tran(Xtf)  # Ttf=pos2tran(Xtf)
    # total transformation matrix
    # Twt[16]
    Twt = pos2tran(Xik)  # Twt=pos2tran(Xik)

    # find T06
    # inTwf[16], inTtf[16], Tw6[16], T06[16]
    inTwf = inv(Twf)
    inTtf = inv(Ttf)
    Tw6 = Twt@inTtf
    T06 = inTwf@Tw6

    # positon of the spherical wrist
    Xsw = [0.0, 0.0, 0.0]
    # Xsw=T06(1:3,4)-d(6)*T06(1:3,3);
    Xsw[0] = T06[0][3]-d6*T06[0][2]
    Xsw[1] = T06[1][3]-d6*T06[1][2]
    Xsw[2] = T06[2][3]-d6*T06[2][2]

    # joints variable
    # Jik=zeros(6,1);
    # first joint
    Jik[0] = atan2(Xsw[1], Xsw[0])-atan2(d3,
                                         sqrt(Xsw[0]**2+Xsw[1]**2-d3**2))

    # second joint
    Jik[1] = pi/2.0 - acos((r2**2 + (Xsw[2] - d1) * (Xsw[2] - d1) + (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1) * (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1) - (r3**2 + d4**2)) / (2.0 * r2 * sqrt(
        (Xsw[2] - d1) * (Xsw[2] - d1) + (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1) * (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1)))) - atan2((Xsw[2] - d1), (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1))
    # Jik(1) = pi/2-acos((r(2) ^ 2+(Xsw(3)-d(1)) ^ 2+(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2-(r(3) ^ 2+d(4) ^ 2))/(2*r(2)*sqrt((Xsw(3)-d(1)) ^ 2+(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)))

    # third joint
    Jik[2] = pi - acos((r2**2 + r3**2 + d4**2 - (Xsw[2] - d1) * (Xsw[2] - d1) - (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) -
                                                                                 r1) * (sqrt(Xsw[0]**2 + Xsw[1]**2 - d3**2) - r1)) / (2 * r2 * sqrt(r3**2 + d4**2))) - atan2(d4, r3)
    # Jik(2) = pi-acos((r(2) ^ 2+r(3) ^ 2+d(4) ^ 2-(Xsw(3)-d(1)) ^ 2-(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2)/(2*r(2)*sqrt(r(3) ^ 2+d(4) ^ 2)))-atan(d(4)/r(3))

    # last three joints
    T01 = T12 = T23 = T02 = T03 = inT03 = T36 = np.zeros((4, 4), dtype=float)

    T01 = stdDh.get_ti2i_1(1, Jik[0])
    # note here is a -90 coz of dh table. figure out why
    T12 = stdDh.get_ti2i_1(2, radians(-90)+Jik[1])
    T23 = stdDh.get_ti2i_1(3, Jik[2])

    T03 = T01@T12@T23
    inT03 = inv(T03)
    T36 = inT03@T06

    # forth joint
    Jik[3] = atan2(-T36[1][2], -T36[0][2])

    # fifth joint
    Jik[4] = atan2(sqrt(T36[0][2]**2 + T36[1][2]**2), T36[2][2])

    # sixth joints
    Jik[5] = atan2(-T36[2][1], T36[2][0])
    # rad to deg
    # MatrixScale(Jik, 6, 1, 180.0/math.pi)  # Jik=Jik/math.pi*180
    Jik = np.degrees(Jik)
    Jik = np.round_(Jik, decimals=2)
    return Jik


def pos2tran(Xpt):
    ''' convert the end point to transfomation matrix '''
    # input 3:6 in radians
    Tpt = np.ndarray(shape=(4, 4), dtype=float)
    # pos to homogeneous transformation matrix
    # note that 3:6 are zyz eluer
    tx, ty, tz = Xpt[3:6]
    Tpt[:3, :3] = stdDh.Rot('z', tx) @ stdDh.Rot('y', ty) @ stdDh.Rot(
        'z', tz)  # rotation matrix
    Tpt[:3, 3] = Xpt[0:3]  # x,y,z
    Tpt[3, :] = [0, 0, 0, 1]
    return Tpt
