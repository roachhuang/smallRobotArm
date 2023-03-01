
import numpy as np
from numpy.linalg import inv
# import math
from math import pi, cos, acos, sin, atan, atan2, sqrt, radians
# import pandas as pd
import std_dh_tbl as stdDh


def InverseK(Xik):
    r1, r2, r3 = 47.0, 110.0, 26.0
    d1, d3, d4, d6 = 133.0, 0.0, 117.50, 28.0
    Jik = np.zeros((6,), dtype=float)
    # inverse kinematics
    # input: Xik - pos value for the calculation of the inverse kinematics
    # output: Jfk - joints value for the calculation of the inversed kinematics

    # from deg to rad
    # Xik(4:6)=Xik(4:6)*math.pi/180;
    # deg to rad
    Xik[3] = Xik[3]*pi/180.0
    Xik[4] = Xik[4]*pi/180.0
    Xik[5] = Xik[5]*pi/180.0

    # Denavit-Hartenberg matrix
    # theta=[0; -90+0; 0; 0; 0; 0]
    theta = np.array([0.0, -90.0, 0.0, 0.0, 0.0, 0.0])
    # alfa=[-90; 0; -90; 90; -90; 0];
    alfa = np.array([-90.0, 0.0, -90.0, 90.0, -90.0, 0.0])
    r = [r1, r2, r3, 0.0, 0.0, 0.0]  # r=[47; 110; 26; 0; 0; 0]
    d = [d1, 0.0, d3, d4, 0.0, d6]  # d=[133; 0; 7; 117.5; 0; 28]
    # from deg to rad
    theta = np.deg2rad(theta)
    alfa = np.deg2rad(alfa)

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
    Xsw[0] = T06[0][3]-d[5]*T06[0][2]
    Xsw[1] = T06[1][3]-d[5]*T06[1][2]
    Xsw[2] = T06[2][3]-d[5]*T06[2][2]

    # joints variable
    # Jik=zeros(6,1);
    # first joint
    Jik[0] = atan2(Xsw[1], Xsw[0])-atan2(d[2],
                                         sqrt(Xsw[0]**2+Xsw[1]**2-d[2]**2))

    # second joint
    Jik[1] = pi/2.0 - acos((r[1] * r[1] + (Xsw[2] - d[0]) * (Xsw[2] - d[0]) + (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - d[2] * d[2]) - r[0]) * (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - d[2] * d[2]) - r[0]) - (r[2] * r[2] + d[3] * d[3])) / (2.0 * r[1] * sqrt(
        (Xsw[2] - d[0]) * (Xsw[2] - d[0]) + (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - d[2] * d[2]) - r[0]) * (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - d[2] * d[2]) - r[0])))) - atan((Xsw[2] - d[0]) / (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - d[2] * d[2]) - r[0]))
    # Jik(1) = pi/2-acos((r(2) ^ 2+(Xsw(3)-d(1)) ^ 2+(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2-(r(3) ^ 2+d(4) ^ 2))/(2*r(2)*sqrt((Xsw(3)-d(1)) ^ 2+(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)))

    # third joint
    Jik[2] = pi - acos((r[1] * r[1] + r[2] * r[2] + d[3] * d[3] - (Xsw[2] - d[0]) * (Xsw[2] - d[0]) - (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - d[2] * d[2]) -
                         r[0]) * (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - d[2] * d[2]) - r[0])) / (2 * r[1] * sqrt(r[2] * r[2] + d[3] * d[3]))) - atan(d[3] / r[2])
    # Jik(2) = pi-acos((r(2) ^ 2+r(3) ^ 2+d(4) ^ 2-(Xsw(3)-d(1)) ^ 2-(sqrt(Xsw(1) ^ 2+Xsw(2) ^ 2-d(3) ^ 2)-r(1)) ^ 2)/(2*r(2)*sqrt(r(3) ^ 2+d(4) ^ 2)))-atan(d(4)/r(3))

    # last three joints
    T01 = T12 = T23 = T02 = T03 = inT03 = T36 = np.zeros((4, 4), dtype=float)

    # T01=DH1line(theta(1)+Jik(1),alfa(1),r(1),d(1));
    T01 = DH1line(theta[0]+Jik[0], alfa[0], r[0], d[0])
    # T12=DH1line(theta(2)+Jik(2),alfa(2),r(2),d(2));
    T12 = DH1line(theta[1]+Jik[1], alfa[1], r[1], d[1])
    # T23=DH1line(theta(3)+Jik(3),alfa(3),r(3),d(3));
    T23 = DH1line(theta[2]+Jik[2], alfa[2], r[2], d[2])
    T02 = T01@T12
    T03 = T02@T23
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
    # Jik = np.degrees(Jik)
    return Jik

def DH1line(thetadh, alfadh, rdh, ddh):
    # creats Denavit-Hartenberg homogeneous transformation matrix
    # first row
    Tdh = np.zeros((4, 4), dtype=float)
    Tdh[0][0] = cos(thetadh)
    Tdh[0][1] = -sin(thetadh)*cos(alfadh)
    Tdh[0][2] = sin(thetadh)*sin(alfadh)
    Tdh[0][3] = rdh*cos(thetadh)
    # second row
    Tdh[1][0] = sin(thetadh)
    Tdh[1][1] = cos(thetadh)*cos(alfadh)
    Tdh[1][2] = -cos(thetadh)*sin(alfadh)
    Tdh[1][3] = rdh*sin(thetadh)
    # third row
    Tdh[2][0] = 0.0
    Tdh[2][1] = sin(alfadh)
    Tdh[2][2] = cos(alfadh)
    Tdh[2][3] = ddh
    # forth row
    Tdh[3][0] = 0.0
    Tdh[3][1] = 0.0
    Tdh[3][2] = 0.0
    Tdh[3][3] = 1.0
    return Tdh
# position to transformation matrix

def pos2tran(Xpt):
    Tpt = np.ndarray(shape=(4, 4), dtype=float)
    # pos to homogeneous transformation matrix
    # first row
    Tpt[0][0] = cos(Xpt[3])*cos(Xpt[4])*cos(Xpt[5])-sin(Xpt[3])*sin(Xpt[5])
    Tpt[0][1] = -cos(Xpt[3])*cos(Xpt[4])*sin(Xpt[5])-sin(Xpt[3])*cos(Xpt[5])
    Tpt[0][2] = cos(Xpt[3])*sin(Xpt[4])
    Tpt[0][3] = Xpt[0]
    # second row
    Tpt[1][0] = sin(Xpt[3])*cos(Xpt[4])*cos(Xpt[5])+cos(Xpt[3])*sin(Xpt[5])
    Tpt[1][1] = -sin(Xpt[3])*cos(Xpt[4])*sin(Xpt[5])+cos(Xpt[3])*cos(Xpt[5])
    Tpt[1][2] = sin(Xpt[3])*sin(Xpt[4])
    Tpt[1][3] = Xpt[1]
    # third row
    Tpt[2][0] = -sin(Xpt[4])*cos(Xpt[5])
    Tpt[2][1] = sin(Xpt[4])*sin(Xpt[5])
    Tpt[2][2] = cos(Xpt[4])
    Tpt[2][3] = Xpt[2]
    # forth row
    Tpt[3][0] = 0.0
    Tpt[3][1] = 0.0
    Tpt[3][2] = 0.0
    Tpt[3][3] = 1.0
    return Tpt

def main():
    t6_0 = np.empty(shape=[4, 4])  # p0 to pf

    dh_tbl = np.array([[radians(-90), 47, 133], [0, 110, 0], [radians(-90), 26, 0],
                       [radians(90), 0, 117.5], [radians(-90), 0, 0],
                       [0, 0, 28]])

    stdDh.setDhTbl(dh_tbl)
    # p = (x, y, z, ZYZ euler angles)
    P = np.array([[164.5, 0.0, 241.0, 90.0, 180.0, -90.0],
                 [164.5, 0.0, 141.0, 90.0, 180.0, -90.0],
                 [164.5+14.7, 35.4, 141.0, 90.0, 180.0, -90.0],
                 [164.5+50.0, 50.0, 141.0, 90.0, 180.0, -90.0],
                 [164.5+85.3, 35.4, 141.0, 90.0, 180.0, -90.0],
                 [164.5+100.0, 0.0, 141.0, 90.0, 180.0, -90.0],
                 [164.5+85.3, -35.4, 141.0, 90.0, 180.0, -90.0],
                 [164.5+50.0, -50.0, 141.0, 90.0, 180.0, -90.0],
                 [164.5+14.7, -35.4, 141.0, 90.0, 180.0, -90.0]])
    '''
    P = np.array([
        [164.5, 0.0, 200.0, 90.0, 180.0, -90.0],
        [164.5+14.7, 35.4, 141.0, 90.0, 180.0, -90.0],
        [164.5+100.0, 0.0, 141.0, 90.0, 180.0, -90.0],
        [264.5, 0.0, 141.0, 0.0, 90.0, 0.0]
    ])
    '''
    np.set_printoptions(precision=4, suppress=True)
    J = np.zeros((6,), dtype=float)

    for p in P:
        tx, ty, tz = np.radians(p[3:6])
        t6_0[:3, :3] = stdDh.Rot('z', tx) @ stdDh.Rot('y', ty) @ stdDh.Rot(
            'z', tz)  # rotation matrix
        t6_0[:3, 3] = p[0:3]  # x,y,z
        t6_0[3, :] = [0, 0, 0, 1]
        J = InverseK(p)
        print(J.round(2))

        ###########################################

        # combine rot and translation vector into transformation matrix

        t1_0 = stdDh.get_ti2i_1(1, J[0])
        t2_1 = stdDh.get_ti2i_1(2, radians(-90)+J[1])
        t3_2 = stdDh.get_ti2i_1(3, J[2])
        # t3_2 = t3_2.subs(q3, t3)

        # t4_0 = t1_0 * t2_1 * t3_2 * t4_3
        t3_0 = t1_0 @ t2_1 @ t3_2
        inv_t3_0 = inv(t3_0)

        t4_3 = stdDh.get_ti2i_1(4)
        t5_4 = stdDh.get_ti2i_1(5)
        t6_5 = stdDh.get_ti2i_1(6)

        #t6_3 = t4_3 @ t5_4 @ t6_5
        t6_3 = inv_t3_0 @ t6_0
        t6=atan2(-(t6_3[2][1]), t6_3[2][0])
        print(t6)
        ######################################


    # [x, y, z, ZYZ Euler angles]
    Xhome = [164.5, 0.0, 241.0, 90.0, 180.0, -90.0]
    X1 = [164.5, 0.0, 141.0, 90.0, 180.0, -90.0]
    X11 = [164.5+14.7, 35.4, 141.0, 90.0, 180.0, -90.0]
    X12 = [164.5+50.0, 50.0, 141.0, 90.0, 180.0, -90.0]
    X13 = [164.5+85.3, 35.4, 141.0, 90.0, 180.0, -90.0]
    X14 = [164.5+100.0, 0.0, 141.0, 90.0, 180.0, -90.0]
    X15 = [164.5+85.3, -35.4, 141.0, 90.0, 180.0, -90.0]
    X16 = [164.5+50.0, -50.0, 141.0, 90.0, 180.0, -90.0]
    X17 = [164.5+14.7, -35.4, 141.0, 90.0, 180.0, -90.0]

    X18 = [164.5+50.0, 0.0, 141.0, 90.0, 180.0, -90.0]

    X2 = [264.5, 0.0, 141.0, 0.0, 90.0, 0.0]
    X3 = [164.5, 100.0, 141.0, 90.0, 90.0, 0.0]
    X4 = [164.5, -100.0, 141.0, 90.0, -90.0, 0.0]
    '''
    # Jhome[6]
    J = InverseK(Xhome)
    print(J)
    J = InverseK(X1)
    print(J)
    J = InverseK(X11)
    print(J)
    J = InverseK(X12)
    print(J)
    J = InverseK(X13)
    print(J)
    J = InverseK(X14)
    print(J)
    J = InverseK(X15)
    print(J)
    J = InverseK(X16)
    print(J)
    J = InverseK(X17)
    print(J)
    J = InverseK(X18)
    print(J)
    J = InverseK(X2)
    print(J)
    J = InverseK(X3)
    print(J)
    J = InverseK(X4)
    print(J)
    '''


if __name__ == "__main__":
    main()
