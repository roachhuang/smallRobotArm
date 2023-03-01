"""
i have a robot arm. its craig dh table params are alpha0=0, alpha1=-90, aphha2=0, alpha3=-90, alpha4=90, alpha5=-90, a0=0,a1=-30, a2=340, a3=-40, a4=0, a5=0, d1=0,d2=0,d3=0,d4=338,d5=0,d6=0
如何利用euler zyz 的公式去求一個4,5,6軸原點相交的機器手臂的4,5,6軸的角度.
"""
# aix 2&3 decide heigh of z and sqt(x**2+y**2)
# q4,q5, q6
# refer to must read UCLA ik.pdf for r6_3
# https://univ.deltamoocx.net/courses/course-v1:AT+AT_010_1102+2022_02_01/courseware/a3e573de127b85f1dcb23ea797cd253f/dc947a72e470ca516e9270c3bb4424e1/?child=first

# this ik456 doesn't work for puma650!!!
from math import atan2, cos, sin, acos, asin, sqrt, pi
import numpy as np
import craig as cg
from sympy import symbols, simplify

def ver456(r6_3, q4s, q5s, q6s):
    for t4 in q4s:
        for t5 in q5s:
            for t6 in q6s:
                t4_3 = cg.get_ti2i_1(4, t4)
                t5_4 = cg.get_ti2i_1(5, t5)
                t6_5 = cg.get_ti2i_1(6, t6)
                t6_3 = t4_3 @ t5_4 @ t6_5
                R6_3 = t6_3[0:3, 0:3]
                #print('r6_3:', r6_3)
                #print('R6_3:', R6_3)
                if np.allclose(r6_3.astype(np.float64), R6_3.astype(np.float64)):
                    print(f'q4:{t4 * 180 / np.pi}, q5:{t5 * 180 / np.pi}, q6:{t6 * 180 / np.pi}')
                    return (np.array([t4, t5, t6], dtype=np.float64))
    # return 'no 456'

def ik4_5_6(r6_0, t1, t2, t3):
    # np.set_printoptions(precision=4, suppress=True)
    q1,q2,q3 = symbols('q1, q2, q3')
    t4_3=cg.get_ti2i_1(4)
    t5_4=cg.get_ti2i_1(5)
    t6_5=cg.get_ti2i_1(6)
    R6_3=t4_3[0:3, 0:3]@t5_4[0:3, 0:3]@t6_5[0:3, 0:3]
    print(f'R6_3={R6_3}')

    t3_2=cg.get_ti2i_1(3, t3)
    t2_1=cg.get_ti2i_1(2, t2)
    t1_0=cg.get_ti2i_1(1, t1)
    r3_0 = t1_0[0:3, 0:3]@t2_1[0:3, 0:3]@t3_2[0:3, 0:3]
    r6_3 = np.transpose(r3_0) @ r6_0
    print(f'r6_3= {r6_3}')
    # r6_3[1,2]=cos(q5)
    q5 = atan2(r6_3[1,2], sqrt(1-(r6_3[1,2])**2))
    print(np.rad2deg(q5))
    q5 = atan2(r6_3[1,2], -sqrt(1-(r6_3[1,2])**2))
    print(np.rad2deg(q5))
    q4 = atan2(-r6_3[0,2], r6_3[2,2])
    q6 = atan2(r6_3[1,0], -r6_3[1,1])

    ############## chatgpt
    phi = atan2(R6_3[2, 1], R6_3[2, 2])
    theta = atan2(-R6_3[2, 0], sqrt(R6_3[2, 1]**2 + R6_3[2, 2]**2))
    psi = atan2(R6_3[1, 0], R6_3[0, 0])
    print(f'q4={np.rad2deg(q4)}, q6={np.rad2deg(q6)}')
    ##################################
    
    phi = atan2(R6_3[1,2], R6_3[2,2])
    theta = atan2(-R6_3[0,2], sqrt(R6_3[1,2]**2 + R6_3[2,2]**2))
    psi = atan2(R6_3[0,1], R6_3[0,0])

    theta = atan2(-R[2,0], sqrt(R[0,0]**2 + R[1,0]**2))
    phi = atan2(R[1,0]/cos(theta), R[0,0]/cos(theta))
    psi = atan2(R[2,1]/cos(theta), R[2,2]/cos(theta))

    phi = atan2(R6_3[1,2], R6_3[2,2])
    theta = asin(-R6_3[0,2])
    psi = atan2(R6_3[0,1], R6_3[0,0])

# input: radian
def ik456(r6_0, t1, t2, t3):
    #q456s = []
    q4s=[]
    q5s=[]
    q6s=[]
    """
    q4,q5,q6=euler angles phi, theta, psi. see unit5 part3
    R6-3=Rz,phi Ry,theat Rz,psi = Rz,q4 Ry,q5 Rz,q6
    (q1,q2,q3)->R3-0->R6-3=(R3-0)t * R6-0->q4,q5,q6
    use standard DH table to compute R3-0
    1) R6-3=R4-3(q4)R5-4(q5)R6-5(q6) see unit5 part3
    2) also R6-3(q4,q5,q6) = (R3-0)t * R6-0
    for (1) = (2)

    """
    #after compute q1-3, we can know R3_0
    #T3_0=T1_0@T2_1@T3_2
    """
    r3_0=np.array([[c1c23, -c1s23, s1],
     [s1c23,    -s1s23,  -c1 ],
     [s23,  c23,  0   ])
    """
    t3_2=cg.get_ti2i_1(3, t3)
    t2_1=cg.get_ti2i_1(2, t2)
    t1_0=cg.get_ti2i_1(1, t1)
    t3_0 = t1_0@t2_1@t3_2
    r3_0 = t3_0[0:3, 0:3]
    alp3 = cg.dh_tbl[3, 0]
    # to align with zyz, we need to rotx(alp) wrt R4_3
    r3prime_0 = r3_0 @ cg.Rot('x', alp3)
    # r3_0=X(alp0)Z(t1)X(alp1)Z(t2)X(alp2)Z(t3)
    # = X(0)Z(58.61)X(-90)Z(-64.46)X(0)Z(-11.98)

    r6_3 = np.transpose(r3_0) @ r6_0

    # Elur angle zyz, NTU; however, puma560's axes 4,5,6 aint the same as ntu's.
    # firstable rotate r4_3 in x axis to be in line w/ euler,
    # b frame to a frame Rz'y'z'(alp,beta, gamma) = Rz'(alp)Ry'(beta)Rz'(gamma)
    # rot('z', q4)@r6_4 =(r3_0@rot('x', alp3)).T @ r6_0
    # rot('z', q4)@r6_4 = rot('z', q4)@r5_4(q5)@r6_5(q6)

    r6_3prime = r3prime_0.T @ r6_0
    # r6_3prime = r6_3
    #print('r6_3prime:', r6_3prime)
    # r6_3prime = r4_3primez'y'z'(alp, beta, gama)
    r13 = r6_3prime[0, 2]
    r23 = r6_3prime[1, 2]
    r31 = r6_3prime[2, 0]
    r32 = r6_3prime[2, 1]
    r33 = r6_3prime[2, 2]
    beta = atan2(sqrt(r31**2 + r32**2), r33)
    alp=atan2(r23 / sin(beta), r13 / sin(beta))
    gamma=atan2(r32 / sin(beta), -r31 / sin(beta))
    q4s=np.append(q4s, alp)
    q5s=np.append(q5s, -beta)
    q6s=np.append(q6s, gamma)

    beta = atan2(-sqrt(r31**2 + r32**2), r33)
    alp=atan2(r23 / sin(beta), r13 / sin(beta))
    gamma=atan2(r32 / sin(beta), -r31 / sin(beta))
    q4s=np.append(q4s, alp)
    q5s=np.append(q5s, -beta)
    q6s=np.append(q6s, gamma)
    #q456s=np.row_stack((ans1, ans2))

    # https://www.meccanismocomplesso.org/en/3d-rotations-and-euler-angles-in-python/

    R = r6_3
    phi = atan2(R[2,2], -R[0,2])
    theta = atan2(sqrt(R[0,2]**2+R[2,2]**2), R[1,2])
    psi = atan2(-R[1,1], R[1,0])
    a4=phi
    a5=theta-pi/2
    a6=psi-theta-pi/2
    print(f'a4={np.rad2deg(phi)}, a5={np.rad2deg(theta)}, a6={np.rad2deg(psi)}')


    '''
    this arr is for reference. don't remove it
    r6_3=np.array([[-c4c5c6-s4s6, -c4c5s6-s4c6, -c4s5],
                [s5c6, -s5s6, c5],
                [-s4c5c6-c4s6, s4c5s6-c4c6, s4s5]])
    '''

    #-----------------------------------------------------------
    '''
    q4s = []
    q5s = []
    q6s = []
    c4s5 = -r6_3[0, 2]
    s4s5 = r6_3[2, 2]
    c5 = -r6_3[1, 2]
    s5c6 = r6_3[1, 0]
    s5s6 = -r6_3[1, 1]
    print('c5', c5)

    # s5!=0, i.e q5 !=0 or pi, s5=+-sqrt(1-c5**2)
    q5_1 = atan2(sqrt(1 - c5**2), c5)
    q5s = np.append(q5s, q5_1)
    #print('q5_1:', q5_1 * 180 / pi)

    q4_1 = atan2(s4s5, c4s5)
    q4s = np.append(q4s, q4_1)
    #print('q4_1:', np.rad2deg(q4_1))

    q6_1 = atan2(s5s6, s5c6)
    q6s = np.append(q6s, q6_1)
    #print('q6_1', np.rad2deg(q6_1))

    # s5 < 0, 0 ~ -pi
    q5_2 = atan2(-sqrt(1 - c5**2), c5)
    q5s = np.append(q5s, q5_2)
    #print('q5_2', q5_2 * 180 / pi)

    # coz s5 is neg
    q4_2 = atan2(-s4s5, -c4s5)
    q4s = np.append(q4s, q4_2)
    #print('q4_2', np.rad2deg(q4_2))

    q6_2 = atan2(-s5s6, -s5c6)
    q6s = np.append(q6s, q6_2)
    #print('q6_2', np.rad2deg(q6_2))
    '''
    #--------------------------------------------------------

    print(f'q4= {np.rad2deg(q4s)}, q5= {np.rad2deg(q5s)}, q6= {np.rad2deg(q6s)}')
    return ver456(r6_3, q4s,q5s,q6s)
