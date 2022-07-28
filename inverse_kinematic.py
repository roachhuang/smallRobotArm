from math import atan2
from cmath import cos, sin, acos, atan, pi, sqrt
from pickletools import long4
import numpy as np

'''
 https://www.onlinemathlearning.com/trig-equations-type.html
    I) acos(q)-bsin(q)=c => acos(q)-bsin(q)=r*cos(q+alp)
        r=sqrt(a**2+b**2)
        alp=atan2(b,a)
        r*cos(q+alp)=c
        qNalp=acos(c/r)
        tow solutions for q:
            1) q=qNalp-alp
            2) q=360-qNalp-alp

    II) asin(q)-bcos(q)=r*sin(q-alp)
'''
def trig_equ(a, b, c):
    r=np.sqrt(a**2+b**2)
    alp=atan2(b,a)
    #r*cos(q+alp)=c
    # or 360-
    qNalp1=acos(c/r)
    qNalp2=2*pi - qNalp1
    q3_1=qNalp1-alp
    q3_2=qNalp2-alp
    print(q3_1*180/pi, q3_2*180/pi)
    return (q3_1, q3_2)

#https://www.coursera.org/learn/robotics1/lecture/bNQfV/4-3-piepers-solution-1
def ik_pieper():
    np.set_printoptions(precision=2, suppress=True)
    """
    P4_3_org is 4th col of T4_3 (i代4到Ti_i-1 的dh formular)
    [x,y,z,1]=P4_0_org = T1_0@T2_1@T3_2@P4_3_org
    =T1_0@T2_1@T3_2@np.array([a3,
                    -s(alpha3)*d4],
                    [c(alpha3)*d4],
                    1]])

                    =T1_0T2_1@np.array([[f1(q3)],
                                [f2(q3)],
                                [f3(q3)],
                                [1]])
    P4_0_org=T1_0@T2_1@T3_2@P4_3_org=T1_0@T2_1@f1(q3)
    np.array([f1(q3)],
    [f2(q3)],
    [f3[q3]])=T3_2@np.array([[a3],
                            [-sin(alpha3)*d4],
                            [cos(alpha3)*d4],
                            1]])

    f1(q3)=a3*c(q3)+d4*s(alpha3)*s(q3)+a2
    f2(q3)=a3*c(alpha2)s3-d4*s(alpha3)*c(alpha2)*c3-d4*s(alpha2)*c(alpah3)-d3*s(alpha2)
    f3(q3)=a3*s(alpha2)s3-d4*s(alpha3)*s(alpha2)*c3+d4*c(alpha2)*c(alpah3)+d3*c(alpha2)
    g(q2)=T2_1@f(q3)
    """
    a2=340
    #q3=0
    d3=0
    alpha2=np.deg2rad(-90)
    # i=3 to the dh Ti_i-1 formular
    """
    T3_2=np.array([[cos(q3), -sin(q3), 0, a2],
                    [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                    [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2), cos(alpha2), cos(alpha2)*d3],
                    [0,0,0,1]
                    ])

    further more
    let g to be the func of q2 and q3
    p4_0_org = p6_0_org = goal coz frame4,5&6 all located at the point of intersection.
    P4_0_org=T1_0@T2_1@[f1]=T1_0@[g(q2,q3)]=np.array([[cos(q1), -sin(q1), 0,0],
                                                      [sin(q1),  cos(q1), 0,0],
                                                      [0,0,1,0],
                                                      [0,0,0,1]]) @ np.array([[g1(q2,q3)],
                                                                                [g2(q2,q3)],
                                                                                [g3(q2,q3)],
                                                                                [1]])
                                                      = np.array([[c1g1(q2,q3)-s1g2(q2,q3)],
                                                        [s1g1(q2,q3)+c1g2(q2,q3)],
                                                        [g3(q2,q3)],
                                                        [1])
    # see above, g3 is the z coord of P4_0_org and independent of q1.

    t2_1=np.array([[c2,-s2,0,a1],
                    [0,0,1,0],
                    [-s2,-c2,0,0],
                    [0,0,0,1]])
    t3_2= t3_2=np.array([[c3,-s3,0,a2],
                    [s3,c3,0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
    g=T2_1@f= T2_1@T3_2@P4_3_org, hence:

    t6_0=Tcup_0@np.lialg_inv(Tcup_6)

    from above we fine P6_0_org=P4_0_org=[227,372,188.6]
    P4_3_org=np.array([-40,338,0,1])
    g=T2_1@T3_2@P4_3_org=np.array([[340c2-40c23-338s23-30],
                                  [0],
                                  [40s23-338c23-340s2]])
                                  =np.array([435.79,0,188.6,1])
    see p12, sqrt(x**2+y**2)=sqrt(227**2+372**2)=435.79, z=188.6
    find, g1(q2,q3)=340c2-40c23-338s23-30=435.79=>
    g1=-40c23-338s23+340c2=465.79   Eq1
    g3=40s23-338c23-340s2=188.6     Eq2
    (a+b+c)**2=a**2+b**2+c**2+2ab+2bc+2ca
    Eq1**2+Eq2**2=Eq3
    40**2+338**2+340**2(40)(340)(-c3)+2(338)(340)(-s3)=252530
    q3=-11.98 or 178.48
    q2=-64.46 or 20.37
    q1=atan2(y,x)=58.61

    g1(q2,q3)=c2f1(q3)-s2f2(q3)+a1
    g2(q2,q3)=s2c(alpah1)f1(q3)+c2c(alpah1)f2(q3)-s(alpha1)f3(q3)-d2s(alpha1)
    g3(q2,q3)=s2s(alpha1)f1(q3)+c2s(alpha1)f2(q3)+c(alpah1)f3(q3)+d2c(alpah1)
    # r=distance from org of frames 0&1 to orgins of frames 4,5&6
    r=x**2+y**2+z**2=g1**2+g2**2+g3**2 (coz T1_0 = Identity matrix)
    =f1**2+f2**2+f3**2+a1**2+d2**2+2d2*f3+2*a1(c2f1-s2f2)
    =(k1c2+k2s2)a1+k3
    k1(q3)=f1
    k2(q3)=-f2
    k3(q3)=f1**2+f2**2+f3**2+a1**2+d2**2+2d2f3
    z = height (independt of q1)
    z=g3=(k1s2-k2c2)s(alpha1)+k4    z also is the func of q2,q3
    k4(q3)=f3c(alpha1)+d2c(alpha1)

    integrate r and z from above (q3 in k)
    r=(k1c2+k2s2)2a1+k3
    z=(k1s2-k2c2)s(alpah1)+k4
    r 為已知， 從 x, y, z 得出。
    if a1=0 (axes 1&2 are intersect), r**2=k3(q3)=f1**2+f2**2+f3**2+a1**2+d2**2+2*d2*f3

    eslif s(alpha1)=0, z=k4(q3)=f3c(alpha1)+d2*c(alpha1)

    else
        (r**2-k3)/2a1=(k1c2+k2s2)
        (z-k4/s(alpha1) = (k1s2-k2c2))
        squaring both sides, we find (r**2-k3/2a1)**2=(k1c2_k2s2)**2 = k1**2*c2**2+k2**2*s2**2+2k1k2c2s2
        (z-k4/s(alpha1))**2=(k1s2-k2c2)**2=k1**2s2**2+k2**2c2**2-2k1k2c2s2
        adding these two equ together, we find a 4th order of equ for q3
        (r**2-k3/2a1)**2 +(z-k4/s(alpha1))**2=k1**2+k2**2

    slove q3 of all three cases by using u=tan(q/2)
    cos(q) = 1-u**2/1+u, sin(q)=2u/1+u**2
    transcendental equation: aci+bsi=c -> a(1-u**2)+2bu=c(1+u**2)
    (a+c)u**2-2bu+(c-a)= 0
    u=b+-sqrt(b**2-a**2-c**2)/a+c
    q=2*atan(u)
    if u is complex there is no real sol to the org transcendental equation
    if a+c=0, then q=180

    after solving q3
    using r=(k1c2+k2s2)2a1+k3 to solve q2 (use u=tan(q2/2))
    using x=c1g1(q2,q3)-s1g2(q2,q3) to slove q1
    """
    #see fig on p3. it is fixed
    Td_w=np.array([[1,0,0,830],
                   [0,1,0,20],
                   [0,0,1,330],
                   [0,0,0,1]])
    tz=np.deg2rad(35)
    # not fixed. according to the cup's current positon. can be compute by camera input data
    # this is use at init state. ie. 0s
    tc_d=np.array([[cos(tz), -sin(tz), 0, -280],
                   [sin(tz), cos(tz),  0, 250],
                   [0,       0,        1, 62.5],
                   [0,       0,        0, 1]])

    # see p2 & p7, tc_w=T0_w@t6_0@Tc_6
    tc_w=Td_w@tc_d
    #print('tc_w', tc_w)
    T0_w=np.array([[1,0,0, 0],
                   [0,1,0, 0],
                   [0,0,1, 373],
                   [0,0,0, 1]])
    # see p7
    Tcup_6=np.array([[0, 0, 1, 0],
                     [0,-1, 0, 0],
                     [1, 0, 0, 206],
                     [0, 0, 0, 1]])

    tz=np.deg2rad(35)
    tcup_0_0s=np.array([[cos(tz), -sin(tz), 0, 550],
                    [sin(tz), cos(tz), 0, 270],
                    [0, 0, 1, 19.5],
                    [0,0,0,1]])
    ty=np.deg2rad(-60)
    tcup_0_2s=np.array([[cos(ty),  0, sin(ty),  330],
                        [0,        1, 0,        372],
                        [-sin(ty), 0, cos(ty),  367],
                        [0,        0, 0,        1]])

    #t6-0 contains q1~q6
    #print('tc0@0s', tc_0_0s)

    # this is only for 0s
    #t6_0=np.linalg.inv(T0_w)@tc_w@np.linalg.inv(Tcup_6)

    #t6_0=tcup_0_0s@Tcup_6
    t6_0= tcup_0_2s@np.linalg.inv(Tcup_6)
    print('t6_0', t6_0)

     #i 帶入4 dh craig formular
    a3=-40
    d4=338
    alpha3=np.deg2rad(-90)
    p4_3_org = np.array([[a3],
                        [-d4*sin(alpha3)],
                        [d4*cos(alpha3)],
                        [1]])
    print('p4_3', p4_3_org)
    '''
    [x,y,z,1]=p4_0_org = T1_0@T2_1@T3_2@P4_3_org
    =T1_0@T2_1@T3_2@np.array([a3
                    -d4s(alpha3)],
                    [d4c(alpha3)],
                    1]])

                    =T1_0T2_1@np.array([[f1(q3)],
                                [f2(q3)],
                                [f3(q3)],
                                [1]])
    np.array([f1(q3)],
    [f2(q3)],
    [f3[q3]])=T3_2@np.array([[a3],
                            -d4*sin(alpha3)],
                            d4*cos(alpha3)],
                            1]])

    t3_2=np.array([[c3,-s3,0,340],
                        [s3,c3,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
    f1~3(q3)=t3_2@P4_3_org=np.array([[340-338s3-40c3],
                                [338c3-40s3],
                                [0],
                                [1]])
    t2_1=np.array([[c2, -s2, 0, -30],
            [0,0, 1, 0],
            [-s2,-c2, 0,0],
            [0,0,0,1]])
    g1~3(q2,q3)=t2_1@f1~3(q3)= np.array([[340c2-40c23-338s23-30],
                                        [0],
                                        [40s23,-338c23-340s2],
                                        [1],)
    '''

    #f1= a2*cos(q3)+d4*sin(alp3)*sin(q3)+a2
    #f2=a3*sin(q3)*cos(alp2)-d4*sin(alp3)*cos(alp2)*cos(q3)-d4*sin(alp2)*cos(alp3)-d3*sin(alp2)
    #f3=a3*sin(q3)*sin(alp2)-d4*sin(alp3)*sin(alp2)*cos(q3)+d4*cos(alp2)*cos(alp3)+d3*cos(alp2)
    #a=f1**2+f2**2+f3**2
    #r_sq=381.3**2+151.8**2+19.5**2
    #r2=f1**2+f2**2+f3**2+a1**2+d2**2+2*d2*f3
    #f1=340c3-338s3+340
    #f2=-40s3+338c3

    alp0=alp2=0
    alp1=alp3=-90
    a1=-30
    a2=340
    a3=-40
    d2=d3=0
    d4=338
    x=t6_0[0,3]
    y=t6_0[1,3]
    z=t6_0[2,3]
    q1 = atan2(y, x)
    print('q1', np.rad2deg(q1))
    # eq1**2+eq2**2=eq3**2 (use 積化合差法 gets asin(x)+bcos(x)=c)
    #c=(252530-40**2-338**2-340**2)/(2*340)
    
    c=((sqrt(x**2+y**2)-a1)**2+z**2-a3**2-d4**2-a2**2)/(2*a2)
    a=a3
    b=d4
    # 2 solutions
    q3=trig_equ(a,b,c)
    # g3=40s23-338c23-340s2=188.6     Eq2   to solve q2
    # 40(s2c3+c2s3)-338(c2c3+s2s3)-340s2=188.6
    c3=cos(q3[0])
    s3=sin(q3[0])
    print('c3,s3', c3, s3)
    #188.6=40(0.98s2+c2(-0.21))-338(c2(0.98)+s2(-0.21))
    #39.2s2-8.4c2-331c2+71s2
    #-230s2-339.4c2=118.6=>-339.4c2-230s2=118.6
    sol_q2= trig_equ(-339.4, 230, 118.6)

    '''
    i end up not using this formular
    u=tan(q/2), c(q)=1-u**2/1+u**2, s(q)=2u/1+u**2
    #acos+bsin=c
    a*(1-u**2/1+u**2)+b*(2u/1+u**2)=c
    multiply 1+u**2 at both ends
    a*(1-u**2)+b*2u=c*(1+u**2)
    a-a*u**2+b*2u=c+c*u**2
    c-a+(c+a)u**2-b*2u=0
    tan(q3/2)=(2*b)+/-sqrt(2b**2-4ac)/2(c+a)
    '''
    #q3= 2*atan((2*b)+sqrt(2*b**2-4*a*c)/2*(c+a))

    #q3=2*atan((b+sqrt(b**2-a**2-c**2))/(a+c))
    #q3=2*atan(sqrt(a**2+b**2+c**2)/c)+atan2(b,a)
    #print('q3', q3*180/pi)
    return (t6_0[0:3, 0:3])

r6_0= ik_pieper()

def ik_ntu(l1,l2,l3, x,y,z):

    """
    https://www.coursera.org/learn/robotics1/lecture/Q7ftE/4-2-duo-zhong-jie-3-example-2
    for dh table see https://www.coursera.org/learn/robotics1/lecture/tvNZN/3-6-dh-biao-da-fa-xiao-jie-2-example
    dh table
        i   alphai-1    ai-1    di  qi
        ------------------------------
        1   0           0       0   q1
        2   -90         -30     0   q2
        3   0           340     0   q3
        4   -90         -40     338 q4
        5   90          0       0   q5
        6   -90         0       0   q6
    #T3_0=T1_0@T2_1@T3_2
    =[[c1c2c3   s1c23 -s23  -a2c3,
     [-c1s23    -s1s23  -c23    -a2s3],
     [-s1   c1  0   -d3],
     [0,0,0,1]]
    ->
    x=l1*cos(q1)+l2*cos(q2)=(l1+l2*cos(q2))cos(q1)+(-l2*sin(q2))*sin(q1)~=k1c1=k2s1
    y=l1*sin(q1)+l2sin(q1+q2)=(l1+l2*cos(q2))*sin(q1)+(l2*sin(q2))cos(q1)~=k1s1+k2c1
    k1=l1+l2*cos(q2); k2=l2*sin(q2)
    define r=+-sqrt(k1**2+k2**2), then k1=r*cos(gama)
           gama=atan2(k2,k1)           k2=r*sin(gama)
    and then
        x/r = cos(gama)*cos(q1)-sin(gama)*sin(q1)=cos(gama+q1)
        y/r = cos(gama)*sin(q1)+sin(gama)*cos(q1)=sin(gama+q1)
        gama+q1=atan2(y/r,x/r)=atan2(y,x)
        q1=atan2(y,x)-atan2(k2,k1)
        k1&k2 depend on q2 so as q1
        q3= phi-q1-q2
    """

# https://univ.deltamoocx.net/courses/course-v1:AT+AT_010_1102+2022_02_01/courseware/a3e573de127b85f1dcb23ea797cd253f/ef53e59ae8e7fd326f807b8b49505fe5/1
# x and y are center coord of the arm
def ik_part3_a(l1, l2, l3,l4, x, y, z):
    d1=0
    s=z-d1
    r1=sqrt(x**2+y**2)
    r2=sqrt(x**2+y**2+(z-(l1+l2)**2))
    #r2_sq=l3+l4-2*l3*l4*cos(180+q3)=l3**2+l4**2+2*l3*l4*cos(q3)
    c3=r2**2-(l3**2+l4**2)/2*l3*l4
    s3=sqrt(1-c3**2)
    print('c3,s3:', c3, s3)
    #oc0=np.array([227,472,188.6])
    # coord of arm center 對frame0
    q1=atan2(y,x)
    #sol 2 (@phase3) -> q1=pi+atan2(x,y)

    # cos(q3)=D; cos(pi-q3)=-cos(q3)
    D = (r1**2 + s**2 - l2**2 - l3**2) / (2 * l2 * l3)

    # q3=atan2(D, +-sqrt(1-D**2))
    # q3 = acos(D) not recommended, coz could be in phase1,3 or 2,4
    # cos**2+sin**2=1 -> sin=+-sqrt(1-cos**2)
    #sol1
    aty = sqrt(1-D**2)
    #print(D)
    q3=atan(-aty/D)

    q2=0
    # q2=alpha-beta
    # r and s are oc 投影到x1, y1 coord. where s=zc-d1(基座高度 or height of joint1)

    # q2 = atan2(s,r)-atan(l3*sin(q3)/l2+l3*cos(q3))
    return (np.rad2deg(q1),q2,q3)

#print(ik_part3_a(0,-30,340,-40, 227,472,188.6))

# q4,q5, q6
# refer to must read UCLA ik.pdf for r6_3 )
# https://univ.deltamoocx.net/courses/course-v1:AT+AT_010_1102+2022_02_01/courseware/a3e573de127b85f1dcb23ea797cd253f/dc947a72e470ca516e9270c3bb4424e1/?child=first
def ik_part4(r6_0, q1,q2,q3):
    np.set_printoptions(precision=3, suppress=True)
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
    t1=np.deg2rad(q1)
    t2=np.deg2rad(q2)
    t3=np.deg2rad(q3)
    # this r3_0 uses standard dh table.
    r3_0=np.array([[cos(t1)*cos(t2+t3), -cos(t1)*sin(t2+t3), -sin(t1)],
    [sin(t1)*cos(t2+t3), -sin(t1)*sin(t2+t3), cos(t1)],
    [-sin(t2+t3), -cos(t2+t3), 0 ]])

    R3_0=np.array([[0.6006, 0.7082, -0.3710],
    [0.24, 0.2830, 0.9286],
    [0.7627, -0.6468, 0]])
    print('r3_0', r3_0)
    r6_3=np.transpose(r3_0)@r6_0
    print('r6-3', r6_3)
    '''
    r6_3=np.array([[c4c5c6-s4s6, -c4c5s6-s4c6, -c4s5],
                [s5c6, -s5s6, -c5],
                [-s4c5c6-c4s6, s4c5s6-c4c6, s4s5]])
    '''
    c4s5=-r6_3[0,2]
    s4s5=r6_3[2,2]
    c5=-r6_3[1,2]
    s5c6=r6_3[1,0]
    s5s6=-r6_3[1,1]
    print('c5', c5)
    # s5!=0, i.e q5 !=0 or pi, s5=+-sqrt(1-c5**2)
    q5_1=atan(sqrt(1-c5**2)/c5)
    print('q5_1:', q5_1*180/pi)
    q4_1=atan2(s4s5,c4s5)
    print('q4_1:', np.rad2deg(q4_1))
    q6_1=atan2(s5s6,s5c6)
    print('q6_1', np.rad2deg(q6_1))

    # s5 < 0, 0 ~ -pi
    q5_2 =atan(-sqrt(1-c5**2)/c5)
    print('q5_2', q5_2*180/pi)
    # coz s5 is neg
    q4_2=atan2(-s4s5,-c4s5)
    print('q4_2', np.rad2deg(q4_2))
    q6_2=atan2(-s5s6,-s5c6)
    print('q6_2', np.rad2deg(q6_2))

    #DH6_3 = np.array([[c(q4-q6), sin(q4-q6),0, 0],
    # sin(q4-q6), -cos(q4-q6),0, 0],
    # [0,0,-1, d4],
    # [0,0,0,1])
    # R6_3=np.transpose(R3_0)@R6_0


ik_part4(r6_0, 58.61, -64.46, -11.98)

# https://www.youtube.com/watch?v=yHsIn6gA1_8&t=30s
def ik_delta(l1, l2, l3, x, y, phi):
    # cos(q2)
    D = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    # q2 = acos(D) not recommended as q2 can be 2 solutions - phase1,3 or 2,4
    #sin(q2)=+-sqrt(1-D**2)
    q2=atan(sqrt(1-D**2)/D)
    # or q2=atan2(-sqrt(1-D**2)/D)

    # q1 dependent on q2. 有偏差值d, oc 不相交于 z0 axis.
    q1=atan(y/x)-atan(l2*sin(q2)/l1+l2*cos(q2))
    print('q1,q2:', q1,q2)

# return q1, q2, q3
def ik(l1, l2, l3, x, y, phi):
    sol1 = []
    sol2 = []

    r_sq = x**2 + y**2
    # refer to 4-2 example 1
    psi = acos((l2**2 - r_sq - l1**2) / (-2 * l1 * sqrt(r_sq)))
    # solution 1 (q2 > 0 deg)
    sol1.append(atan2(y, x) - psi)
    sol1.append(acos((r_sq - l1**2 - l2**2) / (2 * l1 * l2)))
    sol1.append(phi - sol1[0] - sol1[1])
    #q1_3 = phi - degrees(q1_1) - degrees(q1_2)

    # solution 2 (q2 < 0 deg)
    sol2.append(atan2(y, x) + psi)
    sol2.append(-sol1[1])
    sol2.append(phi - sol2[0] - sol2[1])

    #q3=phi-q1-q2
    # joint space
    #print('sol1:', degrees(q1_1), degrees(q1_2), q1_3)
    #print('sol2:', degrees(q2_1), degrees(q2_2), q2_3)
    print('sol1 (q1~q3):', sol1)
    #print('sol2 (q1~q3):', sol2)
    return sol1
