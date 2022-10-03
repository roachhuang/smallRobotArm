# quiz
from cmath import acos, atan, pi, sqrt
from math import radians
import test as t
import craig as cg
import sympy as sp
from sympy import Symbol, init_printing, solve, sin, cos, symbols, trigsimp, simplify
import numpy as np

np.set_printoptions(precision=3, suppress=True)
# init_printing(use_unicode=True)
init_printing( use_latex='mathjax' )  # use pretty math output

# alpha(i-1), ai-1, di, thetai
dh_tbl=np.array([[0,0,0],
                    [radians(-90), -30, 0],
                    [0, 340, 0],
                    [radians(-90), -40, 338],
                    [radians(90), 0, 0],
                    [radians(-90),0,0]])
cg.setDhTbl(dh_tbl)

def quiz4_3():
    d2= np.sqrt(69.28**2+40**2)
    '''
    dh_tbl=np.array([[0,0,0],
                    [radians(-90), -30, 0],
                    [0, 340, 0],
                    [radians(-90), -40, 338],
                    [radians(90), 0, 0],
                    [radians(-90),0,0]])

    cg.setDhTbl(dh_tbl)
    '''
    #t1_0=cg.get_ti2i_1(5)
    #t1_0=cg.get_ti2i_1(4)
    #t1_0=cg.get_ti2i_1(3)

    t2_1=cg.get_ti2i_1(2)
    t1_0=cg.get_ti2i_1(1)
    #q2=0 coz it is a primstic
    print('t2_0:', simplify(t1_0@t2_1))
    q1= np.arctan2(40, 69.28)
    print('q1:', q1*180/pi)
    print('d2:', round(d2))

# https://www.coursera.org/learn/robotics1/exam/73ksc/quiz-4/attempt?redirectToCover=true
def quiz4_4():
    T0_w = np.array([[1,0,0,0], [0,1,0,0],[0,0,1,373],[0,0,0,1]])
    # Tc_w=Td_w @ tc_d
    Pc_d=[-500,452,410]
    # see fig on p2
    Td_w=np.array([[1,0,0,830],
        [0,1,0,20],
        [0,0,1,330],
        [0,0,0,1]])

    # see fig on p2
    Td_0=np.array([[1,0,0,830],
          [0,1,0,20],
          [0,0,1,-43],
          [0,0,0,1]])

    t=np.deg2rad(-60)
    # rotation in y axis
    tc_d=np.array([[cos(t),  0, sin(t), -500],
                   [0,       1, 0,       452],
                   [-sin(t), 0, cos(t),  410],
                   [0, 0, 0, 1]
                ])
    tc_w=simplify(Td_w@tc_d)
    # my t0_w td_0 & tc_d are correct
    # tc_w=simplify(T0_w@Td_0@tc_d)
    tc_0=simplify(Td_0@tc_d)
    print('tc_0:', tc_0)
    # real parts only
    print('tc_w:', tc_w)
    # 0.5//-0.866//0.866//330/472//740

    # get coord of end effector
    # return tc_w[0:3, 3]
    return tc_0

def quiz4_5(tc_0):
    np.set_printoptions(precision=3, suppress=True)
    # see p7
    Tcup_6 = np.array([[0,  0, 1, 0],
                       [0, -1, 0, 0],
                       [1,  0, 0, 206],
                       [0,  0, 0, 1]])
    T0_w = np.array([[1,0,0,0], [0,1,0,0],[0,0,1,373],[0,0,0,1]])
    Td_0=np.array([[1,0,0,830],
          [0,1,0,20],
          [0,0,1,-43],
          [0,0,0,1]])

    # the cup is rotated in y axis
    ty= np.deg2rad(-60)
    x=-500
    y=452
    z=410
    tcup_d = np.array([[cos(ty), 0, sin(ty), x], [0, 1, 0, y],
                          [-sin(ty), 0, cos(ty), z], [0, 0, 0, 1]])

    # sequence matters!!! think about it.
    tcup_0= simplify(Td_0 @ tcup_d)
    print('tcup_0', tcup_0)
    '''
    dh_tbl=np.array([[0,0,0],
                    [np.deg2rad(-90), 0, 0],
                    [0, 2, 0.5],
                    [np.deg2rad(-90), 0.1666, 2],
                    [np.deg2rad(90), 0, 0],
                    [np.deg2rad(90),0,0]])

    '''
    # t6_0=np.linalg.inv(t0_w) @ Tc_w @ np.linalg.inv(Tcup_6)
    # tc_0 = tc_6 @ t6_0 ->
    t6_0=simplify(tc_0 @ np.linalg.inv(Tcup_6))

    '''
    t6_0 =np.array([[-1/sqrt(2),0, 1/sqrt(2), 1],
                    [0,-1,0,1],
                    [1/sqrt(2),0, 1/sqrt(2), -1],
                    [0,0,0,1]])
    '''
    print('t6_0', t6_0)
    t.pieper(t6_0)
    # theta3: -158//35
    # quiz4_6(t6_0, -2.7603560010540047)
    # quiz4_6(t6_0, -0.6168272962769602)

    #theta2: 12//-50
    #theta1: 64

# compute q2
def quiz4_6(t6_0, q3):
    (x,y,z)=t6_0[0:3, 3]
    t2_1=cg.get_ti2i_1(2)
    t4_3 = cg.get_ti2i_1(4)
    p4_3_org = t4_3[:, 3]
    # send caculated q3 to ti_2_i-1
    new_t3_2= cg.get_ti2i_1(3, q3)
    p4_0_org= t2_1@new_t3_2@p4_3_org
    # z coordinate for p6_0_org=p4_0_org
    g3=p4_0_org[2]
    #print('g3:', g3)
    q2=Symbol('q2')
    #g3=z, note that z is independent of theta1
    th2=solve(sp.Eq(g3, z), q2)
    print('@q3:=', q3*180/pi)
    print('theta2:=', th2[0]*180/pi, th2[1]*180/pi)
    return th2

quiz4_3()
tc_0 = quiz4_4()

quiz4_5(tc_0)


'''
    # https://zhuanlan.zhihu.com/p/440748878
    (alp4, a4, d5)=cg.dh_tbl[4, :]
    (alp3, a3, d4)=cg.dh_tbl[3, :]
    (alp2, a2, d3)=cg.dh_tbl[2, :]
    (alp1, a1, d2)=cg.dh_tbl[1, :]
    q1=math.atan2(d3, abs(sqrt(x+y+z-d3*d3)))+math.atan2(y,x)
    print('q1', np.rad2deg(q1))

    q1 = math.atan2(y, x)
    print('q1', q1*180/pi)

    #https://zhuanlan.zhihu.com/p/440748878
    k3= ((a1-cos(q1)*x-sin(q1)*y)**2+z**2-(a3**2+d4**2+a2**2))/(2*a2)
    s = sqrt(a3**2+d4**2-k3**2)
    print('s:', s)
    q3 = atan(k3/s) + atan(a3/d4)
    print('q3::', q3*180/pi)

    g=cos(q1)*x+sin(q1)*y -a1
    e=a3*cos(q3)+d4*sin(q3)+a2
    f=a3*sin(q3)-d4*cos(q3)
    q2=atan(z*e-g*f/g*e+z*f)
    print('q2:', q2*180/pi)

t2_1= cg.get_ti2i_1(2)
t3_2= cg.get_ti2i_1(3)
print('myt3_2:', simplify(t2_1 @ t3_2))
'''
def quiz456(q4):
    q5 = acos(sin(q4), cos(q4))
