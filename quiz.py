# quiz
from cmath import acos, atan, pi, sqrt
from math import radians
import math
from tkinter import S
import craig as cg
#from sympy import *
from sympy import Symbol, init_printing, solve, sin, cos, symbols, trigsimp, simplify
import numpy as np

np.set_printoptions(precision=3, suppress=True)
init_printing(use_unicode=True)
# init_printing( use_latex='mathjax' )  # use pretty math output
dh_tbl=np.array([[0,0,0],
                    [radians(-90), -30, 0],
                    [0, 340, 0],
                    [radians(-90), -40, 338],
                    [radians(90), 0, 0],
                    [radians(-90),0,0]])
cg.setDhTbl(dh_tbl)

def quiz4_3():
    d2= np.sqrt(69.28**2+40**2)
    # alpha(i-1), ai-1, di, thetai
    dh_tbl=np.array([[0,0,0],
                    [radians(-90), -30, 0],
                    [0, 340, 0],
                    [radians(-90), -40, 338],
                    [radians(90), 0, 0],
                    [radians(-90),0,0]])

    cg.setDhTbl(dh_tbl)

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

    t=np.deg2rad(60)
    # rotation in y axis
    tc_d=np.array([[cos(t),  0, sin(t), -500],
                   [0,       1, 0,       452],
                   [-sin(t), 0, cos(t),  410],
                   [0, 0, 0, 1]
                ])
    tc_w=simplify(Td_w@tc_d)
    tc_w=simplify(T0_w@Td_0@tc_d)
    tc_0=simplify(Td_0@tc_d)
    print('tc_0:', tc_0)
    # real parts only
    print('tc_w:', tc_w)
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
    ty= np.deg2rad(60)
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

    #p4_0_org=P6_0_org
    (x,y,z)=t6_0[0:3, 3]
    # (x,y,z)=(7,4,7)
    print('4-0-org:', x,y,z)
    # (i代4到Ti_i-1 的dh formular)
    t4_3=cg.get_ti2i_1(4)
    # returns the 3rd col
    p4_3_org=t4_3[:, 3]
    print('p4_3:', p4_3_org)
    t2_1= cg.get_ti2i_1(2)
    t3_2= cg.get_ti2i_1(3)

    # g3_equ=z
    p4_0_org = simplify(t2_1 @ t3_2 @ p4_3_org)
    print('p4_0:', p4_0_org)
    # !!!manually adjust +30, find a way to auto it. see p13
    g1=p4_0_org[0]
    #num = g1.find(lambda e: e.is_numeric, group=False)
    #num = [float(x) for x in str(g1).split() if x.isnumeric()]
    num=-cg.extract_num(str(g1))

    print('num:', num)
    # add num to two sides
    g1 += num
    print('g1:', g1)
    g1_equ=sqrt(x**2+y**2)+num
    # g1_equ=x+num
    g2_equ=y
    g3_equ=z
    #r_equ=round(g1_equ**2+g2_equ**2+g3_equ**2)
    r_equ=round(g1_equ**2+g3_equ**2)

    #print('p4_0:', p4_0_org)
    #p4_0_org=P6_0_org
    g3=p4_0_org[2]
    print('g3:', g3)

    #q2, q3=symbols('q2, q3')
    g1=trigsimp(g1)
    #print('trigsimp_g1:', g1)
    #print('trigsimp_g3:', trigsimp(g3))
    g3=trigsimp(g3)
    # expand (a+b+c)^2
    r = trigsimp((g1**2).expand()+(g3**2).expand())
    r+=(-r_equ)
    print('r:', simplify(r))
    #q3 = cg.trig_equ(-27200, 229840, 372656)
    #print('q3:', q3)
    q3=Symbol('q3')
    # sol=solve(r, q3)
    # print('sol_q3:', sol[0]*180/pi, 180-sol[1]*180/pi)
    cg.trig_equ(-27200, -229840, 354852)

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
    q3 = math.atan2(k3, s) + math.atan2(a3,d4)
    print('q3::', q3*180/pi)

    g=cos(q1)*x+sin(q1)*y -a1
    e=a3*cos(q3)+d4*sin(q3)+a2
    f=a3*sin(q3)-d4*cos(q3)
    q2=math.atan2(z*e-g*f, g*e+z*f)
    print('q2:', 180-np.rad2deg(q2))

t2_1= cg.get_ti2i_1(2)
t3_2= cg.get_ti2i_1(3)
print('myt3_2:', simplify(t2_1 @ t3_2))

quiz4_3()
tc_0 = quiz4_4()

quiz4_5(tc_0)

def quiz456(q4, q5):
    q5 = acos(sin(q4), cos(q4))
