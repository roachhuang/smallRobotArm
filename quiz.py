# quiz
from cmath import cos, pi, sin
import math
import craig as cg
#from sympy import *
from sympy import Symbol, init_printing, solve, sin, cos, symbols, trigsimp
import numpy as np

np.set_printoptions(precision=3, suppress=True)
init_printing( use_latex='mathjax' )  # use pretty math output

def quiz4_3():
    d2= np.sqrt(69.28**2+40**2)
    dhTbl = np.array([[0,0,0],
         [np.deg2rad(-90), 0, d2]])

    cg.setDhTbl(dhTbl)

    t2_1=cg.get_ti2i_1(2)
    t1_0=cg.get_ti2i_1(1)
    #q2=0 coz it is a primstic
    print(t1_0@t2_1)
    q1= np.arctan2(40, 69.28)
    print('q1:', q1*180/pi)
    print('d2:', round(d2))

# https://www.coursera.org/learn/robotics1/exam/73ksc/quiz-4/attempt?redirectToCover=true
def quiz4_4():
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
    tc_d=np.array([[cos(t),  0, sin(t), -500],
                 [0,       1, 0,        452],
                 [-sin(t), 0, cos(t),   410],
                 [0, 0, 0, 1]
                ])
    tc_w=Td_w@tc_d
    tc_0=Td_0@tc_d
    print('tc_0:', tc_0.real)
    # real parts only
    print('tc_w:', tc_w.real)
    # get coord of end effector
    # return tc_w[0:3, 3]
    return tc_0

def quiz4_5(tcup_0):
    np.set_printoptions(precision=3, suppress=True)
    # see p7
    Tcup_6 = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 206],
                       [0, 0, 0, 1]])
    '''
    ty= np.deg2rad(60)
    tcup_0 = np.array([[cos(ty), 0, sin(ty), x], [0, 1, 0, y],
                          [-sin(ty), 0, cos(ty), z], [0, 0, 0, 1]])
    '''
    dh_tbl=np.array([[0,0,0],
                    [np.deg2rad(-90), -30, 0],
                    [0, 340, 0],
                    [np.deg2rad(-90), -40, 338],
                    [np.deg2rad(90), 0, 0],
                    [np.deg2rad(-90),0,0]])
    cg.setDhTbl(dh_tbl)
    #t6_0=tcup_0_0s@Tcup_6
    t6_0 = tcup_0 @ np.linalg.inv(Tcup_6)
    print('t6_0', t6_0)
    #p4_0_org=P6_0_org
    (x,y,z)=t6_0[0:3, 3]

    # (i代4到Ti_i-1 的dh formular)
    t4_3=cg.get_ti2i_1(4)
    # returns the 3rd col
    p4_3_org=t4_3[:, 3]
    print('p4_3:', p4_3_org)
    t2_1= cg.get_ti2i_1(2)
    t3_2= cg.get_ti2i_1(3)

    g3_equ=z
    p4_0_org= t2_1@t3_2@p4_3_org
    print('p4_0:', p4_0_org)
    # !!!manually adjust +30, find a way to auto it. see p13
    g1=p4_0_org[0]
    #num = g1.find(lambda e: e.is_numeric, group=False)
    #num = [float(x) for x in str(g1).split() if x.isnumeric()]
    num=-cg.extract_num(str(g1))
    # what about its sign?
    print('num:', num)
    # add num to two sides
    g1 += num
    g1_equ=math.sqrt(x**2+y**2)+num
    r_equ=g1_equ**2+g3_equ**2 - 231444
    #print('p4_0:', p4_0_org)
    g3=p4_0_org[2]
    #print('g3:', g3)
    #q2, q3=symbols('q2, q3')
    g1=trigsimp(g1)
    #print('trigsimp_g1:', g1)
    #print('trigsimp_g3:', trigsimp(g3))
    g3=trigsimp(g3)
    # expand (a+b+c)^2
    r = trigsimp((g1**2).expand()+(g3**2).expand())
    print('r:', r)
    q3 = cg.trig_equ(-27200, -229840, r_equ)
    print('q3:', q3)
    #q3=Symbol('q3')
    #sol=solve(r-round(r_equ), q3)
    #print('q3:', sol[0]*180/pi, 180-sol[1]*180/pi)

    q1 = math.atan2(y, x)
    print('q1', q1*180/pi)

quiz4_3()
tc_0 = quiz4_4()

quiz4_5(tc_0)
