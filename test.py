from cmath import atan, pi
from math import atan2
from sympy import Eq, Symbol, init_printing, solve, solveset, sin, cos, simplify, trigsimp

import numpy as np
import sympy as sp
import craig as cg

init_printing(use_unicode=True, use_latex='mathjax')
np.set_printoptions(precision=3, suppress=True)

def pieper(t6_0):
    q1s=[]
    q2s=[]
    q3s=[]

    u, q1, q2, q3=sp.symbols('u,q1,q2,q3')

    (alp4, a4, d5)=cg.dh_tbl[4, :]
    (alp3, a3, d4)=cg.dh_tbl[3, :]
    (alp2, a2, d3)=cg.dh_tbl[2, :]
    (alp1, a1, d2)=cg.dh_tbl[1, :]

    print('t6_0', t6_0)
    #p4_0_org=P6_0_org
    (x,y,z)=t6_0[0:3, 3]
    print('4-0-org:', x,y,z)

    #(alp3, a3, d4)=cg.dh_tbl[3, :]
    #p4_3=np.array([a3, -d4*sin(alp3), d4*round(cos(alp3)),1])
    #print('p4_3:', p4_3)

    '''
    [x,y,z,1]t=P4_0_org=t1_0@t2_1@t3_2@p4_3_org
    =
    '''
    t4_3=cg.get_ti2i_1(4)
    p4_3=t4_3[:, 3]
    print('p4_3:', p4_3)
    t1_0=cg.get_ti2i_1(1)
    t2_1=cg.get_ti2i_1(2)
    t3_2=cg.get_ti2i_1(3)
    #p4_0_org  = t2_1@t3_2@p4_3

    #f is a func of theta3
    f=trigsimp(t3_2@p4_3)
    f1=f[0]
    f2=f[1]
    f3=f[2]
    # g is a func of theta2 and theta3
    g=trigsimp(t2_1@f)
    g1=trigsimp(g[0])
    g2=trigsimp(g[1])
    g3=trigsimp(g[2])
    print('g1:', g1)
    print('g2:', g2)
    print('g3:', g3)

    '''
    p4_0=[x,y,z,1] = t1_0@g = [c1g1-s1g2, s1g1+c1g2, g3, 1]
    frame0 to wrist 的長度平方 so as to eliminate q1
    c1g1**2-s1g2**2+s1g1**2+c1g2**2+g3**2=> g1**2(c1**2+s1**2)+g2**2(s1**2+c1**2)+g3**2
    hence, r=x**2+y**2+z**2=g1**2+g2**2+g3**2
    r is func of only theta2 & 3
    z = g3 (z is func of theta 3)
    '''

    r=x**2+y**2+z**2
    k1=f1
    k2=-f2
    k3=trigsimp(f1**2 + f2**2 + f3**2 + a1**2 + d2**2 + 2*d2*f3)
    k4=f3*cos(alp1)+d2*cos(alp1)

    #
    #r=trigsimp((k1*cos(q2)+k2*sin(q2))*2*a1 + k3)
    #z=trigsimp((k1*sin(q2)-k2*cos(q2))*sin(alp1) + k4)

    # watch out for the case when a+c=0, theta = 180
    if (a1==0):
        '''
        the distance from the origins of the 0 and 1 frames to the center of the spherical wrist
        (which is the origin of frames 4, 5, and 6) is a function of θ3 only;
        r=k3
        '''
        #q3s=solve(Eq(k3, x**2+y**2+z**2), q3)
        q3s=solve(Eq(r, k3), q3)
    # general case when a1 != 0 & alpha1 != 0
    # combine k3 and g3
    elif (alp1 == 0):
        # z=k4
        q3s=solve(Eq(z, k4), q3)
    else:
        '''
        r=(k1c2+k2s2)*2a1+k3
        z=(k1s2-k2c2)*sin(alp1)+k4
        r**2+z**2 to eliminate theta2, you can see from above 2 equations
        move k3&k4 to the left side, r and z 取平方和 to eliminate q2
        (r-k3/2*a1)**2 + (z-k4/sin(alp1))**2 = k1**2+k2**2
        =>(r-k3)**2/4*a1**2 + (z-k4)**2/sin(alp1)**2 = k1**2+k2**2
        '''
        #u=tan(q3/2)
        c3=(1-u**2)/(1+u**2)
        s3=(2*u)/(1+u**2)
        # very tricky - lexpr needs () for all expression!!!
        lExpr = (r-k3)**2/(4*(a1**2)) + ((z-k4)**2)/(sin(alp1))**2
        lExpr=sp.expand_trig(lExpr)
        #lExpr=trigsimp(((r-k3)**2)/(4*(a1**2)) + ((z-k4)**2)/(sin(alp1))**2)
        print('lExpr:', lExpr)
        lExpr = lExpr.subs([(sin(q3), s3), (cos(q3), c3)])
        #lExpr = lExpr.subs([(sin(q3), simplify('2*u/1+u**2')), (cos(q3), simplify('1-u**2/1+u**2'))])
        lExpr = lExpr.expand()
        print('lExpr:', lExpr)
        #rExpr = (k1**2).expand()+(k2**2).expand()
        rExpr = k1**2+k2**2
        #rExpr = trigsimp(rExpr)
        rExpr=sp.expand_trig(rExpr)
        print('rExpr:', rExpr)
        rExpr = rExpr.subs([(sin(q3), s3), (cos(q3), c3)])
        #rExpr=rExpr.subs([(sin(q3), simplify('2*u/1+u**2')), (cos(q3), simplify('1-u**2/1+u**2'))])
        rExpr = rExpr.expand()
        print('rexpr:', rExpr)
        expr=lExpr-rExpr
        roots=solveset(Eq(lExpr, rExpr), u)
        #roots=expr.all_roots()
        # sp.pprint([expr.evalf(chop=True) for expr in results])
        print('u:', roots)
        for root in roots:
            q3s.append(2*atan2(root, 1))
            #rad = 2*atan(root)
    # remove duplicates from the list
    q3s=list(dict.fromkeys(q3s))
    for t3 in q3s:
        print('t3:=', t3*180/pi)
        # solve q2: r=(k1*c2+k2*s2)*2*a1+k3
        k1 = k1.subs(q3, t3)
        k2 = k2.subs(q3, t3)
        k3 = k3.subs(q3, t3)
        rExpr=(k1*cos(q2)+k2*sin(q2))*2*a1+k3
        tmp=solve(Eq(r, rExpr), q2)
        q2s.extend(tmp)

    q2s=list(dict.fromkeys(q2s))
    # solve q1: x=c1*g1(q2,q3)-s1*g2(q2,q3)
    for t2 in q2s:
        print('t2:=', t2*180/pi)
        for t3 in q3s:
            g1 = g1.subs([(q2,t2),(q3, t3)])
            g2 = g2.subs([(q2,t2),(q3, t3)])
            rExpr=cos(q1)*g1-sin(q1)*g2
            tmp=solve(Eq(x, rExpr),q1)
            q1s.extend(tmp)

    q1s=list(dict.fromkeys(q1s))
    for t1 in q1s:
        print('t1:=', t1*180/pi)

    #pc_0
    cg.verify_ik((227,372), 300, 338, 206, q1s, q2s, q3s)
    print('all done!')

def ntu_pieper():
    #warnings.filterwarnings('ignore')
    #from inverse_kinematic import trig_equ
    dh_tbl=np.array([[0,0,0],
                        [np.deg2rad(-90), -30, 0],
                        [0, 340, 0],
                        [np.deg2rad(-90), -40, 338],
                        [np.deg2rad(90), 0, 0],
                        [np.deg2rad(-90),0,0]])

    cg.setDhTbl(dh_tbl)
    #getcontext().prec=2
    #getcontext().rounding = ROUND_UP

    ty = np.deg2rad(-60) # rotate y axis
    tcup_0_2s = np.array([[cos(ty), 0, sin(ty), 330], [0, 1, 0, 372],
                            [-sin(ty), 0, cos(ty), 367], [0, 0, 0, 1]])
    Tcup_6 = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 206],
                        [0, 0, 0, 1]])
    tcup_0_2s = np.array([[cos(ty), 0, sin(ty), 330], [0, 1, 0, 372],
                            [-sin(ty), 0, cos(ty), 367], [0, 0, 0, 1]])

    t6_0 = tcup_0_2s @ np.linalg.inv(Tcup_6)
    pieper(t6_0)

ntu_pieper()
