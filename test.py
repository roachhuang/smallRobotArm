from decimal import *
from cmath import acos, pi
from math import atan2
from sympy import Symbol, init_printing, solve, sin, cos, symbols, trigsimp
from sympy.simplify.fu import fu, L, TR0, TR10, TR3, TR8, TR9, TR10i, TR11
import numpy as np
import craig as cg
#import warnings

#warnings.filterwarnings('ignore')
#from inverse_kinematic import trig_equ
dh_tbl=np.array([[0,0,0],
                    [np.deg2rad(-90), -30, 0],
                    [0, 340, 0],
                    [np.deg2rad(-90), -40, 338],
                    [np.deg2rad(90), 0, 0],
                    [np.deg2rad(-90),0,0]])

cg.setDhTbl(dh_tbl)
getcontext().prec=2
getcontext().rounding = ROUND_UP

ty = np.deg2rad(-60) # rotate y axis
tcup_0_2s = np.array([[cos(ty), 0, sin(ty), 330], [0, 1, 0, 372],
                          [-sin(ty), 0, cos(ty), 367], [0, 0, 0, 1]])
Tcup_6 = np.array([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 206],
                       [0, 0, 0, 1]])

init_printing(use_unicode=True)
np.set_printoptions(precision=2, suppress=True)
a1 = -30
print((227)**2 + (372)**2 + 188.6**2 + 2 * (227 + 372) * 30 + 30**2)
print((435.79 - a1)**2 + 188.6**2)
c = (252530 - 40**2 - 338**2 - 340**2) / (2 * 340)
print('c', c)
a = -40
b = 338
#a*cos-b*sin=c
r = np.sqrt(a**2 + b**2)
alp = atan2(b, a)
#r*cos(q+alp)=c
# or 360-
qNalp1 = acos(c / r)
qNalp2 = 2 * pi - qNalp1
q31 = (qNalp1 - alp).real
q32 = (qNalp2 - alp).real
print('q3_1, q3_2:', (q31 * 180 / pi), q32 * 180 / pi)

c3=cos(q31)
s3=sin(q31)
x=Symbol('x')
expr=(40*(sin(x)*c3+cos(x)*s3)-338*(cos(x)*c3-sin(x)*s3)-340*sin(x)-188.6)
sol=solve(expr, x)
print(expr)
print('q2:', sol[0]*180/pi, 180-sol[1]*180/pi)

q2, q3=symbols('q2, q3')
expr1=trigsimp(-40*cos(q2+q3)-338*sin(q2+q3)+340*cos(q2)-465.79)
#print('expr1:', expr1)
expr2=trigsimp(40*sin(q2+q3)-338*cos(q2+q3)-340*sin(q2)-188.6)
#print('my', trigsimp(expr1**2+expr2**2))
#s= TR8(expr1**2+expr2**2)
#print(s)
#q = solve((expr1**2+expr2**2), q3)
#print('q3:', q)

# ti_i-1
'''
def get_ti2i_1(i):
    np.set_printoptions(precision=2, suppress=True)
    # fill in dh tbl wrt robot arms' dh params
    dh_tbl=np.array([[0,0,0],
                    [np.deg2rad(-90), -30, 0],
                    [0, 340, 0],
                    [np.deg2rad(-90), -40, 338],
                    [np.deg2rad(90), 0, 0],
                    [np.deg2rad(-90),0,0]])
    # array idx starts frm 0, so i-1
    (alp, ai, di)=dh_tbl[i-1, :]
    ci = Symbol('c'+str(i))
    si = Symbol('s'+str(i))
    t=np.array([[ci, -si, 0, round(ai)],
                    [si*round(cos(alp),2), ci*round(cos(alp),2), round(-sin(alp)), round(-sin(alp)*di)],
                    [si*round(sin(alp),2), ci*round(sin(alp),2), round(cos(alp)), round(cos(alp)*di)],
                    [0,0,0,1]
                    ])
    print(t)
    return(t)
'''
def getT4_0_org():
    np.set_printoptions(suppress=True)

    # (i代4到Ti_i-1 的dh formular)
    t4_3=cg.get_ti2i_1(4)
    # returns the 3rd col
    p4_3_org=t4_3[:, 3]
    print('p4_3:', p4_3_org)
    t2_1= cg.get_ti2i_1(2)
    t3_2= cg.get_ti2i_1(3)

    p4_0_org= t2_1@t3_2@p4_3_org
    # !!!manually adjust +30, find a way to auto it.
    g1=p4_0_org[0]+30
    #print('p4_0:', p4_0_org)
    g2=p4_0_org[1]
    #print('g2:', g2)
    g3=p4_0_org[2]
    #print('g3:', g3)
    #q2, q3=symbols('q2, q3')
    g1=trigsimp(g1)
    print('trigsimp_g1:', g1)
    print('trigsimp_g2:', trigsimp(g2))
    print('trigsimp_g3:', trigsimp(g3))
    g3=trigsimp(g3)
    r = trigsimp((g1**2).expand()+(g3**2).expand())
    print('r:', r)
    #print(((np.sqrt(257**2+372**2))**2+188.6**2))

getT4_0_org()

