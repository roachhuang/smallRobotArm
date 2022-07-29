
from cmath import acos, pi
from math import atan2
from sympy import Symbol, solve, sin, cos, simplify, symbols
import numpy as np

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
q1 = qNalp1 - alp
q2 = qNalp2 - alp
print(q1 * 180 / pi, q2 * 180 / pi)

c3=cos(q1)
s3=sin(q1)
x=Symbol('x')
expr=(40*(sin(x)*c3+cos(x)*s3)-338*(cos(x)*c3-sin(x)*s3)-340*sin(x)-188.6)
sol=solve(expr, x)
#print(expr)
#print(sol[0]*180/pi, 180-sol[1]*180/pi)

q2, q3=symbols('q2, q3')
expr1=-40*cos(q2+q3).expand(trig=True)-338*sin(q2+q3).expand(trig=True)+340*cos(q2)-465.79
print('expr1:', expr1)
#expr2=40*sin(q2+q3).expand(trig=True)-338*cos(q2+q3).expand(trig=True)-340*sin(q2)-188.6
#q = solve(expr1**2+expr2**2, q3)
#print(q)

def get_ti2i_1(i):
    np.set_printoptions(precision=2, suppress=True)
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

    np.set_printoptions(precision=2, suppress=True)
    print(t)
    return(t)

def dh():
    np.set_printoptions(suppress=True)

    # (i代4到Ti_i-1 的dh formular)
    t4_3=get_ti2i_1(4)
    # returns the 3rd col
    p4_3_org=t4_3[:, 3]

    t2_1= get_ti2i_1(2)
    t3_2= get_ti2i_1(3)
    print('p4_0_org:', t2_1@t3_2@p4_3_org)

dh()