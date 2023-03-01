# from cmath import acos
from math import atan2, pi, sqrt
# the reason for importing cos and sin from sympy is Rot case (can't import them from math, plz note!!!)
from sympy import Symbol, init_printing, sin, cos, symbols, nsimplify, Matrix
import numpy as np
import pandas as pd

# np.set_printoptions(precision=2, suppress=True)

# dh for quiz4
dh_tbl = []


def setDhTbl(stdDh):
    global dh_tbl
    dh_tbl = stdDh
    col_names = ['alphai', 'ai', 'di']
    stdDH = pd.DataFrame(stdDh, columns=col_names)
    print(stdDH)
    print('------------------------------------ ')

# Rotation about axis with theta in radian.


def Rot(axis, rad):
    theta = Symbol('theta')
    theta = rad

    if axis == 'x':
        return np.array([[1, 0, 0], [0, cos(theta), -sin(theta)],
                         [0, sin(theta), cos(theta)]])
    elif axis == 'y':
        return np.array([[cos(theta), 0, sin(theta)], [0, 1, 0],
                         [-sin(theta), 0, cos(theta)]])
    elif axis == 'z':
        return np.array([[cos(theta), -sin(theta), 0],
                         [sin(theta), cos(theta), 0], [0, 0, 1]])

# ti_i-1

def get_ti2i_1(i, theta=None):
    # fill in dh tbl wrt robot arms' dh params

    # array idx starts frm 0, so i-1
    alfa, ai, di, th = symbols('alfa, ai, di, th')
    (alfa, ai, di) = dh_tbl[i - 1, :]
    if theta is None:
        th = 'q' + str(i)
    else:
        th = theta

    # the reason for using sympy's Matrix is that we need to apply it with sympy's simplify func
    # to eliminate sth likes 1.xxxxxe-14 * sin(qx)
    m = Matrix([[cos(th), -sin(th)*cos(alfa), sin(th)*sin(alfa), ai*cos(th)],
                [sin(th), cos(th)*cos(alfa), -cos(th)*sin(alfa), ai*sin(th)],
                [0, sin(alfa), cos(alfa), di],
                [0, 0, 0, 1]])

    if theta is None:
        m = nsimplify(m, tolerance=1e-10, rational=True)
        # print(f't{i}-{i-1}: {m}')
        return np.array(m)
    else:
        m = np.array(m).astype(np.float64)
        return m
