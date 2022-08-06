from cmath import acos, pi
from math import atan2
from sympy import Symbol, init_printing, solve, sin, cos, symbols
import numpy as np

dh_tbl=np.array([[0,0,0],
                    [np.deg2rad(-90), -30, 0],
                    [0, 340, 0],
                    [np.deg2rad(-90), -40, 338],
                    [np.deg2rad(90), 0, 0],
                    [np.deg2rad(-90),0,0]])

# ti_i-1
def get_ti2i_1(i):
    np.set_printoptions(precision=2, suppress=True)
    # fill in dh tbl wrt robot arms' dh params


    '''
    dh_tbl=np.array([[0,0,0],
                    [np.deg2rad(-90), 0, 10]])
    '''

    # array idx starts frm 0, so i-1
    (alp, ai, di)=dh_tbl[i-1, :]
    t='q'+str(i)
    ci = Symbol('cos'+str(i))
    si = Symbol('sin'+str(i))
    t=np.array([[cos(t), -sin(t), 0, round(ai)],
                [sin(t)*round(cos(alp),2), cos(t)*round(cos(alp),2), round(-sin(alp)), round(-sin(alp)*di)],
                [sin(t)*round(sin(alp),2), cos(t)*round(sin(alp),2), round(cos(alp)), round(cos(alp)*di)],
                [0,0,0,1]
                ])
    print(t)
    return(t)
