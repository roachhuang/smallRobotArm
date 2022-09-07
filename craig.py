from asyncio.windows_events import NULL
from cmath import acos, pi
from math import atan2
from sympy import Symbol, init_printing, solve, sin, cos, symbols
import numpy as np
'''
dh_tbl=np.array([[0,0,0],
                    [np.deg2rad(-90), -30, 0],
                    [0, 340, 0],
                    [np.deg2rad(-90), -40, 338],
                    [np.deg2rad(90), 0, 0],
                    [np.deg2rad(-90),0,0]])
'''
#dh for quiz4
dh_tbl = []


def setDhTbl(dh):
    global dh_tbl
    dh_tbl = dh


# ti_i-1
def get_ti2i_1(i, theta=NULL):
    np.set_printoptions(precision=3, suppress=True)
    init_printing(use_latex='mathjax')  # use pretty math output
    # fill in dh tbl wrt robot arms' dh params

    # array idx starts frm 0, so i-1
    alp, ai, di, t = symbols('alp, ai, di, t')
    (alp, ai, di) = dh_tbl[i - 1, :]
    if (theta == NULL):
        t = 'q' + str(i)
    else:
        t = theta
    #ci = Symbol('cos'+str(i))
    #si = Symbol('sin'+str(i))
    t = np.array([[cos(t), -sin(t), 0, round(ai)],
                  [
                      sin(t) * round(cos(alp), 2),
                      cos(t) * round(cos(alp), 2),
                      round(-sin(alp)),
                      round(-sin(alp) * di)
                  ],
                  [
                      sin(t) * round(sin(alp), 2),
                      cos(t) * round(sin(alp), 2),
                      round(cos(alp)),
                      round(cos(alp) * di)
                  ], [0, 0, 0, 1]])
    print(f't{i}-{i-1}:', t.real)
    return (t.real)


# a- cos' param, b for sin  asin(x)+bcos(x)=c


# a- cos' param, b for sin
def trig_equ(a, b, c):
    np.set_printoptions(precision=3, suppress=True)
    r = np.sqrt(a**2 + b**2)
    alp = atan2(b, a)
    #r*cos(q+alp)=c
    # or 360-
    qNalp1 = acos(c / r)
    qNalp2 = 2 * pi - qNalp1
    q_1 = (qNalp1 - alp).real
    q_2 = (qNalp2 - alp).real
    print('q3:', q_1 * 180 / pi, q_2 * 180 / pi)
    return (q_1, q_2)


def is_negative_number_digit(n: str) -> bool:
    try:
        int(n)
        return True
    except ValueError:
        return False


def extract_num(inp_str):
    # inp_str=inp_str.strip()
    # inp_str= inp_str.replace("+ ", "+")
    # inp_str= inp_str.replace("- ", "-")

    print("Original String : ", inp_str)
    num = 0
    s = '+'
    for c in inp_str.split():
        if c == '+' or c == '-':
            s = c  # keep the sign of nxt numeric
        elif c.isnumeric():
            num = num - float(c) if s == '-' else num + float(c)
            print("Extracted numbers from the list : ", num)
    return num