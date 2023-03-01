from cmath import acos, atan, pi, sqrt
from math import radians
import craig as cg
# import sympy as sp
from sympy import Symbol, init_printing, solve, symbols, trigsimp, simplify
import numpy as np

print(np.version.version)
np.set_printoptions(precision=2, suppress=True)

#init_printing(use_unicode=True)
# init_printing( use_latex='mathjax' )  # use pretty math output
q1, q2, q3, q4, q5, q6 = symbols('q1,q2,q3,q4,q5,q6')

dh_tbl = np.array([[0, 0, 0], [radians(-90), 0, 220], [0, 430, -90],
                   [radians(-90), 0, 430], [radians(90), 0, 0],
                   [radians(-90), 0, 0]])

cg.setDhTbl(dh_tbl)
t1 = radians(15)
t2 = radians(-40)
t3 = radians(-50)
t4 = radians(30)
t5 = radians(70)
t6 = radians(25)


# note: 四捨五入至兩位有效數字
def quiz4():
    # dh table for puma560
    dh_tbl = np.array([[0, 0, 0], [radians(-90), 0, 220], [0, 430, -90],
                       [radians(-90), 0, 430], [radians(90), 0, 0],
                       [radians(-90), 0, 0]])

    cg.setDhTbl(dh_tbl)

    t1_0 = cg.get_ti2i_1(1, t1)
    # substitute q1 with t1
    #t1_0 = t1_0.subs(q1, t1)

    t2_1 = cg.get_ti2i_1(2, t2)
    #t2_1 = t2_1.subs(q2, t2)

    t3_2 = cg.get_ti2i_1(3, t3)
    #t3_2 = t3_2.subs(q3, t3)

    t4_3 = cg.get_ti2i_1(4, t4)
    #t4_3 = t4_3.subs(q4, t4)

    t5_4 = cg.get_ti2i_1(5, t5)
    #t5_4 = t5_4.subs(q5, t5)

    t6_5 = cg.get_ti2i_1(6, t6)
    #t6_5 = t6_5.subs(q6, t6)
    #t6_5 = t6_5.applyfunc(lambda x: round(x,2))

    #t4_0 = t1_0 * t2_1 * t3_2 * t4_3
    t4_0 = t1_0 @ t2_1 @ t3_2 @ t4_3
    print('t4_0:', t4_0)
    #t40 = np.round(t4_0.astype(np.double), decimals=2)
    p4_0 = t4_0[:, 3]
    for x in p4_0:
        print('ans4:', cg.my_sigfig(x,2))
    # print('p4_0:', cg.my_sigfig(p4_0, 2))

    t6_0 = t4_0 @ t5_4 @ t6_5
    print ('t6_0:', np.round(t6_0.astype(np.double),2))
    # below is just for 1-d array
    # t6_0 = np.array([cg.my_sigfig(num, 2) for num in t6_0])

    #sig=np.vectorize(cg.my_sigfig)
    #result=sig(t6_0)
    #print('t6_0:', result)

    #ans to quiz4: 700//320//280
    #ans to quiz2: 0//430//-90//90//0//0
    #ans to quiz5: 700//-0.28//0.54//320


def quiz3():
    t2_1 = cg.get_ti2i_1(2, t2)
    print(type(t2_1))
    #print ('t2_1:', t2_1)
    #print ('t2_1:', np.round(t2_1.astype(np.double),2))
    #ans: 0//220//0.64//-0.77


quiz3()

quiz4()