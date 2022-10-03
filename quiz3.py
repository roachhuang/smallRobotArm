from cmath import acos, atan, pi, sqrt
from math import radians
import craig as cg
import sympy as sp
from sympy import Symbol, init_printing, solve, sin, cos, symbols, trigsimp, simplify
import numpy as np

print (np.version.version)

np.set_printoptions(precision=3, suppress=True)
# init_printing(use_unicode=True)
init_printing( use_latex='mathjax' )  # use pretty math output
q1, q2, q3=sp.symbols('q1,q2,q3')

dh_tbl=np.array([[0,0,0],
                    [radians(-90), 0, 220],
                    [0, 430, -90],
                    [radians(-90), 0, 430],
                    [radians(90), 0, 0],
                    [radians(-90),0,0]])

cg.setDhTbl(dh_tbl)
q1=15
q2=-40
q3=-50
q4=30
q5=70
q6=25

def quiz3():
    t1_0 = cg.get_ti2i_1(1, radians(q1))
    t2_1 = cg.get_ti2i_1(2, radians(q2))
    t3_2 = cg.get_ti2i_1(3, radians(q3))
    t4_3 = cg.get_ti2i_1(4, radians(q4))

    t5_4 = cg.get_ti2i_1(5, radians(q5))
    t6_5 = cg.get_ti2i_1(6, radians(q6))

    t4_0 = t1_0@t2_1@t3_2@t4_3
    #t40 = np.round(t4_0.astype(np.double), decimals=2)
    p4_0 = t4_0[:, 3]
    print('p4_0:', p4_0)

    t6_0=t4_0@t5_4@t6_5
    print ('t6_0:', t6_0)
    #print ('t6_0:', np.round(t6_0.astype(np.double),2))
    #ans: 0//220//0.64//-0.77

def quiz2():
    t2_1 = cg.get_ti2i_1(2, radians(q2))
    print (type(t2_1))
    print ('t2_1:', t2_1)
    #print ('t2_1:', np.round(t2_1.astype(np.double),2))
    #ans:
quiz2()

quiz3()