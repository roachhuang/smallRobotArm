# traj
#https://www.coursera.org/learn/robotics1/lecture/bNQfV/4-3-piepers-solution-1
# from cmath import isclose
from math import atan2, pi, isclose
from sympy import Eq, Symbol, init_printing, solve, solveset, sin, cos, simplify, symbols, trigsimp

import ik456 as ik
import numpy as np
import sympy as sp
import craig as cg
import pandas as pd

init_printing(use_unicode=True, use_latex='mathjax')

def pieper(t6_0):
    np.set_printoptions(precision=4, suppress=True)
    q1s = []
    q23s = []
    q123s = []
    q3s = []
    #solved 3 axies in tuple
    qs = []
    list_of_qs=[]

    u, q1, q2, q3 = symbols('u,q1,q2,q3')

    (alp4, a4, d5) = cg.dh_tbl[4, :]
    (alp3, a3, d4) = cg.dh_tbl[3, :]
    (alp2, a2, d3) = cg.dh_tbl[2, :]
    (alp1, a1, d2) = cg.dh_tbl[1, :]

    print('t6_0', t6_0)
    #p4_0_org=P6_0_org
    (x, y, z) = t6_0[0:3, 3]
    print('4-0-org:', x, y, z)

    #(alp3, a3, d4)=cg.dh_tbl[3, :]
    #p4_3=np.array([a3, -d4*sin(alp3), d4*round(cos(alp3)),1])
    #print('p4_3:', p4_3)
    '''
    [x,y,z,1]t=P4_0_org=t1_0@t2_1@t3_2@p4_3_org
    '''
    t4_3 = cg.get_ti2i_1(4)
    p4_3 = t4_3[:, 3]
    print('p4_3:', p4_3)
    t1_0 = cg.get_ti2i_1(1)
    t2_1 = cg.get_ti2i_1(2)
    t3_2 = cg.get_ti2i_1(3)

    #f is a func of theta3, trigsimp her may not need, remove it later
    # f = trigsimp(t3_2 @ p4_3)
    f = t3_2 @ p4_3

    f1, f2, f3 = f[0:3]
    # g is a func of theta2 and theta3, trigsimp in her is essential!!!
    g = trigsimp(t2_1 @ f)
    g1, g2, g3 = g[0:3]

    #print('g1:', g1)
    #print('g2:', g2)
    #print('g3:', g3)
    '''
    p4_0=[x,y,z,1] = t1_0@g = [c1g1-s1g2, s1g1+c1g2, g3, 1]
    x=p4_0orgx=c1g1-s1g2, y=s1g1+c1g2
    frame0 to wrist 的長度平方 so as to eliminate q1
    c1g1**2-s1g2**2+s1g1**2+c1g2**2+g3**2=> g1**2(c1**2+s1**2)+g2**2(s1**2+c1**2)+g3**2
    hence, r=x**2+y**2+z**2=g1**2+g2**2+g3**2
    r is func of only theta2 & 3
    z = g3 (z is func of theta 3)
    '''

    """
    the distance from the origin of the 0 and 1 frames to the center of the spherical
    wrist will also be independent of θ1. The square of this distance, denoted by r is simply the sum of the
    squares of the first three elements of the vector in Equation (3.8);
    """
    r = x**2 + y**2 + z**2
    k1 = f1
    k2 = -f2
    k3 = trigsimp(f1**2 + f2**2 + f3**2 + a1**2 + d2**2 + 2 * d2 * f3)
    k4 = f3 * cos(alp1) + d2 * cos(alp1)

    #
    #r=trigsimp((k1*cos(q2)+k2*sin(q2))*2*a1 + k3)
    #z=trigsimp((k1*sin(q2)-k2*cos(q2))*sin(alp1) + k4)

    # watch out for the case when a+c=0, theta = 180

    if (a1 == 0.0):
        '''
        This will be the case when axes 1 and 2 intersect.
        the distance from the origins of the 0 and 1 frames to the center of the spherical wrist
        (which is the origin of frames 4, 5, and 6) is a function of θ3 only;
        r=k3
        '''
        #q3s=solve(Eq(k3, x**2+y**2+z**2), q3)
        q3s = solve(Eq(r, k3), q3)
        rExpr = (k1 * sin(q2) - k2 * cos(q2)) * sin(alp1) + k4
        lExpr = z
    # general case when a1 != 0 & alpha1 != 0
    # combine k3 and g3
    elif (alp1 == 0.0):
        # z=k4. the height of the spherical wrist center in the 0 frame will be k4.
        q3s = solve(Eq(z, k4), q3)
        rExpr = (k1 * cos(q2) + k2 * sin(q2)) * 2 * a1 + k3
        lExpr = r
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
        c3 = (1 - u**2) / (1 + u**2)
        s3 = (2 * u) / (1 + u**2)
        # very tricky - lexpr needs () for all expression!!!
        lExpr = (r - k3)**2 / (4 * (a1**2)) + ((z - k4)**2) / (sin(alp1))**2
        lExpr = sp.expand(lExpr)
        #print('lExpr:', lExpr)
        lExpr = lExpr.subs([(sin(q3), s3), (cos(q3), c3)])
        #lExpr = lExpr.subs([(sin(q3), simplify('2*u/1+u**2')), (cos(q3), simplify('1-u**2/1+u**2'))])
        #   lExpr = lExpr.expand()
        #print('lExpr:', lExpr)
        rExpr = k1**2 + k2**2
        rExpr = sp.expand(rExpr)
        #print('rExpr:', rExpr)
        rExpr = rExpr.subs([(sin(q3), s3), (cos(q3), c3)])
        # rExpr = rExpr.expand()
        #print('rexpr:', rExpr)
        roots = solve(Eq(lExpr, rExpr), u)
        # print('u:', roots)
        for root in roots:
            t3 = 2 * atan2(root, 1)
            q3s = np.append(q3s, t3)

        # this is for computing q2
        rExpr = (k1 * cos(q2) + k2 * sin(q2)) * 2 * a1 + k3
        lExpr = r

    # remove duplicates from the list
    # solve q2: r=(k1*c2+k2*s2)*2*a1+k3
    # q3s = [*set(q3s)]
    for t3 in q3s:
        r_exp = rExpr.subs(q3, t3)
        tmp = solve(Eq(lExpr, r_exp), q2)
        # one t3 has 2 answers for q2
        q23s.extend(tuple(zip(tmp, [t3, t3])))

    # q2s = [*set(q2s)]

    # to pair q2 and q3 in the array
    q23s = np.array(q23s, dtype=np.float64)
    '''
    x=c1*g1(q2,q3)-s1*g2(q2,q3)
    y=s1*g1(q2,q3)+c1*g2(q2,q3)
    => [[q1, -g2], [g2, g1]] @ [c1,s1].T = [x, y].T
    [c1, s1].T=[[g1, -g2],[g2, g1]].inv @ [x,y].T
    q1 = atan2(s1, c1)
    '''
    print('Q1:', np.rad2deg(atan2(y, x)))
    xy = np.array([x, y]).T
    for t23 in q23s:
        gg1 = g1.subs([(q2, t23[0]), (q3, t23[1])])
        gg2 = g2.subs([(q2, t23[0]), (q3, t23[1])])
        ag = np.array([[gg1, -gg2], [gg2, gg1]], dtype=np.float64)
        cs = np.linalg.inv(ag) @ xy
        t1 = atan2(cs[1], cs[0])
        q1s.append(t1)

    # one pair of q2,q3 returns one q1 only
    q123s = np.insert(q23s, 0, q1s, axis=1)
    # q1 = [-90, 90], constrain q1 btw -90 and 90
    # q123s = np.delete(q123s, np.where((q123s[:,0]<-1.57)|(q123s[:,0]>1.57))[0], axis=0)
    print(f'q1-3: {np.rad2deg(q123s)}')
    #q1s = [*set(q1s)]

    # q1 is simply atan2(y,x) according to ntu
    # q1s.append(atan2(y,x))

    #verify ik for q1-3
    #x=c1g1-s1g2, y=s1g1+c1g2
    # expr_x = sp.expand_trig(cos(q1) * g1 - sin(q1) * g2)
    # expr_y = sp.expand_trig(sin(q1) * g1 + cos(q1) * g2)
    expr_x = cos(q1) * g1 - sin(q1) * g2
    expr_y = sin(q1) * g1 + cos(q1) * g2
    expr_z = g3
    for t123 in q123s:
        myX = expr_x.subs([(q1, t123[0]), (q2, t123[1]), (q3, t123[2])])
        myY = expr_y.subs([(q1, t123[0]), (q2, t123[1]), (q3, t123[2])])
        myZ = expr_z.subs([(q2, t123[1]), (q3, t123[2])])
        if isclose(myX, x) and isclose(myY, y) and isclose(myZ, z):
            print(f'q1: {t123[0] * 180 / pi}, q2: {t123[1] * 180 / pi}, q3: {t123[2] * 180 / pi}')
            #qs.append((t1, t2, t3))
            #q1-3=np.append(qs, [t1,t2,t3])
            # qs array contains verified q1~q3
            #qs = np.append(qs, [t1, t2, t3])
            q123 = np.array([t123[0], t123[1], t123[2]], dtype=np.float64)

            q456 = ik.ik456(t6_0[0:3, 0:3], t123[0], t123[1], t123[2])
            # q456 = ik.ik4_5_6(t6_0[0:3, 0:3], t1, t2, t3)
            qs = np.concatenate((q123, q456))
            list_of_qs=np.append(list_of_qs, qs)
            # get one verified q1-3 is enough
            # print(f'q1-6: {np.rad2deg(qs)}')

    list_of_qs=np.reshape(list_of_qs, (-1,6) )
    # ver q1-q6 for all solutions
    col_names = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6']
    np.asarray(t6_0, dtype='float')
    for ts in list_of_qs:
        fk_t6_0=cg.fk_6axes(ts)
        #print(fk_t6_0)
        assert np.allclose(t6_0.astype('float'), fk_t6_0.astype('float'))
    print('FK6 all done!!!')
    print('------------------------------------ ')
    solutions = pd.DataFrame(list_of_qs, columns=col_names)
    print(np.rad2deg(solutions))
    print('------------------------------------ ')
    # return the 1st solution
    return list_of_qs[0]
    #return qs



