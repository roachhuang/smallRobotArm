# Ters_kinematik.pdf 3.3.3 / p11

from math import sqrt, radians
import numpy as np
import craig as cg
import pieper as pp

# warnings.filterwarnings('ignore')
# from inverse_kinematic import trig_equ

# np.set_printoptions(precision=3, suppress=True)

# PUMA 560's Craig DH table
dh_tbl = np.array([[0, 0, 0], [radians(-90), 0, 0], [0, 2, 0.5],
                   [radians(-90), 0.1666, 2], [radians(90), 0, 0],
                   [radians(-90), 0, 0]], dtype=np.float64)

cg.setDhTbl(dh_tbl)
# getcontext().prec=2
# getcontext().rounding = ROUND_UP

t6_0 = np.array([[-(1 / sqrt(2)), 0, 1 / sqrt(2), 1], [0, -1, 0, 1],
                 [1 / sqrt(2), 0, 1 / sqrt(2), -1], [0, 0, 0, 1]], dtype=np.float64)

pp.pieper(t6_0)


"""
q1,q2,q3:   24.3, -28.7, 45.9
            24.3, 102.0, 143.7
            -114.3, 77.14, 45.9
            -114.3, -151.32, 143.7
"""

"""
def getT4_0_org():
    np.set_printoptions(precision=3, suppress=True)

    # (i代4到Ti_i-1 的dh formular)
    t4_3 = cg.get_ti2i_1(4)
    # returns the 3rd col
    p4_3_org = t4_3[:, 3]
    print('p4_3:', p4_3_org)
    t2_1 = cg.get_ti2i_1(2)
    t3_2 = cg.get_ti2i_1(3)

    p4_0_org = t2_1 @ t3_2 @ p4_3_org
    # !!!manually adjust +30, find a way to auto it.
    g1 = p4_0_org[0]
    #print('p4_0:', p4_0_org)
    g2 = p4_0_org[1]
    #print('g2:', g2)
    g3 = p4_0_org[2]
    #print('g3:', g3)
    #q2, q3=symbols('q2, q3')
    #g1=trigsimp(g1)
    print('trigsimp_g1:', g1)
    print('trigsimp_g2:', trigsimp(g2))
    print('trigsimp_g3:', trigsimp(g3))
    #g3=trigsimp(g3)
    r = trigsimp((g1**2).expand() + (g2**2).expand() + (g3**2).expand())
    print('r:', r)
    #print(((np.sqrt(257**2+372**2))**2+188.6**2))

def computeQ2(q3):
    # send caculated q3 to ti_2_i-1
    new_t3_2= cg.get_ti2i_1(3, q3)
    p4_0_org= t2_1@new_t3_2@p4_3_org
    # z coordinate for p6_0_org=p4_0_org
    g3=p4_0_org[2]
    #print('g3:', g3)
    q2=Symbol('q2')
    #g3=z, note that z is independent of theta1
    th2=solve(Eq(g3, z), q2)
    print('@q3:=', q3*180/pi)
    print('theta2:=', (th2[0]*180)/pi, th2[1]*180/pi)
    return th2

def computeQ1(th2, th3):
    xy=np.array([x,y])
    #new_t2_1 = cg.get_ti2i_1(2, q2)
    #new_t3_2 = cg.get_ti2i_1(3, q3)
    #print('4_3:', p4_3_org)
    #new_g = trisimp(new_t2_1 @ new_t3_2 @ p4_3_org)
    #print('new 4_0:', new_g)

    # ng1=-2.0*sin(q2 + q3) + 2.0*cos(q2) + 0.1666*cos(q2 + q3)
    # substitute q2 & q3 with th2 and th3 for g1 and g2 equations
    q2, q3 = symbols('q2 q3')
    ng1=g1.subs([(q2, th2), (q3, th3)])
    ng2=g2.subs([(q2, th2), (q3, th3)])

    '''
    [[g1, -g2],[g2, g1]][cos(q1), sin(q1)]=[x,y]
    -> [cos(q1), sin(q1)]=a.inv() @ xy-> q1=atan2(sin, cos)
    '''
    a = Matrix([[ng1, -ng2],[ng2, ng1]])
    a_inv=a.inv() @ xy
    th1 = atan2(a_inv[1], a_inv[0])

    print('@q2, q3:', th2*180/pi, th3*180/pi)
    print('theta1:=', np.rad2deg(th1))
    return th1

# getT4_0_org()

def puma560():
    global g1, g2, p4_3_org, t1_0, t2_1, x, y, z

    (alp4, a4, d5) = cg.dh_tbl[4, :]
    (alp3, a3, d4) = cg.dh_tbl[3, :]
    (alp2, a2, d3) = cg.dh_tbl[2, :]
    (alp1, a1, d2) = cg.dh_tbl[1, :]

    #(alp3, a3, d4)=cg.dh_tbl[3, :]
    #p4_3=np.array([a3, -d4*sin(alp3), d4*round(cos(alp3)),1])
    #print('p4_3:', p4_3)
    '''
    [x,y,z,1]=P4_0_org=t1_0@t2_1@t3_2@p4_3_org
    =
    '''
    t4_3 = cg.get_ti2i_1(4)
    p4_3_org = t4_3[:, 3]
    #print('p4_3:', p4_3_org)
    t1_0 = cg.get_ti2i_1(1)
    t2_1 = cg.get_ti2i_1(2)
    t3_2 = cg.get_ti2i_1(3)
    p4_0_org = trigsimp(t1_0 @ t2_1 @ t3_2 @ p4_3_org)
    #print('p4_0_org', p4_0_org)
    # p6_0 = p4_0
    (x, y, z) = t6_0[0:3, 3].real
    print('4-0-org:', x, y, z)

    #f is a func of theta3
    f = trigsimp(t3_2 @ p4_3_org)
    f1 = f[0]
    f2 = f[1]
    f3 = f[2]
    #f1=d4*sin(alp3)+a2
    #print('f1:', f1)
    #print('f2:', f2)
    #print('f3:', f3)
    # g is a func of theta2 and theta3
    g = trigsimp(t2_1 @ f)
    #print('g:', g)
    g1 = g[0]
    g2 = g[1]
    g3 = g[2]

    print('g1:', g1)
    print('g2:', g2)
    #print('g3:', g3)
    #p4_0=[x,y,z,1]=t1_0@g
    # r=x**2+y**2+z**2=g1**2+g2**2+g3**2
    # r is func of only theta2 & 3
    # z = g3 (z is func of theta 3)
    # r=simplify(g1**2+g2**2+g3**2)
    # r is the desired distance from the org of frame0 to the org of frame6
    r = trigsimp((g1**2).expand() + (g2**2).expand() + (g3**2).expand())
    # r = trigsimp((f1**2).expand() + (f2**2).expand() + (f3**2).expand())
    print('r:', r)
    q3=Symbol('q3')
    # r = x**2+y**2+z**2
    th3= solve(Eq(r, x**2+y**2+z**2), q3)
    print('theta3:=', th3[0]*180/pi, th3[1]*180/pi)

    th2_on_th3_0=computeQ2(th3[0])
    th2_on_th3_1=computeQ2(th3[1])

    computeQ1(th2_on_th3_0[0],th3[0])
    computeQ1(th2_on_th3_1[1],th3[1])

    computeQ1(th2_on_th3_0[1],th3[0])
    computeQ1(th2_on_th3_1[0],th3[1])
"""
