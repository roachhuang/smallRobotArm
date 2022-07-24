from cmath import sqrt
from math import atan2, cos
from math import sin
import numpy as np
from roboticstoolbox import null

print('r:', sqrt(227**2+372**2))

def dh1():
    r1_0=np.array([['c1','-s1','0'],
                    ['s1','c1','0'],
                    ['0','0','1']
                    ])
    r2_1=np.array([['c2','-s2','0'],
                    ['0','0','1'],
                    ['-s2','-c2','0']
                    ])
    r3_2=np.array([['c3','-s3','0'],
                    ['s3','c3','0'],
                    ['0','0','1']
                    ])
                    
    t2_1=np.array([['c2','-s2','0','a1'],
                    ['0','0','1','0'],
                    ['-s2','-c2','0','0'],
                    ['0','0','0','1']])

    t3_2=np.array([['c3','-s3','0','a2'],
                    ['s3','c3','0','0'],
                    ['0','0','1','0'],
                    ['0','0','0','1']])

    t4_3=np.array([['c4','-s4',0,'a3'],
                    [0,0,1,'d4'],
                    ['-s4','-c4',0,0],
                    [0,0,0,1]])

    alphai_1=np.array([0, 0, np.deg2rad(-90), 0, np.deg2rad(-90), np.deg2rad(90), np.deg2rad(-90)])
    ai_1=np.array([0, 0,'a1','a2','a3',0,0])
    d=np.array([0, 0,0,0,'d4',0,0])
    s=np.array(['', 's1','s2','s3','s4','s5','s6'])
    neg_s=np.array(['', '-s1','-s2','-s3','-s4','-s5','-s6'])
    c=np.array(['', 'c1','c2','c3','c4','c5','c6'])
    #Tn_0=T1_0@T2_1@T3_2@...Tn_n-1
    #i=4

    print('T6_1:', t2_1*t3_2)

dh1()

def dh():
    alphai_1=np.array([0, 0, np.deg2rad(-90), 0, np.deg2rad(-90), np.deg2rad(90), np.deg2rad(-90)])
    ai_1=np.array([0, 0,-30,340,-40,0,0])
    d=np.array([0, 0,0,0,338,0,0])
    s=np.array(['', 's1','s2','s3','s4','s5','s6'])
    neg_s=np.array(['', '-s1','-s2','-s3','-s4','-s5','-s6'])
    c=np.array(['', 'c1','c2','c3','c4','c5','c6'])
    # Tn_0=T1_0@T2_1@T3_2@...Tn_n-1
    #i=4

    #print('T1_0', Ti_i_1)
    T=[]
    for i in range(1, 7):
        Ti_i_1=np.array([[c[i], neg_s[i], 0, ai_1[i]],
                    [s[i]+str(cos(alphai_1[i])), c[i]+str(cos(alphai_1[i])), -sin(alphai_1[i]), -sin(alphai_1[i])*d[i]],
                    [s[i]+str(sin(alphai_1[i])), c[i]+str(sin(alphai_1[i])), cos(alphai_1[i]), cos(alphai_1[i])*d[i]],
                    [0,0,0,1]
                    ])

        #i=x
        T.append(Ti_i_1)
        print(T[i-1])

    print('T6_0:', T[0]@T[1]@T[2]@T[3]@T[4]@T[5])

#dh()

def fk_3axes(l1, l2, l3, q1, q2, q3):
    x = l1 * cos(q1) + l2 * cos(q1 + q2) + l3 * cos(q1 + q2 + q3)
    y = l1 * sin(q1) + l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3)


def fk(l1, l2, q1, q2):
    x = l1 * cos(q1) + l2 * cos(q1 + q2)
    y = l1 * sin(q1) + l2 * sin(q1 + q2)
    print(x, y)
    return (x, y)

"""
i alpha(i-1)   a(i-1)  d(i)  theta(i)
--------------------------------
1   0           0       0   t1
2   0           l1      0   t2
3   0           l2      0   t3
"""

def dh(l1, l2, t1, t2):
    end_point = np.array([l2, 0, 0, 1])
    print(end_point.shape)

    T1_0 = np.array([[cos(t1), -sin(t1), 0, 0], [sin(t1),
                                                 cos(t1), 0, 0], [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    T2_1 = np.array([[cos(t2), -sin(t2), 0, l1], [sin(t2),
                                                  cos(t2), 0, 0], [0, 0, 1, 0],
                     [0, 0, 0, 1]])

    T2_0 = T1_0 @ T2_1
    #print(T2_0)
    # multiplication order matter
    print('dh:', T2_0 @ end_point)
    # if l3 takes into consideration - T3_0 @ [l3,0,0,1]


fk(5, 3, 2.5, 2.21)
fk(5, 3, 2.02, 1.05)

#fk(5, 3, 0.31, 2.33)
fk(5, 3, 0.35, 2.35)
fk(5, 3, -1.3, 2.2)
dh(5, 3, -1.3, 2.2)
#fk(5, 3, -1.62, 2.35)
#fk(5, 3, -5.62, 0.74)


def cal_q(dt, a0, a1, a2, a3):
    return (a0 + (a1 * dt) + (a2 * dt**2) + (a3 * dt**3))


print('0~2s of t1')
print(cal_q(0, 2.5, 0, -0.08, -0.02))
print(cal_q(2, 2.5, 0, -0.08, -0.02))
print('0~2 of t2')
print(cal_q(0, 2.21, 0, -0.83, 0.27))
print(cal_q(2, 2.21, 0, -0.83, 0.27))

print('2~4s of t1')
print(cal_q(0, 1.99, -0.6, -0.22, 0.05))
print(cal_q(2, 1.99, -0.6, -0.22, 0.05))
print('2~4s of t2')
print(cal_q(0, 1.01, -0.14, 0.76, -0.18))
print(cal_q(2, 1.01, -0.14, 0.76, -0.18))

print('4~9s of t1')
print(cal_q(0, 0.35, -0.83, 0.1, -0))
print(cal_q(5, 0.35, -0.83, 0.1, -0))
print('4~9 of t2')
print(cal_q(0, 2.35, 0.77, -0.31, 0.03))
print(cal_q(5, 2.35, 0.77, -0.31, 0.03))

#-----------Z-Y-Z-----
# beta is in rad
q2 = beta = atan2(-1, 0)
sinB = sin(beta)
print('beta:', np.rad2deg(beta))
q1 = alpha = atan2(0.5736 / sinB, 0.8192 / sinB)
print('alpha:', np.rad2deg(alpha))
q3 = gama = atan2(0 / sinB, -1 / sinB)
print('gama:', np.rad2deg(gama))
# -145, -90, 0
r11 = 0.8192
r12 = -0.5736
r21 = 0.5736
r22 = 0.8192
r23 = 0
# s5< 0 q6=atan2(s1r11-c1r21, -s1r12+c1r22)
#s5 !=0 q6=atan2(-s1r11+c1r23,s1r12-c1r22)
q6 = atan2(-sin(q1) * r11 + cos(q1) * r23, sin(q1) * r12 - cos(q1) * r22)
print('q6:', np.rad2deg(q6))
print('Tc-0@t0:', cos(np.deg2rad(35)))