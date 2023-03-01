# quiz5 3~6 @ coursera's robotics by ntu

from math import radians
import numpy as np
import inverse_kinematic as ik
# from inverse_kinematic import ik
#import timer

# refer to the table where the question given.

dt1 = 2
dt2 = 2
dt3 = 5

l1 = 5
l2 = 3
l3 = 1

viaPoint = 2

T = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [1, dt1, dt1**2, dt1**3, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, dt2, dt2**2, dt2**3, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 1, dt3, dt3**2, dt3**3],
              [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2 * dt3, 3 * (dt3**2)],
              [0, 1, 2 * dt1, 3 * (dt1**2), 0, -1, 0, 0, 0, 0, 0, 0],
              [0, 0, 2, 6 * dt1, 0, 0, -2, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 1, 2 * dt2, 3 * (dt2**2), 0, -1, 0, 0],
              [0, 0, 0, 0, 0, 0, 2, 6 * dt2, 0, 0, -2, 0]])

# Cartesian-space
"""
5-6 exam 2
C = np.array([
    [-4, 0, 90],
    [0, 3, 45],
    [0, 3, 45],
    [3, 3, 30],
    [3, 3, 30],
    [4, 0, 0],
])
"""
C = np.array([
    [-4, 0, 120],
    [-5, 5, 45],
    [-5, 5, 45],
    [2, 3, 30],
    [2, 3, 30],
    [2, -3, 0],
])

if (np.linalg.det(T)):
    T_inv = np.linalg.inv(T)
else:
    print('error det !=0; no inverse')
#print(T_inv)
#print(np.matmul(T_inv, C_of_X))
"""
C_of_Q1 = np.array([2.50, 1.99, 1.99, 0.35, 0.35, -1.62, 0, 0, 0, 0, 0, 0])
C_of_Q2 = np.array([2.21, 1.0, 1.0, 2.35, 2.35, 2.35, 0, 0, 0, 0, 0, 0])
C_of_Q3 = np.array([-2.62, -0.9, -0.9, -0.6, -0.6, 1.37, 0, 0, 0, 0, 0, 0])
"""
#C_of_Q = np.ndarray(shape=(0, 3), dtype=float)
Q12x3 = []

# convert cartesian space to joint space: 3 segments 0-2s 2-4s 4-9s
for i in C:
    q123 = ik.ik(l1, l2, l3, i[0], i[1], radians(i[2]))
    ik.ik_delta(l1, l2, l3, i[0], i[1], radians(i[2]))
    print('q123:', q123)
    Q12x3 = np.append(Q12x3, q123)
# print (Q12x1)
#padding 0 for last 6 rows
# col:theta1~theat3 of the arm
Q12x3.resize((12, 3), refcheck=False)
print('theta:', Q12x3.real)

# 2 dicimal points accroding to 5-6 exm2
np.set_printoptions(precision=3, suppress=True)
A12x3 = (np.round(T_inv @ Q12x3, 2))

# make a0~a3 in one row, 0~1s,2~4s,4~9s in one group(3x4), each group represent one theta
# 3xseg, 3xtheta, 4xai
A12x3 = np.transpose(A12x3).reshape(3, 3, 4)
#A = np.reshape(A, (3, 3, 4))
# A is now a 3-D array
# row: theta, col:aij,
print('A12x3:', A12x3.real)
'''
each theta has 3 polyonmials (init, via & final)
and each seg(i) has 4 coefficients (ai0~ai3)
hence, each theta has 12 coefficients
'''

# q:0~2, seg: 0~2
def getCoefficients(q, segment):
    #a = np.empty([1, 4])
    a = []
    # return an arry of the 4 coefficients - ai0,ai1,ai2,ai3 (where i is seg)
    # Sj(t)=Aj+Bj*dt+Cj*dt**2+Dj*dt**3
    # Sj'(t) = Bj+2*Cj*dt+3*Dj*dt**2
    for x in range(4):
        a = np.append(a, A12x3[q, segment, x])
    return a
'''
ans:
5. 2.21//0//-0.83//0.27
6. -2.21//0.14//-0.61//0.05
'''


#print(getCoefficients(0, 1))
"""

get the 4 'a' coefficients for each joint
time in 0-2s
    for i in rang(3)
        use 0-2s' coeff to compute Sj(t) & Sj'(t) for each joint[i]
        # https://www.youtube.com/watch?v=xyOQ4J4BroE
        servo[i].moveTo(pos, vel)
time in 2-4s
    same as above

"""
