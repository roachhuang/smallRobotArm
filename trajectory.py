# quiz5 @ coursera's robotics by ntu

from math import radians
import numpy as np
from inverse_kinematic import ik

# refer to the table where the question given.
dt = dt1 = dt2 = 2
dt3 = 5

l1 = 5
l2 = 3
l3 = 1

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
C = np.array([
    [-4, 0, 120],
    [-5, 5, 45],
    [-5, 5, 45],
    [2, 3, 30],
    [2, 3, 30],
    [2, -3, 0],
])
T_inv = np.linalg.inv(T)
#print(T_inv)
#print(np.matmul(T_inv, C_of_X))
np.set_printoptions(precision=3, suppress=True)
"""
C_of_Q1 = np.array([2.50, 1.99, 1.99, 0.35, 0.35, -1.62, 0, 0, 0, 0, 0, 0])
C_of_Q2 = np.array([2.21, 1.0, 1.0, 2.35, 2.35, 2.35, 0, 0, 0, 0, 0, 0])
C_of_Q3 = np.array([-2.62, -0.9, -0.9, -0.6, -0.6, 1.37, 0, 0, 0, 0, 0, 0])
"""
#C_of_Q = np.ndarray(shape=(0, 3), dtype=float)
C_of_Q = []

# convert cartesian space to joint space: 3 lines 0-2s 2-4s 4-9s
for i in C:
    q123=ik(l1, l2, l3, i[0], i[1], radians(i[2]))
    #print(q1to3)
    C_of_Q = np.append(C_of_Q, q123)

#padding 0 for last 6 rows
C_of_Q.resize(12,3)
#print(C_of_Q)
print(np.round(T_inv @ C_of_Q, 2))
