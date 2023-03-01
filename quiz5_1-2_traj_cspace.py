# quiz5 @ coursera's robotics by ntu
import numpy as np

# refer to the table where the question given.
dt = dt1 = dt2 = 2
dt3 = 5
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

# conditions
C_of_X = np.array([-4, -5, -5, 2, 2, 2, 0, 0, 0, 0, 0, 0])
C_of_Y = np.array([0, 5, 5, 3, 3, -3, 0, 0, 0, 0, 0, 0])
C_of_theta = np.array([120, 45, 45, 30, 30, 0, 0, 0, 0, 0, 0, 0])
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
# padding the rest with 0 to form a 12x3 matrix
C.resize(12,3)
#print(C)
# output x col, y col and theta col, each col has ai0~ai3 (total 3 lines)
print('A:', np.round(T_inv @ C, 2))


'''quiz5
ans:
    1. -5//1.44//2.19//-0.58
    2.  5.00//1.67//-2.08//0.37
'''