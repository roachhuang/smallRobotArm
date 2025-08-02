
from roboticstoolbox import RevoluteDH
import numpy as np

"""
d: link offset 
a: link length
alpha: link twist
offset: kinematic-joint variable offset
r6=distance btw axis6 and end-effector
"""
# Robot DH parameters (mm)
a1, a2, a3 = 47.0, 110.0, 26.0
d1, d4, d6 = 133.0, 117.50, 28.0

# Standard DH table for robotics toolbox
std_dh_tbl = [
    RevoluteDH(d=d1, a=a1, alpha=-np.pi / 2),           # Joint 1
    RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi / 2),  # Joint 2 (with offset)
    RevoluteDH(d=0, a=a3, alpha=-np.pi / 2),            # Joint 3
    RevoluteDH(d=d4, a=0, alpha=np.pi / 2),             # Joint 4
    RevoluteDH(d=0, a=0, alpha=-np.pi / 2),             # Joint 5
    RevoluteDH(d=d6, a=0, alpha=0),                     # Joint 6
]

# DH parameters array [alpha, a, d] for custom kinematics
std_dh_params = np.array([
    [-np.pi/2, a1, d1],
    [0, a2, 0],
    [-np.pi/2, a3, 0],
    [np.pi/2, 0, d4],
    [-np.pi/2, 0, 0],
    [0, 0, d6],
])
