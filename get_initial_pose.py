import numpy as np
from scipy.spatial.transform import Rotation as R

a1, a2, a3 = 47.0, 110.0, 26.0
d1, d4, d6 = 133.0, 117.50, 28.0

std_dh_table = [
    {"d": d1, "a": a1, "alpha": -np.pi / 2, "offset": 0},
    {"d": 0, "a": a2, "alpha": 0, "offset": -np.pi / 2},
    {"d": 0, "a": a3, "alpha": -np.pi / 2, "offset": 0},
    {"d": d4, "a": 0, "alpha": np.pi / 2, "offset": 0},
    {"d": 0, "a": 0, "alpha": -np.pi / 2, "offset": 0},
    {"d": d6, "a": 0, "alpha": 0, "offset": 0},
]

def dh_to_transformation(row):
    d = row["d"]
    a = row["a"]
    alpha = row["alpha"]
    theta = row["offset"] #initial joint angle is the offset

    T = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return T

T_total = np.eye(4)
for row in std_dh_table:
    T_total = T_total @ dh_to_transformation(row)

print('t-total', T_total)

# Extract position
position = T_total[:3, 3]
print("Initial Position:", position)

# Extract rotation (as Euler angles)
rotation_matrix = T_total[:3, :3]
r = R.from_matrix(rotation_matrix)
euler_angles = r.as_euler("xyz", degrees=True)
print("Initial Euler Angles (XYZ):", euler_angles)