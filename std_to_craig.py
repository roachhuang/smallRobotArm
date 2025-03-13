import numpy as np
import math

class RevoluteDH:
    def __init__(self, d, a, alpha, offset=0):
        self.d = d
        self.a = a
        self.alpha = alpha
        self.offset = offset

def craig_dh_transformation(theta, offset, d, a, alpha):
    """
    Calculates the Craig DH transformation matrix.

    Args:
        theta: Joint variable (in radians).
        offset: Joint variable offset (in radians).
        d, a, alpha: Craig DH parameters.

    Returns:
        4x4 NumPy array representing the transformation matrix.
    """

    theta_adjusted = theta + offset  # Apply the offset

    cos_theta = math.cos(theta_adjusted)
    sin_theta = math.sin(theta_adjusted)
    cos_alpha = math.cos(alpha)
    sin_alpha = math.sin(alpha)

    # Craig DH Transformation Matrix (Different Order!)
    T = np.array([
        [cos_theta, -sin_theta, 0, a],
        [sin_theta * cos_alpha, cos_theta * cos_alpha, -sin_alpha, -d * sin_alpha],
        [sin_theta * sin_alpha, cos_theta * sin_alpha, cos_alpha, d * cos_alpha],
        [0, 0, 0, 1]
    ])

    return T

# Your DH parameters
a1, a2, a3 = 47.0, 110.0, 26.0
d1, d4, d6 = 133.0, 117.50, 28.0

# Your standard DH table
std_dh_table = [
    RevoluteDH(d=d1, a=a1, alpha=-np.pi / 2),  # joint 1
    RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi / 2),  # joint 2
    RevoluteDH(d=0, a=a3, alpha=-np.pi / 2),  # joint 3
    RevoluteDH(d=d4, a=0, alpha=np.pi / 2),  # joint 4
    RevoluteDH(d=0, a=0, alpha=-np.pi / 2),  # joint 5
    RevoluteDH(d=d6, a=0, alpha=0),  # joint 6
]

# Create Craig DH table (parameters are the same)
craig_dh_table = [RevoluteDH(d=link.d, a=link.a, alpha=link.alpha, offset=link.offset) for link in std_dh_table]

# Example usage (with example DH parameters)
theta = np.pi / 4  # Example joint variable
offset = craig_dh_table[1].offset # get the offset for joint 2.
d = craig_dh_table[1].d
a = craig_dh_table[1].a
alpha = craig_dh_table[1].alpha

T_craig = craig_dh_transformation(theta, offset, d, a, alpha)
print("\nCraig DH Transformation matrix for joint 2 example:\n", T_craig)

# Standard DH Transformation (for comparison)
def standard_dh_transformation(theta, offset, d, a, alpha):
    theta_adjusted = theta + offset
    cos_theta = math.cos(theta_adjusted)
    sin_theta = math.sin(theta_adjusted)
    cos_alpha = math.cos(alpha)
    sin_alpha = math.sin(alpha)

    T = np.array([
        [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
        [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1]
    ])
    return T

T_standard = standard_dh_transformation(theta, offset, d, a, alpha)
print("\nStandard DH Transformation matrix for joint 2 example:\n", T_standard)

print("\nAre the matricies different?:", not np.array_equal(T_craig, T_standard))