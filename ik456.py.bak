"""
Inverse Kinematics for Robot Arm Joints 4, 5, 6 (Euler ZYZ Solution)
====================================================================

This module provides functions to solve the inverse kinematics (IK) for the last three joints (q4, 5, q6)
of a 6-DOF robot arm using the Euler ZYZ convention. The robot's Denavit-Hartenberg (DH) parameters are
assumed to be set in the `craig` module.

References:
-----------
- UCLA IK notes: https://univ.deltamoocx.net/courses/course-v1:AT+AT_010_1102+2022_02_01/courseware/a3e573de127b85f1dcb23ea797cd253f/dc947a72e470ca516e9270c3bb4424e1/?child=first

Note:
-----
- This implementation is tailored for a specific robot configuration and may not work for all robots (e.g., PUMA 650).
- The code assumes the use of the `craig` module for DH transformations and the `spatialmath` library for SO3 rotations.
"""

from math import atan2, cos, sin, asin, sqrt, pi
import numpy as np
import craig as cg
from sympy import symbols, simplify
from spatialmath import SO3
from typing import Sequence


def ver456(
    r3_6: np.ndarray,
    q4s: Sequence[float],
    q5s: Sequence[float],
    q6s: Sequence[float],
) -> np.ndarray:
    """
    Verify candidate solutions for q4, q5, q6 by comparing the computed rotation matrix
    with the desired r3_6 matrix.

    Parameters
    ----------
    r3_6 : np.ndarray
        Desired 3x3 rotation matrix from frame 3 to 6.
    q4s, q5s, q6s : Sequence[float]
        Sequences of candidate joint angles (in radians) for joints 4, 5, 6.

    Returns
    -------
    np.ndarray
        Array of valid [q4, q5, q6] solutions (in radians), shape (N, 3).
    """
    q456s = np.empty(shape=[0, 3], dtype=np.float64)
    for t4 in q4s:
        for t5 in q5s:
            for t6 in q6s:
                t4_3 = cg.get_ti2i_1(4, t4)
                t5_4 = cg.get_ti2i_1(5, t5)
                t6_5 = cg.get_ti2i_1(6, t6)
                t3_6 = t4_3 @ t5_4 @ t6_5
                R3_6 = t3_6[0:3, 0:3]

                # np.set_printoptions(precision=15) # Set precision to 15 decimal places
                # print("r3_6 with higher precision:\n", r3_6)
                # print("\nR3_6 with higher precision:\n", R3_6)
                # np.set_printoptions(precision=8) # Reset precision to default (optional)

                if np.allclose(
                    r3_6.astype(np.float64),
                    R3_6.astype(np.float64),
                    rtol=1e-3,
                    atol=1e-3,
                ):
                    print(
                        f"verify: q4:{np.degrees(t4)}, q5:{np.degrees(t5)}, q6:{np.degrees(t6)}"
                    )
                    q456s = np.append(q456s, [t4, t5, t6])
    return q456s.reshape(-1, 3)  # Reshape to ensure it's a 2D array with 3 columns


def ik456(
    r0_6: np.ndarray,
    t1: float,
    t2: float,
    t3: float,
) -> np.ndarray:
    """
    Compute possible solutions for joints 4, 5, 6 (q4, q5, q6) given the end-effector
    rotation matrix and the first three joint angles.

    Parameters
    ----------
    r0_6 : np.ndarray
        3x3 rotation matrix from base to end-effector.
    t1, t2, t3 : float
        Joint angles (in radians) for joints 1, 2, 3.

    Returns
    -------
    np.ndarray
        Array of valid [q4, q5, q6] solutions (in radians), shape (N, 3).
    """
    q4s = []
    q5s = []
    q6s = []
    """
    q4,q5,q6=euler angles phi, theta, psi. see unit5 part3
    R3_6=Rz,phi Ry,theat Rz,psi = Rz,q4 Ry,q5 Rz,q6
    (q1,q2,q3)->R3-0->R6-3=(R3-0)t * R6-0->q4,q5,q6
    use standard DH table to compute R3-0
    1) R6-3=R4-3(q4)R5-4(q5)R6-5(q6) see unit5 part3
    2) also R6-3(q4,q5,q6) = (R3-0)t * R6-0
    for (1) = (2)

    """
    # after compute q1-3, we can know R3_0
    # T3_0=T1_0@T2_1@T3_2
    """
    r3_0=np.array([[c1c23, -c1s23, s1],
     [s1c23,    -s1s23,  -c1 ],
     [s23,  c23,  0   ])
    """
    t2_3 = cg.get_ti2i_1(3, t3)
    t1_2 = cg.get_ti2i_1(2, t2)
    t0_1 = cg.get_ti2i_1(1, t1)
    t0_3 = t0_1 @ t1_2 @ t2_3
    r0_3 = t0_3[0:3, 0:3]

    alp0 = cg.dh_tbl[0, 0]
    alp1 = cg.dh_tbl[1, 0]
    alp2 = cg.dh_tbl[2, 0]
    alp3 = cg.dh_tbl[3, 0]
    SO_R0_3 = (
        SO3.Rx(alp0, unit="rad")
        @ SO3.Rz(t1)
        @ SO3.Rx(alp1)
        @ SO3.Rz(t2)
        @ SO3.Rx(alp2)
        @ SO3.Rz(t3)
    )

    # to align with zyz, we need to rotx(alp3) wrt R3_4 to start rot w/ z axis on joint 4.
    r0_3prime = r0_3 @ cg.Rot("x", alp3)

    r3_6 = r0_3.T @ r0_6

    # r3_6prime = np.linalg.inv(r0_3prime) @ r0_6
    r3_6prime = r0_3prime.T @ r0_6

    r13 = r3_6prime[0, 2]
    r23 = r3_6prime[1, 2]
    r31 = r3_6prime[2, 0]
    r32 = r3_6prime[2, 1]
    r33 = r3_6prime[2, 2]
    beta1 = atan2(sqrt(r31**2 + r32**2), r33)
    beta2 = atan2(-sqrt(r31**2 + r32**2), r33)

    alp1 = atan2(r23 / sin(beta1), r13 / sin(beta1))
    alp2 = atan2(r23 / sin(beta2), r13 / sin(beta2))

    gamma1 = atan2(r32 / sin(beta1), -r31 / sin(beta1))
    gamma2 = atan2(r32 / sin(beta2), -r31 / sin(beta2))

    # map zyz euler angles to dh angles
    q4s = np.append(q4s, [alp1 + pi, alp2 + pi])
    q4s = np.append(q4s, [alp1 - pi, alp2 - pi])
    q4s_lmt = q4s[(q4s >= -pi/2) & (q4s <= pi/2)]

    q5s = np.append(q5s, [beta1, beta2])
    q5s_lmt = q5s[(q5s >= -pi / 2) & (q5s <= pi / 2)]

    q6s = np.append(q6s, [gamma1 + pi, gamma2 + pi])
    q6s = np.append(q6s, [gamma1 - pi, gamma2 - pi])
    q6s_lmt = q6s[(q6s >= -pi/2) & (q6s <= pi/2)]

    # q456s=np.row_stack((ans1, ans2))

    # https://www.meccanismocomplesso.org/en/3d-rotations-and-euler-angles-in-python/

    print(f"q4= {np.rad2deg(q4s)}, q5= {np.rad2deg(q5s)}, q6= {np.rad2deg(q6s)}")
    return ver456(r3_6, q4s_lmt, q5s_lmt, q6s_lmt)


def main():
    # this is not ans to quiz. it is for ppt.   
    # pose = (227,372,188.6, 0, -30, 180) on ppt
    dh_tbl = np.array(
        [
            [0, 0, 0],
            [np.radians(-90), -30, 0],
            [0, 340, 0],
            [np.radians(-90), -40, 338],
            [np.radians(90), 0, 0],
            [np.radians(-90), 0, 0],
        ]
    )
    cg.setDhTbl(dh_tbl)
    t123s = np.array(
        [
            [
                np.radians(58.61),
                np.radians(-64.46),
                np.radians(-11.98),
            ],
            # this elbow down solution is still mathmatical correct but will hit the table, so don't use it.
            # [np.radians(58.61), np.radians(20.37), np.radians(178.48)],
        ]
    )
    # on ppt sect. 7-1 @ 16:13
    r0_6 = np.array([[-0.866, 0, 0.5], [0, -1, 0], [0.5, 0, 0.866]])
    for t1, t2, t3 in t123s:
        ik456(r0_6, t1, t2, t3)


if __name__ == "__main__":
    main()
