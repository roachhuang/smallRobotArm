"""Canonical Jacobian and twist conventions for robot_tools.

Modern Robotics Ch. 5 (Velocity Kinematics and Statics).

CONVENTION (canonical, everywhere inside robot_tools)
-----------------------------------------------------
Twists are V = [omega; v] (angular first), as in MR Eq. 3.70 and the
`modern_robotics` library. Jacobians stack rows the same way:

    J = [ J_omega ]   (3 x n)
        [ J_v     ]   (3 x n)

Two DISTINCT foreign conventions exist at the roboticstoolbox boundary:

1. Block order: rtb stacks [v; omega]. Fixed by `swap_twist_blocks` /
   `swap_jacobian_blocks` (involutions: applying twice is identity).
2. Reference point: rtb's `jacob0` is the GEOMETRIC Jacobian - its
   linear rows give the velocity of the end-effector point. MR's
   space Jacobian (Eq. 5.11) gives the SPATIAL twist - the linear
   rows give the velocity of the body-frame point instantaneously
   coincident with the space-frame origin. Related by
       p_dot_ee = v_s + omega x p_ee
   Fixed by `spatial_to_geometric` / `geometric_to_spatial`.

Only cross this boundary through the named converters below; never
reorder or re-reference twist blocks inline.
"""

from __future__ import annotations

import numpy as np
import modern_robotics as mr

__all__ = [
    "jacobian_space_from_rtb",
    "jacobian_geometric_from_rtb",
    "jacobian_space",
    "jacobian_body",
    "body_jacobian_from_space",
    "swap_twist_blocks",
    "swap_jacobian_blocks",
    "spatial_to_geometric",
    "geometric_to_spatial",
    "mr_to_rtb_jacobian",
    "rtb_to_mr_jacobian",
    "sigma_min",
    "joint_torques_from_wrench",
]


# ------------------------------------------------------------ producers
# The ONLY Jacobian producers in robot_tools. Ch. 5.1.1 / 5.1.2.

def jacobian_space(Slist: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Space Jacobian J_s(q), MR Eq. 5.11. Rows [omega; v], frame {s}."""
    return mr.JacobianSpace(np.asarray(Slist), np.asarray(q))


def jacobian_body(Blist: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Body Jacobian J_b(q), MR Eq. 5.18. Rows [omega; v], frame {b}."""
    return mr.JacobianBody(np.asarray(Blist), np.asarray(q))


def body_jacobian_from_space(
    J_s: np.ndarray, Slist: np.ndarray, M: np.ndarray, q: np.ndarray
) -> np.ndarray:
    """J_b = [Ad_{T_bs}] J_s (MR Eq. 5.22), T_bs = FKinSpace(M,S,q)^-1.

    Prefer `jacobian_body(Blist, q)` when Blist is available; this is
    for callers that only hold Slist/M (e.g. SmallRbtArm.Slist_body, M).
    """
    T_sb = mr.FKinSpace(M, Slist, q)
    return mr.Adjoint(mr.TransInv(T_sb)) @ J_s


# ----------------------------------------------------------- converters
# Foreign-convention boundary. Involutions where noted.

def swap_twist_blocks(V: np.ndarray) -> np.ndarray:
    """[omega; v] <-> [v; omega] for a 6-vector. Involution."""
    V = np.asarray(V)
    return np.concatenate([V[3:6], V[0:3]])


def swap_jacobian_blocks(J: np.ndarray) -> np.ndarray:
    """Swap the top/bottom 3-row blocks of a 6xn Jacobian. Involution."""
    J = np.asarray(J)
    return np.vstack([J[3:6, :], J[0:3, :]])


def spatial_to_geometric(J_s: np.ndarray, p_ee: np.ndarray) -> np.ndarray:
    """Re-reference the linear rows from the {s}-origin to the EE point.

    p_dot_ee = v_s + omega x p_ee  =>  J_geo_v = J_s_v - [p_ee] J_s_omega
    Output still stacked [omega; v] (MR block order), linear rows now
    giving the EE-point velocity, i.e. rtb's jacob0 content.
    """
    J_s = np.asarray(J_s)
    p = np.asarray(p_ee)
    Jw, Jv = J_s[0:3, :], J_s[3:6, :]
    return np.vstack([Jw, Jv - mr.VecToso3(p) @ Jw])


def geometric_to_spatial(J_geo: np.ndarray, p_ee: np.ndarray) -> np.ndarray:
    """Inverse of `spatial_to_geometric`."""
    J = np.asarray(J_geo)
    p = np.asarray(p_ee)
    Jw, Jv = J[0:3, :], J[3:6, :]
    return np.vstack([Jw, Jv + mr.VecToso3(p) @ Jw])


def rtb_to_mr_jacobian(J_rtb0: np.ndarray, p_ee: np.ndarray) -> np.ndarray:
    """rtb `jacob0` ([v;omega], geometric) -> MR space Jacobian."""
    return geometric_to_spatial(swap_jacobian_blocks(J_rtb0), p_ee)


def mr_to_rtb_jacobian(J_s: np.ndarray, p_ee: np.ndarray) -> np.ndarray:
    """MR space Jacobian -> rtb `jacob0` ([v;omega], geometric)."""
    return swap_jacobian_blocks(spatial_to_geometric(J_s, p_ee))


# ------------------------------------------------------------- analysis

def sigma_min(J_or_S: np.ndarray) -> float:
    """Smallest singular value; near-singularity measure (MR Sec. 5.4).

    Frame-invariant in the sense that J_b and J_s share rank, but the
    numeric value differs between them (Adjoint is not orthogonal) -
    compare thresholds against ONE convention. robot_tools standard:
    evaluate on the SPACE Jacobian.

    Accepts either a Jacobian matrix (SVD computed here) or an
    already-computed 1-D array of singular values, so callers that also
    need the full spectrum (e.g. for manipulability) don't pay for a
    second SVD.
    """
    arr = np.asarray(J_or_S)
    S = arr if arr.ndim == 1 else np.linalg.svd(arr, compute_uv=False)
    return float(S[-1])


def joint_torques_from_wrench(J: np.ndarray, F: np.ndarray) -> np.ndarray:
    """Static joint torques tau = J^T F (MR Eq. 5.26).

    Frame consistency is on the caller: pass (J_b, F_b) or (J_s, F_s),
    never mixed. Wrench stacked [m; f] to match [omega; v] twists.
    """
    return np.asarray(J).T @ np.asarray(F)


# ------------------------------------------------- rtb robot-handle boundary

def jacobian_space_from_rtb(robot, q: np.ndarray) -> np.ndarray:
    """MR space Jacobian from an rtb DHRobot handle.

    For call sites that only hold a roboticstoolbox robot (e.g. the
    controllers): converts robot.jacob0 (geometric, [v; omega]) into the
    MR space Jacobian ([omega; v], {s}-origin referenced).
    """
    q = np.asarray(q)
    p_ee = robot.fkine(q).A[0:3, 3]
    return rtb_to_mr_jacobian(robot.jacob0(q), p_ee)


def jacobian_geometric_from_rtb(robot, q: np.ndarray) -> np.ndarray:
    """Geometric Jacobian in MR block order from an rtb DHRobot handle.

    Same content as robot.jacob0 (linear rows = EE-point velocity), but
    stacked [omega; v] to match the robot_tools canonical order.
    """
    return swap_jacobian_blocks(robot.jacob0(np.asarray(q)))
