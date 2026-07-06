"""Numerical inverse kinematics for robot_tools.

Implements Modern Robotics Ch. 6.2 (Numerical Inverse Kinematics) as a thin
wrapper around the ``modern_robotics`` library -- no reinvention of the
Newton-Raphson core.

Book alignment
--------------
- MR 6.2.1: Newton-Raphson root finding.
- MR 6.2.2: the twist-form iteration actually used by the library,

      theta^{i+1} = theta^i + pinv(J(theta^i)) @ V            (MR Eq. 6.7 form)

  where V is the twist (space or body frame) that takes T(theta^i) to T_sd
  in unit time, computed via the matrix log:  [V_b] = log(T^{-1}(theta) T_sd).
- MR 6.6 (Software): ``IKinSpace(Slist, M, T, thetalist0, eomg, ev)``.

We use the *space-frame* variant to match the rest of robot_tools
(FKinSpace / JacobianSpace / Slist).  IKinBody is mathematically equivalent
(MR 6.2.2); only the frame in which the error twist and Jacobian are
expressed differs.

Twist convention: everything here is canonical MR ``[omega; v]``.

Robustness note (MR 6.2, Fig. 6.7): plain Newton-Raphson converges only for
seeds "close to" a solution and returns success=False otherwise.  The
optional restart layer re-seeds by sampling *within joint limits* (not
[-pi, pi]) until convergence -- see ``solve_ik_with_restarts``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import modern_robotics as mr
import numpy as np

# MR 6.6 default tolerances used in the book's Example 6.1:
# eomg = 0.001 rad (~0.057 deg), ev = 1e-4 m (100 microns).
DEFAULT_EOMG = 1e-3
DEFAULT_EV = 1e-4


@dataclass
class IKResult:
    """Outcome of a numerical IK solve.

    Attributes
    ----------
    q : np.ndarray
        Joint vector.  Meaningful only if ``success`` is True (on failure it
        is the last iterate, per mr.IKinSpace semantics).
    success : bool
        Whether both error tolerances (eomg, ev) were met within the
        library's max iterations (20 in modern_robotics).
    seed : np.ndarray
        The initial guess that produced this result.
    n_restarts_used : int
        0 = converged from the caller's seed; k = converged on the k-th
        random restart.
    """

    q: np.ndarray
    success: bool
    seed: np.ndarray = field(default_factory=lambda: np.array([]))
    n_restarts_used: int = 0


def solve_ik(
    Slist: np.ndarray,
    M: np.ndarray,
    T_sd: np.ndarray,
    q0: np.ndarray,
    eomg: float = DEFAULT_EOMG,
    ev: float = DEFAULT_EV,
) -> IKResult:
    """Single-shot numerical IK.  Thin wrapper over ``mr.IKinSpace``.

    Parameters mirror MR 6.6 exactly so the book can be used as the manual:

    Slist : (6, n) space-frame screw axes, MR [omega; v] convention
            (columns S_i, as used by FKinSpace/JacobianSpace).
    M     : (4, 4) end-effector home configuration.
    T_sd  : (4, 4) desired end-effector configuration in {s}.
    q0    : (n,) initial guess.  Convergence basin is local (MR Fig. 6.7);
            with multiple IK solutions, Newton-Raphson finds the one
            "closest" to q0 (MR 6.2.2) -- so seed with the current joint
            state, or with the closed-form geometric solution when
            available (MR 6.2 intro recommends exactly this hybrid).
    eomg  : tolerance on ||omega|| of the error twist [rad].
    ev    : tolerance on ||v|| of the error twist [m].
    """
    q0 = np.asarray(q0, dtype=float)
    q, success = mr.IKinSpace(Slist, M, T_sd, q0, eomg, ev)
    q = np.asarray(q, dtype=float)
    return IKResult(q=q, success=bool(success), seed=q0)


def solve_ik_with_restarts(
    Slist: np.ndarray,
    M: np.ndarray,
    T_sd: np.ndarray,
    q0: np.ndarray,
    joint_limits: np.ndarray,
    n_restarts: int = 20,
    eomg: float = DEFAULT_EOMG,
    ev: float = DEFAULT_EV,
    rng: np.random.Generator | None = None,
    wrap_to_limits: bool = True,
) -> IKResult:
    """Numerical IK with random-restart robustness layer.

    Strategy (unchanged math -- only the seeding is augmented):

    1. Try the caller's seed ``q0`` first (cheapest, and honors the
       "closest solution" behavior of MR 6.2.2).
    2. On failure, re-seed with samples drawn uniformly *within
       joint_limits* -- not [-pi, pi] -- and retry, up to ``n_restarts``
       times.  Sampling inside the reachable joint space keeps seeds in
       physically meaningful basins of attraction.
    3. First convergent solution wins.  If ``wrap_to_limits``, each
       revolute solution is wrapped by 2*pi into the limit interval when
       possible; a solution that cannot be brought within limits is
       treated as a failure and the search continues.

    joint_limits : (n, 2) array of [lower, upper] per joint [rad].
    rng          : np.random.Generator for reproducible restarts
                   (default: np.random.default_rng()).
    """
    joint_limits = np.asarray(joint_limits, dtype=float)
    n = joint_limits.shape[0]
    if joint_limits.shape != (n, 2):
        raise ValueError(f"joint_limits must be (n, 2), got {joint_limits.shape}")
    rng = rng if rng is not None else np.random.default_rng()

    def _attempt(seed: np.ndarray, k: int) -> IKResult | None:
        res = solve_ik(Slist, M, T_sd, seed, eomg, ev)
        if not res.success:
            return None
        q = _wrap_into_limits(res.q, joint_limits) if wrap_to_limits else res.q
        if q is None:
            return None  # converged, but outside joint limits -> keep searching
        return IKResult(q=q, success=True, seed=seed, n_restarts_used=k)

    result = _attempt(np.asarray(q0, dtype=float), 0)
    if result is not None:
        return result

    for k in range(1, n_restarts + 1):
        seed = rng.uniform(joint_limits[:, 0], joint_limits[:, 1])
        result = _attempt(seed, k)
        if result is not None:
            return result

    return IKResult(q=np.asarray(q0, dtype=float), success=False,
                    seed=np.asarray(q0, dtype=float), n_restarts_used=n_restarts)


def _wrap_into_limits(q: np.ndarray, joint_limits: np.ndarray) -> np.ndarray | None:
    """Wrap each revolute joint value by multiples of 2*pi into its limits.

    mr.IKinSpace is unconstrained, so a converged solution may sit at e.g.
    q_i = 7.1 rad when the equivalent 0.82 rad is within limits.  Returns
    the wrapped vector, or None if any joint cannot be placed in-limits.
    """
    q = np.array(q, dtype=float)
    lo, hi = joint_limits[:, 0], joint_limits[:, 1]
    for i in range(q.shape[0]):
        if lo[i] <= q[i] <= hi[i]:
            continue
        # candidate equivalents q_i + 2*pi*m
        m_lo = np.ceil((lo[i] - q[i]) / (2 * np.pi))
        m_hi = np.floor((hi[i] - q[i]) / (2 * np.pi))
        if m_lo > m_hi:
            return None
        # pick the equivalent closest to the interval midpoint
        mids = q[i] + 2 * np.pi * np.arange(m_lo, m_hi + 1)
        q[i] = mids[np.argmin(np.abs(mids - 0.5 * (lo[i] + hi[i])))]
    return q


def ik_pose_error(Slist: np.ndarray, M: np.ndarray, T_sd: np.ndarray,
                  q: np.ndarray) -> tuple[float, float]:
    """(||omega_b||, ||v_b||) of the body error twist at q -- the exact
    quantities mr.IKinSpace tests against (eomg, ev).  Useful in tests and
    for verifying solutions independently of the solver."""
    T = mr.FKinSpace(M, Slist, q)
    Vb = mr.se3ToVec(mr.MatrixLog6(np.linalg.inv(T) @ T_sd))
    return float(np.linalg.norm(Vb[:3])), float(np.linalg.norm(Vb[3:]))