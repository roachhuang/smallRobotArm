"""Time-optimal time scaling under per-joint kinematic limits.

Modern Robotics Ch. 9.4 (Time-Optimal Time Scaling), adapted for
stepper-actuated arms: the binding constraints are per-joint velocity
(step rate) and acceleration limits rather than torque limits, so the
path dynamics (Eq. 9.32) reduce to pure kinematics:

    q(t)      = theta(s(t))
    q_dot     = theta'(s) * s_dot                      (chain rule)
    q_ddot    = theta'(s) * s_ddot + theta''(s) * s_dot^2

Per-joint limits |q_dot_i| <= qd_max_i and |q_ddot_i| <= qdd_max_i
translate into the phase-plane constraints of Eq. 9.37:

    s_dot <= sdot_lim(s)          (velocity limit curve, Sec. 9.4.1)
    L(s, s_dot) <= s_ddot <= U(s, s_dot)

with, per joint i (compare Eq. 9.36, where theta'_i plays the role of
the "effective inertia" m_i(s)):

    theta'_i != 0:  s_ddot in [(-qdd_i - theta''_i sdot^2)/theta'_i,
                               (+qdd_i - theta''_i sdot^2)/theta'_i]
                    (interval flips when theta'_i < 0)
    theta'_i == 0:  zero-inertia point (Sec. 9.4.4): joint i constrains
                    s_dot only, via |theta''_i| sdot^2 <= qdd_i.

Algorithm
---------
Instead of the switch-point search of Sec. 9.4.2 (steps 2-6), we use
the equivalent, numerically robust discretized construction on a grid
s_0..s_N (work in w = s_dot^2, since dw/ds = 2 s_ddot is linear):

  1. Velocity limit curve  -> textbook Fig. 9.10/9.11 "limit curve":
     largest w(s) satisfying the q_dot limits AND L <= U feasibility.
  2. Forward pass  (integrate s_ddot = U from (0,0), clamped to the
     limit curve)  -> the union of all "A_i" max-acceleration curves;
     clamping reproduces the tangent points found by the binary search
     in step 4 of the textbook algorithm.
  3. Backward pass (integrate s_ddot = L from (1,0), clamped)
     -> the "F" max-deceleration curve(s) of step 2.
  4. Pointwise min of the two passes -> the bang-bang optimal profile;
     switches S are where the active pass changes.

Time is recovered by integrating dt = ds / s_dot.

The result is exactly time optimal for the discretized problem; grid
resolution trades accuracy for planning time (default 2000 points plans
in a few ms for a 6-DOF arm).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

__all__ = [
    "KinematicLimits",
    "PathFunction",
    "TimeOptimalTimeScaling",
    "joint_straight_line_path",
]

_EPS = 1e-12


@dataclass(frozen=True)
class KinematicLimits:
    """Per-joint symmetric velocity/acceleration limits.

    For steppers: qd_max is the max step rate converted to rad/s
    (steps_per_sec * 2*pi / (steps_per_rev * microstep * gear_ratio)),
    qdd_max the max ramp rate the driver/torque margin allows.
    """

    qd_max: np.ndarray   # (n,) rad/s, > 0
    qdd_max: np.ndarray  # (n,) rad/s^2, > 0

    def __post_init__(self):
        qd = np.asarray(self.qd_max, dtype=float)
        qdd = np.asarray(self.qdd_max, dtype=float)
        if np.any(qd <= 0) or np.any(qdd <= 0):
            raise ValueError("limits must be strictly positive")
        object.__setattr__(self, "qd_max", qd)
        object.__setattr__(self, "qdd_max", qdd)


class PathFunction:
    """Wraps a geometric path theta(s), s in [0, 1], with derivatives.

    theta : (float) -> (n,) joint vector. Must be C2 (or close to it)
            for the second derivative to be meaningful.
    dtheta, ddtheta : optional analytic derivatives w.r.t. s. When not
            supplied, central finite differences are used.
    """

    def __init__(
        self,
        theta: Callable[[float], np.ndarray],
        dtheta: Optional[Callable[[float], np.ndarray]] = None,
        ddtheta: Optional[Callable[[float], np.ndarray]] = None,
        fd_step: float = 1e-6,
    ):
        self._theta = theta
        self._dtheta = dtheta
        self._ddtheta = ddtheta
        self._h = fd_step

    def theta(self, s: float) -> np.ndarray:
        return np.asarray(self._theta(float(np.clip(s, 0.0, 1.0))), dtype=float)

    def dtheta(self, s: float) -> np.ndarray:
        if self._dtheta is not None:
            return np.asarray(self._dtheta(s), dtype=float)
        h = self._h
        s0, s1 = np.clip(s - h, 0.0, 1.0), np.clip(s + h, 0.0, 1.0)
        return (self.theta(s1) - self.theta(s0)) / (s1 - s0)

    def ddtheta(self, s: float) -> np.ndarray:
        if self._ddtheta is not None:
            return np.asarray(self._ddtheta(s), dtype=float)
        h = 1e-4  # larger step: 2nd-order FD is noise-sensitive
        s0, s1 = np.clip(s - h, 0.0, 1.0), np.clip(s + h, 0.0, 1.0)
        sm = 0.5 * (s0 + s1)
        return (self.theta(s1) - 2.0 * self.theta(sm) + self.theta(s0)) / ((0.5 * (s1 - s0)) ** 2)


def joint_straight_line_path(q_start: np.ndarray, q_end: np.ndarray) -> PathFunction:
    """Straight line in joint space: theta(s) = q_start + s*(q_end-q_start).

    With constant kinematic limits this reduces the optimal scaling to
    the trapezoidal profile of MR Sec. 9.2.2.2 - useful as a unit test.
    """
    q0 = np.asarray(q_start, dtype=float)
    dq = np.asarray(q_end, dtype=float) - q0
    return PathFunction(
        theta=lambda s: q0 + s * dq,
        dtheta=lambda s: dq,
        ddtheta=lambda s: np.zeros_like(dq),
    )


@dataclass
class _Profile:
    """Planned phase-plane profile on the s grid, plus time map."""

    s: np.ndarray        # (N,) grid over [0, 1]
    sdot: np.ndarray     # (N,) optimal s_dot(s)
    sddot: np.ndarray    # (N,) s_ddot(s) realized by the profile
    t: np.ndarray        # (N,) t(s_k), t[-1] = total time
    switches: np.ndarray  # s values where accel/decel phase changes

    @property
    def total_time(self) -> float:
        return float(self.t[-1])


class TimeOptimalTimeScaling:
    """Time-optimal s(t) for a given path under kinematic limits.

    >>> path = joint_straight_line_path(q0, q1)
    >>> planner = TimeOptimalTimeScaling(path, limits, n_grid=2000)
    >>> profile = planner.plan()
    >>> t, q, qd, qdd = planner.sample(dt=0.01)
    >>> qd_fn = planner.q_dot_func()   # for VelocityController streaming
    """

    def __init__(self, path: PathFunction, limits: KinematicLimits, n_grid: int = 2000):
        if n_grid < 10:
            raise ValueError("n_grid too small")
        self.path = path
        self.limits = limits
        self.n_grid = int(n_grid)
        self._profile: Optional[_Profile] = None
        # Precompute path derivatives on the grid (planning bottleneck).
        self._s = np.linspace(0.0, 1.0, self.n_grid)
        self._dth = np.array([path.dtheta(s) for s in self._s])    # (N, n)
        self._ddth = np.array([path.ddtheta(s) for s in self._s])  # (N, n)

    # ---------------------------------------------------------------- limits

    def _w_limit(self, k: int) -> float:
        """Largest w = s_dot^2 at grid point k satisfying all constraints.

        Combines (a) the q_dot velocity limit curve and (b) feasibility
        L <= U of the acceleration interval (Sec. 9.4.1: above the limit
        curve the motion cone vanishes), including zero-"inertia" joints.
        """
        dth, ddth = self._dth[k], self._ddth[k]
        qd, qdd = self.limits.qd_max, self.limits.qdd_max

        w = np.inf
        # (a) |theta'_i| sdot <= qd_i
        nz = np.abs(dth) > _EPS
        if np.any(nz):
            w = min(w, float(np.min((qd[nz] / np.abs(dth[nz])) ** 2)))
        # (b) zero-inertia joints: |theta''_i| w <= qdd_i (Sec. 9.4.4)
        zi = ~nz & (np.abs(ddth) > _EPS)
        if np.any(zi):
            w = min(w, float(np.min(qdd[zi] / np.abs(ddth[zi]))))
        # (c) pairwise feasibility L <= U for active joints: bisect on w.
        if np.any(nz) and self._LU(k, max(w, 0.0))[0] > self._LU(k, max(w, 0.0))[1]:
            lo, hi = 0.0, w
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                L, U = self._LU(k, mid)
                if L <= U:
                    lo = mid
                else:
                    hi = mid
            w = lo
        return w

    def _LU(self, k: int, w: float) -> tuple[float, float]:
        """Acceleration interval [L, U] on s_ddot at grid point k, w = sdot^2.

        Kinematic analogue of Eq. 9.36 with m_i -> theta'_i,
        c_i sdot^2 -> theta''_i sdot^2, g_i -> 0, tau -> +-qdd_i.
        """
        dth, ddth = self._dth[k], self._ddth[k]
        qdd = self.limits.qdd_max
        L, U = -np.inf, np.inf
        for i in range(dth.size):
            di = dth[i]
            if abs(di) <= _EPS:
                continue  # zero-inertia joint: constrains w only
            a = (-qdd[i] - ddth[i] * w) / di
            b = (+qdd[i] - ddth[i] * w) / di
            lo_i, hi_i = (a, b) if di > 0 else (b, a)
            L, U = max(L, lo_i), min(U, hi_i)
        return L, U

    # ------------------------------------------------------------- planning

    def plan(self) -> _Profile:
        s, N = self._s, self.n_grid
        ds = s[1] - s[0]

        w_lim = np.array([self._w_limit(k) for k in range(N)])

        # Forward pass: s_ddot = U (max accel), clamp to limit curve.
        # Textbook steps 3-4: the clamp is the tangent-point construction.
        w_fwd = np.zeros(N)
        for k in range(N - 1):
            _, U = self._LU(k, w_fwd[k])
            w_fwd[k + 1] = min(w_fwd[k] + 2.0 * U * ds, w_lim[k + 1])

        # Backward pass: s_ddot = L (max decel) from (1, 0). Step 2.
        w_bwd = np.zeros(N)
        for k in range(N - 1, 0, -1):
            L, _ = self._LU(k, w_bwd[k])
            w_bwd[k - 1] = min(w_bwd[k] - 2.0 * L * ds, w_lim[k - 1])

        w = np.minimum(w_fwd, w_bwd)
        w[0] = w[-1] = 0.0
        sdot = np.sqrt(np.maximum(w, 0.0))

        # Realized s_ddot from d(w)/ds = 2 s_ddot (for q_ddot reporting).
        sddot = np.gradient(w, s) / 2.0

        # Time map: dt = ds / sdot; near sdot=0 use constant-accel form
        # dt = (sdot_{k+1} - sdot_k) / s_ddot to stay integrable.
        t = np.zeros(N)
        for k in range(N - 1):
            pair = sdot[k] + sdot[k + 1]
            if pair > 1e-9:
                dt = 2.0 * ds / pair
            else:
                acc = max(abs(sddot[k]), _EPS)
                dt = np.sqrt(2.0 * ds / acc)
            t[k + 1] = t[k] + dt

        # Switch points: sign changes of the active phase (accel<->decel).
        active_bwd = w_bwd < w_fwd
        switches = s[1:][active_bwd[1:] != active_bwd[:-1]]

        self._profile = _Profile(s=s, sdot=sdot, sddot=sddot, t=t, switches=switches)
        return self._profile

    # ------------------------------------------------------------- sampling

    def _require_plan(self) -> _Profile:
        return self._profile if self._profile is not None else self.plan()

    def s_of_t(self, t: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(s, s_dot, s_ddot) at times t via interpolation of the plan."""
        p = self._require_plan()
        tq = np.clip(np.atleast_1d(t), 0.0, p.total_time)
        s = np.interp(tq, p.t, p.s)
        sd = np.interp(tq, p.t, p.sdot)
        sdd = np.interp(tq, p.t, p.sddot)
        return s, sd, sdd

    def sample(self, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Uniform-time samples (t, q, q_dot, q_ddot), incl. the endpoint."""
        p = self._require_plan()
        t = np.arange(0.0, p.total_time, dt)
        if t[-1] < p.total_time:
            t = np.append(t, p.total_time)
        s, sd, sdd = self.s_of_t(t)
        q = np.array([self.path.theta(si) for si in s])
        dth = np.array([self.path.dtheta(si) for si in s])
        ddth = np.array([self.path.ddtheta(si) for si in s])
        qd = dth * sd[:, None]
        qdd = dth * sdd[:, None] + ddth * (sd**2)[:, None]
        return t, q, qd, qdd

    def q_dot_func(self) -> Callable[[float], np.ndarray]:
        """q_dot(t) closure for VelocityController.joint_space_vel_ctrl()."""
        def qd_fn(t: float) -> np.ndarray:
            s, sd, _ = self.s_of_t(np.array([t]))
            return self.path.dtheta(float(s[0])) * float(sd[0])
        return qd_fn

    # ----------------------------------------------------------- diagnostics

    def feasibility_report(self, dt: float = 0.005) -> dict:
        """Max limit utilization (should be <= ~1.0 + grid error)."""
        _, _, qd, qdd = self.sample(dt)
        return {
            "total_time": self._require_plan().total_time,
            "qd_util_max": float(np.max(np.abs(qd) / self.limits.qd_max)),
            "qdd_util_max": float(np.max(np.abs(qdd) / self.limits.qdd_max)),
            "n_switches": int(self._require_plan().switches.size),
        }
