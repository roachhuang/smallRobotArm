"""Velocity Controller Module

This module provides velocity control functionality for the small robot arm,
including joint-space and Cartesian-space velocity control with smoothing.

Classes:
    VelocityController: Velocity controller for the robot arm
"""

import logging
import time
from typing import Callable, Dict, List

import numpy as np
from numpy import ndarray

from .base_controller import BaseController
from ..kinematics import jacobians as jk

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Tunable parameters
# ---------------------------------------------------------------------------

# Per-joint velocity / acceleration / jerk limits
JOINT_VEL_LIMIT_RAD = np.array([2.0] * 6)   # rad/s, shape (6,) — ndarray so
                                              # array ops in _smooth_q_dot are correct
JOINT_ACC_LIMIT     = [5.0]  * 6             # rad/s²
JOINT_JERK_LIMIT    = [10.0] * 6             # rad/s³

# DLS / manipulability parameters
DLS_EPSILON              = 0.01   # σ_min threshold below which damping activates
MANIPULABILITY_THRESHOLD = 0.01   # minimum manipulability before warning
LAMBDA_MAX               = 0.1    # maximum DLS damping coefficient
LAMBDA_MIN               = 1e-4   # floor to prevent numerical blowup

# EMA smoothing bounds
ALPHA_MIN = 0.1
ALPHA_MAX = 0.9

# Debug log interval (steps) — logs once per second at 50 Hz
DEBUG_LOG_EVERY = 50


class VelocityController(BaseController):
    """Velocity controller for the small robot arm.

    Provides joint-space and Cartesian-space velocity control with adaptive
    EMA smoothing, dynamic DLS singularity avoidance, and structured logging.

    Args:
        robot:  Robot model exposing ``jacob0()`` and joint-angle interfaces.
        debug:  If True, log diagnostics every ``DEBUG_LOG_EVERY`` steps.
    """

    def __init__(self, robot, debug: bool = False):
        super().__init__(robot)

        self.debug = debug

        self.joint_limits: Dict[str, List[float]] = {
            'vel':  list(JOINT_VEL_LIMIT_RAD),
            'acc':  JOINT_ACC_LIMIT,
            'jerk': JOINT_JERK_LIMIT,
        }

        # Per-joint velocity ceiling — ndarray so division in _smooth_q_dot
        # produces a correctly shaped (6,) result, not a list-of-scalars.
        self.max_q_dot_rad: ndarray = JOINT_VEL_LIMIT_RAD.copy()

        # Internal EMA state — reset by _run_control_loop before each motion
        self._q_dot_prev: ndarray = np.zeros(6)

        # Cartesian controller parameters
        self.dls_epsilon              = DLS_EPSILON
        self.manipulability_threshold = MANIPULABILITY_THRESHOLD

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _smooth_q_dot(self, q_dot_raw: ndarray) -> ndarray:
        """Smooth joint velocity using time-optimal scaling and adaptive EMA.

        Args:
            q_dot_raw: Raw joint velocities in rad/s, shape (6,).

        Returns:
            Smoothed joint velocities in rad/s, shape (6,).
        """
        # Guard against NaN / Inf — hold last good velocity
        if not np.all(np.isfinite(q_dot_raw)):
            return self._q_dot_prev.copy()

        # Time-optimal scaling: uniformly scale down if any joint exceeds limit
        max_ratios = np.abs(q_dot_raw / self.max_q_dot_rad)
        scaling    = 1.0 / np.max(max_ratios) if np.any(max_ratios > 1.0) else 1.0
        q_dot_scaled = q_dot_raw * scaling

        # Adaptive alpha: higher speed → lower alpha → more smoothing (sigmoid)
        max_q_dot   = np.max(self.max_q_dot_rad)
        speed_ratio = (
            np.linalg.norm(q_dot_scaled) / max_q_dot
            if max_q_dot > 1e-6 else 0.0
        )
        alpha = np.clip(
            0.5 / (1.0 + np.exp(5.0 * (speed_ratio - 0.5))),
            ALPHA_MIN, ALPHA_MAX,
        )
        # Normalise EMA decay for the actual control period so smoothing
        # behaviour is consistent regardless of dt
        alpha_dt = 1.0 - (1.0 - alpha) ** (self.dt / 0.005)

        q_dot = alpha_dt * q_dot_scaled + (1.0 - alpha_dt) * self._q_dot_prev
        self._q_dot_prev = q_dot
        return q_dot

    def _compute_dls_inverse(self, J: ndarray) -> tuple[ndarray, ndarray, float]:
        """Compute the Damped Least Squares pseudo-inverse of the Jacobian.

        Implements Nakamura & Hanafusa (1986) variable damping: λ = 0 when
        σ_min ≥ ε (well-conditioned), ramping quadratically to LAMBDA_MAX
        as σ_min → 0 (approaching singularity).

        The system (JJᵀ + λI) X = J is solved via LU decomposition rather
        than ``pinv``. This is valid because the damped matrix is guaranteed
        strictly positive-definite (λ > 0), making LU faster (~3–5×) and
        numerically cleaner than a full SVD-based pseudoinverse.

        Args:
            J: Jacobian matrix, shape (6, n_joints).

        Returns:
            Tuple of:
              J_dls     — DLS inverse, shape (n_joints, 6).
              S         — Singular values, shape (6,), descending.
              lambda_sq — Damping coefficient actually applied.
        """
        _, S, _ = np.linalg.svd(J)
        # smallest singular value — the real risk signal
        sigma_min = float(S[-1])

        # Ramp damping only below the singularity threshold
        if sigma_min >= self.dls_epsilon:
            lambda_sq = 0.0
        else:
            ratio     = sigma_min / self.dls_epsilon
            lambda_sq = (LAMBDA_MAX ** 2) * (1.0 - ratio ** 2)
        lambda_sq = max(lambda_sq, LAMBDA_MIN)
        # In the equation $JJ^T + lambda^2I, this lambda^2 term acts as a numerical floor.  Even if $JJ^T$ is singular and has zero eigenvalues, the combined matrix will have eigenvalues no smaller than lambda^2. This ensures the matrix is "well-conditioned" and the inverse doesn't explode
        A = J @ J.T + lambda_sq * np.eye(J.shape[0])
        try:
            J_dls = np.linalg.solve(A, J).T
        except np.linalg.LinAlgError:
            # Fallback: raise damping floor and retry once
            log.warning("DLS solve failed (lambda_sq=%.2e); retrying with "
                        "increased damping.", lambda_sq)
            A_fallback = J @ J.T + 0.01 * np.eye(J.shape[0])
            J_dls = np.linalg.solve(A_fallback, J).T

        return J_dls, S, lambda_sq

    def _run_control_loop(
        self,
        n_steps: int,
        tick: Callable[[int, float], None],
    ) -> None:
        """Run a timed control loop, calling ``tick(i, t)`` each step.

        Handles EMA state reset, real-time synchronisation, and exception
        routing so individual control methods only implement their per-tick
        logic.

        Recoverable errors (``ValueError``, ``LinAlgError``) log a warning
        and break the loop cleanly. All other exceptions trigger ``go_home()``
        before re-raising so the arm is always safe-stopped.

        Args:
            n_steps: Total number of control steps.
            tick:    Callable invoked as ``tick(step_index, elapsed_time_s)``.
        """
        self._q_dot_prev = np.zeros(6)
        start_time = time.perf_counter()

        for i in range(n_steps):
            t = i * self.dt
            try:
                tick(i, t)
            except (ValueError, np.linalg.LinAlgError) as e:
                log.warning("Recoverable error at step %d: %s", i, e)
                break
            except Exception as e:
                log.error("Unrecoverable error at step %d: %s", i, e)
                self.go_home()
                raise

            next_time = start_time + (i + 1) * self.dt
            time.sleep(max(0.0, next_time - time.perf_counter()))

    # ------------------------------------------------------------------
    # Public control loops
    # ------------------------------------------------------------------

    def joint_space_vel_ctrl(
        self,
        q_dot_func: Callable[[float], ndarray],
        duration: float,
    ) -> None:
        """Apply closed-loop joint-space velocity control.

        Re-reads joint angles from hardware each tick so that communication
        latency, motor slip, or a missed command cannot cause the integrated
        position to drift from actual joint positions.

        Args:
            q_dot_func: Returns a 6D joint velocity vector (rad/s) at time t.
            duration:   Total control duration in seconds.
        """
        n_steps = int(duration / self.dt)

        def tick(i: int, t: float) -> None:
            # Closed-loop: re-read encoder positions every tick
            q_rad     = np.radians(self.current_angles)
            q_dot_raw = q_dot_func(t)
            q_dot     = self._smooth_q_dot(q_dot_raw)
            q_rad    += q_dot * self.dt
            self.move_to_angles(np.degrees(q_rad), header='g', ack=True)

        self._run_control_loop(n_steps, tick)

    def cartesian_space_vel_ctrl(
        self,
        x_dot_func: Callable[[float], ndarray],
        duration: float,
    ) -> None:
        """Cartesian-space velocity control with DLS singularity avoidance.

        Re-reads joint angles from hardware each tick (closed-loop) to prevent
        drift accumulation.

        Args:
            x_dot_func: Returns a 6D end-effector twist [omega, v] (rad/s,
                mm/s) at t, robot_tools' canonical MR block order.
            duration:   Total control duration in seconds.
        """
        n_steps = int(duration / self.dt)

        def tick(i: int, t: float) -> None:
            # Closed-loop: re-read encoder positions every tick
            q_rad = np.radians(self.current_angles)
            # Named producer, not a raw robot.jacob0() call — J already in
            # robot_tools' canonical MR block order ([omega; v]).
            J     = jk.jacobian_geometric_from_rtb(self.robot, q_rad)

            J_dls, S, lambda_sq = self._compute_dls_inverse(J)

            w = float(np.prod(S))

            if w < self.manipulability_threshold:
                log.warning("Low manipulability at step %d: %.4f", i, w)

            if self.debug and i % DEBUG_LOG_EVERY == 0:
                log.debug(
                    "[step %4d] S=%s, w=%.4f, λ²=%.6f, ‖J_dls‖=%.4f",
                    i, np.round(S, 4), w, lambda_sq,
                    np.linalg.norm(J_dls),
                )

            x_dot     = x_dot_func(t)
            q_dot_raw = J_dls @ x_dot
            q_dot     = self._smooth_q_dot(q_dot_raw)
            q_rad    += q_dot * self.dt
            self.move_to_angles(np.degrees(q_rad), header='g', ack=True)

        self._run_control_loop(n_steps, tick)

    # ------------------------------------------------------------------
    # Analysis utilities
    # ------------------------------------------------------------------

    def get_optimal_directions(self, U: ndarray, Vt: ndarray) -> Dict[str, ndarray]:
        """Return task-space and joint-space principal motion directions.

        Args:
            U:   Left singular vectors of the Jacobian, shape (6, 6).
            Vt:  Right singular vectors (transposed), shape (6, 6).

        Returns:
            Dict with keys: easiest_cartesian_motion, hardest_cartesian_motion,
            most_effective_joint, least_effective_joint.
        """
        return {
            'easiest_cartesian_motion': U[:, 0],
            'hardest_cartesian_motion': U[:, -1],
            'most_effective_joint':     Vt[0, :],
            'least_effective_joint':    Vt[-1, :],
        }

    def is_near_singularity(self, S: ndarray, threshold: float = 100.0) -> bool:
        """Return True if the condition number of J exceeds threshold.

        Args:
            S:         Singular values of the Jacobian.
            threshold: Condition number limit (default 100).
        """
        return bool(S[0] / S[-1] > threshold)

    def null_space_directions(
        self, S: ndarray, Vt: ndarray, tol: float = 0.01
    ) -> ndarray:
        """Return joint-space directions that produce no end-effector motion.

        Args:
            S:   Singular values of the Jacobian.
            Vt:  Right singular vectors (transposed).
            tol: Singular value threshold below which a direction is null.
        """
        return Vt[S < tol]

   
    def adaptive_control_gains(
        self, S: ndarray
    ) -> tuple[ndarray, ndarray]:
        """Return position and velocity gain matrices scaled by singular values.

        Args:
            S: Singular values of the Jacobian.

        Returns:
            Tuple of (Kp, Kv) diagonal gain matrices.
        """
        return np.diag(S * 100.0), np.diag(np.sqrt(S) * 20.0)