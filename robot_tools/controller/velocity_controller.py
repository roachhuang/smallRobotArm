"""Velocity Controller Module

This module provides velocity control functionality for the small robot arm,
including joint-space and Cartesian-space velocity control with smoothing.

Classes:
    VelocityController: Velocity controller for the robot arm
"""

import time
from typing import Callable, Dict, List

import numpy as np
from numpy import ndarray

from .base_controller import BaseController

# ---------------------------------------------------------------------------
# Tunable parameters
# ---------------------------------------------------------------------------

# Per-joint velocity / acceleration / jerk limits
JOINT_VEL_LIMIT_RAD  = [2.0] * 6   # rad/s — one per joint
JOINT_ACC_LIMIT      = [5.0] * 6                          # rad/s²
JOINT_JERK_LIMIT     = [10.0] * 6                         # rad/s³

# DLS / manipulability parameters
DLS_EPSILON               = 0.01   # fallback damping factor
MANIPULABILITY_THRESHOLD  = 0.01   # minimum manipulability before warning
W_MAX                     = 0.01   # nominal manipulability (tune per arm)
LAMBDA_MAX                = 0.1    # maximum DLS damping coefficient
LAMBDA_MIN                = 1e-4   # floor to prevent numerical blowup

# EMA smoothing bounds
ALPHA_MIN = 0.1
ALPHA_MAX = 0.9

# Debug print interval (steps) — prints once per second at 50 Hz
DEBUG_PRINT_EVERY = 50


class VelocityController(BaseController):
    """Velocity controller for the small robot arm.

    Provides joint-space and Cartesian-space velocity control with adaptive
    EMA smoothing, dynamic DLS singularity avoidance, and optional debug
    logging.

    Args:
        robot: Robot model exposing ``jacob0()`` and joint-angle interfaces.
        debug: If True, print diagnostics every ``DEBUG_PRINT_EVERY`` steps.
    """

    def __init__(self, robot, debug: bool = False):
        super().__init__(robot)

        self.debug = debug

        self.joint_limits: Dict[str, List[float]] = {
            'vel':  list(JOINT_VEL_LIMIT_RAD),
            'acc':  JOINT_ACC_LIMIT,
            'jerk': JOINT_JERK_LIMIT,
        }

        # Per-joint velocity ceiling — shape (6,) so array ops are correct
        self.max_q_dot_rad: ndarray = JOINT_VEL_LIMIT_RAD.copy()

        # Internal EMA state
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
        scaling = 1.0 / np.max(max_ratios) if np.any(max_ratios > 1.0) else 1.0
        q_dot_scaled = q_dot_raw * scaling

        # Adaptive alpha: higher speed → lower alpha → more smoothing (sigmoid)
        max_q_dot  = np.max(self.max_q_dot_rad)
        speed_ratio = (
            np.linalg.norm(q_dot_scaled) / max_q_dot
            if max_q_dot > 1e-6 else 0.0
        )
        alpha = np.clip(
            0.5 / (1.0 + np.exp(5.0 * (speed_ratio - 0.5))),
            ALPHA_MIN, ALPHA_MAX,
        )
        # Adjust EMA coefficient for the actual control period
        alpha_dt = 1.0 - (1.0 - alpha) ** (self.dt / 0.005)

        q_dot = alpha_dt * q_dot_scaled + (1.0 - alpha_dt) * self._q_dot_prev
        self._q_dot_prev = q_dot
        return q_dot

    def _compute_dls_inverse(self, J: ndarray) -> tuple[ndarray, ndarray, float]:
        U, S, Vt = np.linalg.svd(J)
        sigma_min = float(S[-1])

        # Nakamura & Hanafusa: ramp lambda only below threshold
        if sigma_min >= self.dls_epsilon:
            lambda_sq = 0.0
        else:
            ratio     = sigma_min / self.dls_epsilon
            lambda_sq = (LAMBDA_MAX ** 2) * (1.0 - ratio ** 2)
        lambda_sq = max(lambda_sq, LAMBDA_MIN)

        A = J @ J.T + lambda_sq * np.eye(6)
        try:
            # solve is faster and cleaner than pinv for guaranteed-invertible A
            J_dls = np.linalg.solve(A, J).T
        except np.linalg.LinAlgError:
            # fallback: increase damping floor and retry
            A_fallback = J @ J.T + 0.01 * np.eye(6)
            J_dls = np.linalg.solve(A_fallback, J).T

        return J_dls, S, lambda_sq

    # ------------------------------------------------------------------
    # Public control loops
    # ------------------------------------------------------------------

    def joint_space_vel_ctrl(
        self,
        q_dot_func: Callable[[float], ndarray],
        duration: float,
    ) -> None:
        """Apply joint-space velocity control.

        Args:
            q_dot_func: Returns a 6D joint velocity vector (rad/s) at time t.
            duration: Total control duration in seconds.
        """
        q_rad      = np.radians(self.current_angles)
        n_steps    = int(duration / self.dt)
        self._q_dot_prev = np.zeros(6)
        start_time = time.perf_counter()

        for i in range(n_steps):
            t = i * self.dt
            try:
                q_dot_raw = q_dot_func(t)
                q_dot     = self._smooth_q_dot(q_dot_raw)
                q_rad    += q_dot * self.dt
                self.move_to_angles(np.degrees(q_rad), header='g', ack=True)

            except (ValueError, np.linalg.LinAlgError) as e:
                print(f"[WARN] Recoverable error at step {i}: {e}")
                break
            except Exception as e:
                print(f"[ERROR] Unrecoverable error at step {i}: {e}")
                self.go_home()
                raise

            next_time = start_time + (i + 1) * self.dt
            time.sleep(max(0.0, next_time - time.perf_counter()))

    def cartesian_space_vel_ctrl(
        self,
        x_dot_func: Callable[[float], ndarray],
        duration: float,
    ) -> None:
        """Cartesian-space velocity control with DLS singularity avoidance.

        Re-reads joint angles from hardware each tick (closed-loop) to prevent
        drift accumulation.

        Args:
            x_dot_func: Returns a 6D end-effector velocity [mm/s, rad/s] at t.
            duration:   Total control duration in seconds.
        """
        n_steps    = int(duration / self.dt)
        self._q_dot_prev = np.zeros(6)
        start_time = time.perf_counter()

        for i in range(n_steps):
            t = i * self.dt
            try:
                # Closed-loop: read actual encoder positions each tick
                q_rad = np.radians(self.current_angles)
                J     = self.robot.jacob0(q_rad)

                J_dls, S, lambda_sq = self._compute_dls_inverse(J)

                if self.debug and i % DEBUG_PRINT_EVERY == 0:
                    w = float(np.prod(S))
                    print(f"[step {i:4d}] S={np.round(S, 4)}, "
                          f"w={w:.4f}, λ²={lambda_sq:.6f}, "
                          f"‖J_dls‖={np.linalg.norm(J_dls):.4f}")

                if float(np.prod(S)) < self.manipulability_threshold:
                    print(f"[WARN] Low manipulability at step {i}: {np.prod(S):.4f}")

                x_dot     = x_dot_func(t)
                q_dot_raw = J_dls @ x_dot
                q_dot     = self._smooth_q_dot(q_dot_raw)
                q_rad    += q_dot * self.dt
                self.move_to_angles(np.degrees(q_rad), header='g', ack=True)

            except (ValueError, np.linalg.LinAlgError) as e:
                print(f"[WARN] Recoverable error at step {i}: {e}")
                break
            except Exception as e:
                print(f"[ERROR] Unrecoverable error at step {i}: {e}")
                self.go_home()
                raise

            next_time = start_time + (i + 1) * self.dt
            time.sleep(max(0.0, next_time - time.perf_counter()))

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

    def null_space_directions(self, S: ndarray, Vt: ndarray,
                              tol: float = 0.01) -> ndarray:
        """Return joint-space directions that produce no end-effector motion.

        Args:
            S:   Singular values of the Jacobian.
            Vt:  Right singular vectors (transposed).
            tol: Singular value threshold below which a direction is null.
        """
        return Vt[S < tol]

    def force_analysis(self, desired_force: ndarray, q: ndarray) -> Dict[str, ndarray]:
        """Compute joint torques and force difficulty for a desired wrench.

        Args:
            desired_force: 6D desired end-effector wrench.
            q:             Joint angles in radians.

        Returns:
            Dict with joint_torques, force_difficulty, hardest_force_direction.
        """
        J = self.robot.compute_jacobian(q)
        _, S, _ = np.linalg.svd(J)
        return {
            'joint_torques':          J.T @ desired_force,
            'force_difficulty':       1.0 / S,
            'hardest_force_direction': S[-1],   # smallest singular value direction
        }

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