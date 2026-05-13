"""Waypoint Trajectory Planning Module

Demonstrates joint-space waypoint trajectory planning for a 6-DOF robot arm
using quintic S-curve velocity profiles. The planning pipeline is fully
decoupled from the hardware execution rate — profiles are pure functions of a
normalised parameter u ∈ [0, 1]; wall-clock time mapping happens only at
execution time inside the controller.

Pipeline:
    1. Load named trajectory config (waypoints + timestamps)
    2. Convert Cartesian waypoints → joint space via IK (with validation)
    3. Optimise joint path for manipulability
    4. Build quintic S-curve velocity profiles (pure functions, no time baked in)
    5. Validate peak velocities against joint limits
    6. Execute segments via joint-space velocity control
    7. Save velocity profile plot (non-blocking)

Example:
    python time_optimal_traj.py

Author: Robot Control Team
Date: 2024
"""

import logging
import sys
from time import sleep
from typing import Callable, Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from numpy import ndarray

from roboticstoolbox import DHRobot

from robot_tools.kinematics import SmallRbtArm, std_dh_params, std_dh_tbl
from robot_tools.controller import VelocityController
from robot_tools.misc.signal_handler import setup_signal_handler
from robot_tools.trajectory.path_optimizer import PathOptimizer

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Trajectory configurations
# Switch active trajectory by changing ACTIVE_CONFIG — no source edits needed.
# ---------------------------------------------------------------------------

TRAJECTORY_CONFIGS: Dict[str, Dict] = {
    "lift_and_slide": {
        "description": "Lift then long horizontal slide",
        # Timestamp (s) at which each waypoint must be reached
        "timestamps": [0.0, 2.0, 20.0, 24.0],
        # [x mm, y mm, z mm, roll deg, pitch deg, yaw deg] in desk frame
        "waypoints": np.array([
            (-200, 160,   60, 0.0,   0.0, 35.0),
            (-200, 160,   90, 0.0,   0.0, 35.0),
            (-295, 270,  350, 0.0, -60.0,  0.0),
            (-295, 350,  350, 0.0, -60.0,  0.0),
        ], dtype=np.float64),
    },
    "original": {
        "description": "Original short lift + rotate sequence",
        "timestamps": [0.0, 8.0, 10.0, 24.0],
        "waypoints": np.array([
            (-200, 100,   60, 0.0,   0.0, 35.0),
            (-200, 100,   90, 0.0,   0.0, 35.0),
            (-295, 170,  350, 0.0, -60.0,  0.0),
            (-295, 270,  350, 0.0, -60.0,  0.0),
        ], dtype=np.float64),
    },
}

ACTIVE_CONFIG = "lift_and_slide"

# ---------------------------------------------------------------------------
# Planning constants — independent of hardware execution rate
# ---------------------------------------------------------------------------

# Sampling resolution for display only.
# Execution evaluates the profile callable on-demand at controller.dt.
DISPLAY_DT         = 0.005   # s (200 Hz) — fixed, never controller.dt
MIN_MANIPULABILITY = 0.05    # manipulability floor for path optimisation

# Hardware settle times
SIGNAL_SETTLE_S = 1   # s — after signal handler registration
ENABLE_SETTLE_S = 1   # s — after controller.enable()


# ---------------------------------------------------------------------------
# Pure velocity profile — no closure over time or duration
# ---------------------------------------------------------------------------

def quintic_scurve_velocity(
    u: float,
    displacement: ndarray,
    duration: float,
) -> ndarray:
    """Evaluate the quintic S-curve velocity at normalised parameter u ∈ [0, 1].

    Uses the smoothstep polynomial s(u) = 10u³ - 15u⁴ + 6u⁵ which satisfies:
        s(0) = 0,  s(1) = 1
        s'(0) = s'(1) = 0    (zero velocity at segment endpoints)
        s''(0) = s''(1) = 0  (zero acceleration at segment endpoints)

    Time is deliberately NOT closed over. The caller maps wall-clock t to
    u = t / duration, keeping profile shape independent of timing so profiles
    can be replayed, speed-scaled, or tested without re-planning.

    Peak velocity occurs at u = 0.5:
        v_peak = (15/8) * displacement / duration

    Args:
        u:            Normalised segment position, clipped to [0, 1].
        displacement: Joint displacement for this segment (rad), shape (n,).
        duration:     Segment duration in seconds (for unit scaling only).

    Returns:
        Joint velocity vector in rad/s, shape (n,).
    """
    u = float(np.clip(u, 0.0, 1.0))
    s_dot = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / duration
    return displacement * s_dot


def make_segment_vel_func(
    displacement: ndarray,
    duration: float,
) -> Callable[[float], ndarray]:
    """Return a velocity callable f(t) → rad/s for one trajectory segment.

    This is the single point where wall-clock time enters the system.
    The returned function maps t → u = t / duration and delegates to the
    pure ``quintic_scurve_velocity``. Both arrays are snapshotted so the
    closure is independent of loop variables.

    Args:
        displacement: Joint displacement for this segment (rad), shape (n,).
        duration:     Segment duration in seconds.

    Returns:
        Callable ``f(t: float) -> ndarray`` for
        ``VelocityController.joint_space_vel_ctrl``.
    """
    _disp = displacement.copy()
    _dur  = float(duration)

    def velocity_func(t: float) -> ndarray:
        return quintic_scurve_velocity(t / _dur, _disp, _dur)

    return velocity_func


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def check_peak_velocity(
    displacement: ndarray,
    duration: float,
    max_q_dot_rad: ndarray,
    segment_index: int,
) -> None:
    """Raise ValueError if the S-curve peak velocity exceeds joint limits.

    For the quintic smoothstep the analytic peak is at u = 0.5:
        v_peak = (15/8) * |displacement| / duration

    Args:
        displacement:  Joint displacement (rad), shape (n,).
        duration:      Segment duration (s).
        max_q_dot_rad: Per-joint velocity ceiling (rad/s), shape (n,).
        segment_index: Used in the error message.

    Raises:
        ValueError: If any joint's peak velocity exceeds its rated limit.
    """
    peak_vel   = (15.0 / 8.0) * np.abs(displacement) / duration
    violations = peak_vel > max_q_dot_rad
    if np.any(violations):
        raise ValueError(
            f"Segment {segment_index} S-curve peak velocity exceeds joint limits.\n"
            f"  Peak  (deg/s): {np.round(np.degrees(peak_vel), 2)}\n"
            f"  Limit (deg/s): {np.round(np.degrees(max_q_dot_rad), 2)}\n"
            f"  Joints over limit: {np.where(violations)[0].tolist()}\n"
            "Increase segment duration or reduce waypoint displacement."
        )


# ---------------------------------------------------------------------------
# IK helper
# ---------------------------------------------------------------------------

def ik_all_waypoints(
    robot: SmallRbtArm,
    cartesian_path: ndarray,
) -> List[ndarray]:
    """Convert every Cartesian waypoint to joint angles via IK.

    Args:
        robot:          Custom robot model.
        cartesian_path: Array of shape (N, 6) — [x, y, z, rx, ry, rz].

    Returns:
        List of N joint-angle arrays in degrees.

    Raises:
        ValueError: If IK fails or returns non-finite values for any waypoint.
    """
    results: List[ndarray] = []
    for i, pose in enumerate(cartesian_path):
        T_06   = robot.convert_p_dc_to_T06(tuple(pose))
        joints = robot.ik(T_06)
        if joints is None or not np.all(np.isfinite(joints)):
            raise ValueError(
                f"IK failed for waypoint {i} — pose {pose}.\n"
                "Check reachability and singularity proximity."
            )
        results.append(np.asarray(joints, dtype=np.float64))
    return results


# ---------------------------------------------------------------------------
# Visualisation — samples at DISPLAY_DT, never at controller.dt
# ---------------------------------------------------------------------------

def plot_velocity_profiles(
    displacements: List[ndarray],
    durations: List[float],
    out_path: str = "velocity_profiles.png",
) -> None:
    """Sample S-curve profiles at DISPLAY_DT and save to file.

    Sampling is done here, at display time, using the fixed DISPLAY_DT
    constant so the plot is independent of both the planner and the
    hardware execution rate.

    Args:
        displacements: Per-segment joint displacements (rad).
        durations:     Per-segment durations (s).
        out_path:      Output PNG file path.
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    t_offset = 0.0

    for seg_idx, (disp, dur) in enumerate(zip(displacements, durations)):
        n_pts = max(2, int(np.ceil(dur / DISPLAY_DT)) + 1)
        t_seg = np.linspace(0.0, dur, n_pts)
        q_dot = np.array([
            quintic_scurve_velocity(t / dur, disp, dur) for t in t_seg
        ])
        for j in range(q_dot.shape[1]):
            label = f"Joint {j + 1}" if seg_idx == 0 else None
            ax.plot(t_seg + t_offset, q_dot[:, j], label=label)
        t_offset += dur

    ax.set_title("Joint Velocity Profiles (Quintic S-Curve)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (rad/s)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    log.info("Velocity profile plot saved to %s", out_path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Trajectory pipeline
# ---------------------------------------------------------------------------

def run_trajectory(
    controller: VelocityController,
    custom_robot: SmallRbtArm,
    path_optimizer: PathOptimizer,
    config: Dict,
) -> None:
    """Plan and execute the full waypoint trajectory.

    Separated from hardware setup so exceptions propagate to the try/finally
    in ``main()``, guaranteeing a safe-stop regardless of failure point.

    Args:
        controller:     Active VelocityController (hardware already enabled).
        custom_robot:   Custom kinematics model.
        path_optimizer: Manipulability optimiser.
        config:         Trajectory config dict with 'timestamps' and 'waypoints'.

    Raises:
        ValueError: On IK failure, peak velocity violation, or bad config.
    """
    timestamps:     List[float] = config["timestamps"]
    cartesian_path: ndarray     = config["waypoints"]

    if len(timestamps) != len(cartesian_path):
        raise ValueError(
            f"timestamps ({len(timestamps)}) and waypoints "
            f"({len(cartesian_path)}) must have equal length."
        )

    durations: List[float] = [
        timestamps[i] - timestamps[i - 1]
        for i in range(1, len(timestamps))
    ]

    # --- move to start position ---
    initial_joints_deg = ik_all_waypoints(custom_robot, cartesian_path[:1])[0]
    controller.move_to_angles(initial_joints_deg)
    input("Press Enter to start trajectory execution…")

    # --- IK for all waypoints ---
    log.info("Computing IK for %d waypoints…", len(cartesian_path))
    joint_path_deg = ik_all_waypoints(custom_robot, cartesian_path)
    joint_path_rad = np.array([np.radians(j) for j in joint_path_deg])

    # --- manipulability optimisation ---
    log.info("Optimising trajectory for manipulability…")
    joint_path_opt: ndarray = path_optimizer.optimize_manipulability_path(
        joint_path_rad, min_manipulability=MIN_MANIPULABILITY
    )
    for i, (orig, opt) in enumerate(zip(joint_path_rad, joint_path_opt)):
        orig_m: float = path_optimizer.eigen_analysis(orig)["manipulability"]
        opt_m:  float = path_optimizer.eigen_analysis(opt)["manipulability"]
        level = logging.WARNING if opt_m < MIN_MANIPULABILITY else logging.INFO
        log.log(level, "Waypoint %d: %.4f → %.4f%s",
                i, orig_m, opt_m,
                " ⚠ still below threshold" if opt_m < MIN_MANIPULABILITY else "")

    # --- build profiles + validate peak velocities (before any hardware move) ---
    log.info("Building S-curve profiles and validating velocity limits…")
    displacements: List[ndarray]                  = []
    vel_funcs:     List[Callable[[float], ndarray]] = []

    for i, dur in enumerate(durations):
        disp = joint_path_opt[i + 1] - joint_path_opt[i]
        check_peak_velocity(disp, dur, controller.max_q_dot_rad, i)
        displacements.append(disp)
        vel_funcs.append(make_segment_vel_func(disp, dur))

    # --- log summary ---
    log.info("Trajectory summary — config: %s (%s)",
             ACTIVE_CONFIG, config["description"])
    log.info("  Timestamps (s): %s", timestamps)
    log.info("  Durations  (s): %s", durations)
    for i, (disp, dur) in enumerate(zip(displacements, durations)):
        peak_deg = np.round(np.degrees((15.0 / 8.0) * np.abs(disp) / dur), 2)
        log.info("  Segment %d peak |vel| (deg/s): %s", i + 1, peak_deg)

    # --- execute ---
    log.info("Executing trajectory…")
    for seg_idx, (dur, vel_func) in enumerate(zip(durations, vel_funcs)):
        log.info("  Segment %d/%d: %.2f s", seg_idx + 1, len(durations), dur)
        controller.joint_space_vel_ctrl(q_dot_func=vel_func, duration=dur)
    log.info("Trajectory execution complete.")

    # --- plot after motion finishes — non-blocking, saved to file ---
    plot_velocity_profiles(displacements, durations)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> int:
    """Entry point — returns 0 on success, 1 on error."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    np.set_printoptions(precision=3, suppress=True)

    if ACTIVE_CONFIG not in TRAJECTORY_CONFIGS:
        log.error("Unknown config '%s'. Available: %s",
                  ACTIVE_CONFIG, list(TRAJECTORY_CONFIGS))
        return 1

    config = TRAJECTORY_CONFIGS[ACTIVE_CONFIG]
    log.info("Loaded config: %s — %s", ACTIVE_CONFIG, config["description"])

    toolbox_robot  = DHRobot(std_dh_tbl, name="smallRobotArm")
    toolbox_robot.ets()
    custom_robot   = SmallRbtArm(std_dh_params)
    controller     = VelocityController(toolbox_robot)
    path_optimizer = PathOptimizer(toolbox_robot)

    setup_signal_handler(controller)
    sleep(SIGNAL_SETTLE_S)

    controller.enable()
    sleep(ENABLE_SETTLE_S)

    try:
        run_trajectory(controller, custom_robot, path_optimizer, config)
    except ValueError as e:
        log.error("Planning error — aborting before any motion: %s", e)
        return 1
    except KeyboardInterrupt:
        log.info("Interrupted by user.")
    except Exception as e:
        log.error("Unexpected error: %s", e)
        return 1
    finally:
        log.info("Safe-stopping: returning home.")
        controller.go_home()

    return 0


if __name__ == "__main__":
    sys.exit(main())