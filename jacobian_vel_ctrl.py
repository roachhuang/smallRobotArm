"""Jacobian-based Velocity Control Demo

Demonstrates velocity control for a 6-DOF robot arm using Jacobian-based
kinematics. Includes motion patterns: circle, zigzag.
"""

import sys
from time import sleep

import numpy as np
from roboticstoolbox import DHRobot

from robot_tools.kinematics import SmallRbtArm, std_dh_params, std_dh_tbl
from robot_tools.controller import VelocityController
from robot_tools.misc.signal_handler import setup_signal_handler
from robot_tools.trajectory.motion_patterns import MotionPatterns

# ---------------------------------------------------------------------------
# Tunable parameters — adjust here without touching motion logic
# ---------------------------------------------------------------------------

# Pose (x, y, z mm, rx, ry, rz deg) at the circle start position in desk frame
CIRCLE_START_POSE = (-260, 100, 110.0, 0.0, 90.0, 0.0)

# Initial X-axis approach move before the circle
APPROACH_VEL_X_MM_S = 20.0        # mm/s
APPROACH_DURATION_S  = 2.5        # s

# Fourier circle parameters
CIRCLE_RADIUS_MM = 50             # mm
CIRCLE_PERIOD_S  = 15             # s — max ~15 s at this radius/speed

# Zigzag parameters
ZIGZAG_SIDE_MM  = 40              # mm
ZIGZAG_PERIOD_S = 30.0            # s

# Jacobian sanity-check tolerance
JACOBIAN_TOL = 1e-4               # max acceptable element-wise difference

# Hardware settle times (required by firmware, see datasheet §3.2)
SIGNAL_SETTLE_S  = 1              # s — after signal handler registration
ENABLE_SETTLE_S  = 1              # s — after controller.enable()
POSITION_SETTLE_S = 2             # s — after move_to_angles()


def check_jacobians(custom_robot: SmallRbtArm,
                    toolbox_robot: DHRobot,
                    q_test: np.ndarray) -> None:
    """Compare custom and toolbox Jacobians; raise if they diverge."""
    J_custom  = custom_robot.jacob0(q_test)
    J_toolbox = toolbox_robot.jacob0(q_test)
    max_diff  = np.max(np.abs(J_custom - J_toolbox))
    print(f"Jacobian shapes — custom: {J_custom.shape}, toolbox: {J_toolbox.shape}")
    print(f"Max element-wise difference: {max_diff:.6f}")
    assert max_diff < JACOBIAN_TOL, (
        f"Jacobian mismatch! Max diff {max_diff:.6f} exceeds tolerance {JACOBIAN_TOL}"
    )


def run_demo(controller: VelocityController,
             custom_robot: SmallRbtArm,
             motion: MotionPatterns) -> None:
    """Execute the full motion sequence.

    Separated from hardware setup so exceptions propagate cleanly to the
    try/finally in main(), ensuring the arm is always safe-stopped.
    """
    # --- move to circle start position ---
    T_06 = custom_robot.convert_p_dc_to_T06(CIRCLE_START_POSE)
    j = custom_robot.ik(T_06)
    controller.move_to_angles(j)
    sleep(POSITION_SETTLE_S)

    # --- approach: move +X by CIRCLE_RADIUS_MM to reach circle perimeter ---
    x_dot_approach = np.array([APPROACH_VEL_X_MM_S, 0.0, 0.0, 0.0, 0.0, 0.0])
    controller.cartesian_space_vel_ctrl(
        x_dot_func=lambda t: x_dot_approach,
        duration=APPROACH_DURATION_S,
    )

    input("Press Enter to start circle motion…")
    controller.cartesian_space_vel_ctrl(
        x_dot_func=lambda t: motion.fourier_circle(
            t, radius=CIRCLE_RADIUS_MM, period=CIRCLE_PERIOD_S
        ),
        duration=CIRCLE_PERIOD_S,
    )

    controller.go_init()

    input("Press Enter to start zigzag pattern…")
    controller.cartesian_space_vel_ctrl(
        x_dot_func=lambda t: motion.zigzag(
            t, side_length=ZIGZAG_SIDE_MM, period=ZIGZAG_PERIOD_S
        ),
        duration=ZIGZAG_PERIOD_S,
    )

    input("Press Enter to go home…")
    controller.go_home()

def main() -> int:
    """Entry point — returns 0 on success, 1 on error."""
    # --- initialise robot models ---
    toolbox_robot = DHRobot(std_dh_tbl, name="smallRobotArm")
    toolbox_robot.ets()

    custom_robot = SmallRbtArm(std_dh_params)
    controller   = VelocityController(toolbox_robot)
    motion       = MotionPatterns()

    # --- verify Jacobian consistency before touching hardware ---
    q_test = np.radians(controller.robot_rest_angles)
    try:
        check_jacobians(custom_robot, toolbox_robot, q_test)
    except AssertionError as e:
        print(f"[ERROR] {e}")
        return 1

    # --- hardware setup with guaranteed safe-stop ---
    setup_signal_handler(controller)
    sleep(SIGNAL_SETTLE_S)

    controller.enable()
    sleep(ENABLE_SETTLE_S)

    try:
        run_demo(controller, custom_robot, motion)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    except Exception as e:
        print(f"[ERROR] Unexpected error during demo: {e}")
        return 1
    finally:
        # Always return the arm home and disable, regardless of how we exit
        print("[INFO] Safe-stopping: returning home and disabling controller.")
        controller.go_home()

    return 0


if __name__ == "__main__":
    sys.exit(main())