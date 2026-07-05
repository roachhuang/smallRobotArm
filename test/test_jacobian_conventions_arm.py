"""Convention regression tests for the smallRobotArm migration (MR Ch. 5).

Canonical convention: twists/Jacobian rows stacked [omega; v] (MR).
These tests pin down BOTH foreign-convention corrections at the
roboticstoolbox boundary — block order AND reference point — using the
robot's real models (PoE Slist/Blist/M vs. the std_dh_tbl DHRobot).
"""
import numpy as np
import modern_robotics as mr
import roboticstoolbox as rtb

from robot_tools.kinematics import SmallRbtArm, std_dh_params, std_dh_tbl
from robot_tools.kinematics import jacobians as jk

rng = np.random.default_rng(7)

arm = SmallRbtArm(std_dh_params)
toolbox = rtb.DHRobot(std_dh_tbl, name="smallRobotArm")


def _rand_q():
    """Random q within (radian) joint limits."""
    lo = np.radians(arm.min_qlimits)
    hi = np.radians(arm.max_qlimits)
    return rng.uniform(lo, hi)


def test_poe_and_dh_models_agree():
    """Sanity: PoE FK == toolbox DH FK (same robot, same q)."""
    assert np.allclose(arm.poe_fk(np.zeros(6)), toolbox.fkine(np.zeros(6)).A,
                       atol=1e-9), "zero-config mismatch: M vs fkine(0)"
    for _ in range(20):
        q = _rand_q()
        assert np.allclose(arm.poe_fk(q), toolbox.fkine(q).A, atol=1e-8)


def test_rtb_cross_validation_needs_both_corrections():
    """jacobian_geometric == rtb jacob0 after block swap; and the swap
    ALONE (without EE re-referencing) must NOT reproduce the space
    Jacobian — that failure is the silent bug this suite guards."""
    for _ in range(20):
        q = _rand_q()
        J_geo = arm.jacobian_geometric(q)
        J_rtb_mr = jk.jacobian_geometric_from_rtb(toolbox, q)
        assert np.allclose(J_geo, J_rtb_mr, atol=1e-8)

        # full spatial round trip
        p_ee = arm.poe_fk(q)[0:3, 3]
        assert np.allclose(jk.rtb_to_mr_jacobian(toolbox.jacob0(q), p_ee),
                           arm.jacobian_space(q), atol=1e-8)

        # block swap alone is NOT the space Jacobian
        assert not np.allclose(jk.swap_jacobian_blocks(toolbox.jacob0(q)),
                               arm.jacobian_space(q), atol=1e-4)


def test_adjoint_identity_body_vs_space():
    """MR Eq. 5.22: J_b = [Ad_{T_bs}] J_s ties Slist, Blist and M together.
    Catches errors in ANY of the hand-derived screw axis tables."""
    for _ in range(20):
        q = _rand_q()
        J_b = arm.jacobian_body(q)
        J_b_via_s = jk.body_jacobian_from_space(
            arm.jacobian_space(q), arm.Slist, arm.M, q)
        assert np.allclose(J_b, J_b_via_s, atol=1e-8)


def test_blist_consistent_with_slist():
    """Blist == [Ad_{M^-1}] Slist (MR Eq. 5.20), column by column."""
    AdMinv = mr.Adjoint(mr.TransInv(arm.M))
    assert np.allclose(arm.Blist, AdMinv @ arm.Slist, atol=1e-9)


def test_statics_virtual_work():
    """tau = J^T F (MR Eq. 5.26) via virtual work with the geometric
    Jacobian and an EE-point wrench [m; f] — the compute_jacobian
    contract used by the force controllers."""
    for _ in range(20):
        q = _rand_q()
        F_ee = rng.standard_normal(6)  # [m; f] at EE, world frame
        J = arm.compute_jacobian(q)
        tau = jk.joint_torques_from_wrench(J, F_ee)
        dq = rng.standard_normal(6) * 1e-7
        assert np.isclose(tau @ dq, F_ee @ (J @ dq), rtol=1e-9)


def test_twist_converters_are_involutions():
    V = rng.standard_normal(6)
    J = rng.standard_normal((6, 6))
    p = rng.standard_normal(3)
    assert np.allclose(jk.swap_twist_blocks(jk.swap_twist_blocks(V)), V)
    assert np.allclose(jk.swap_jacobian_blocks(jk.swap_jacobian_blocks(J)), J)
    assert np.allclose(
        jk.geometric_to_spatial(jk.spatial_to_geometric(J, p), p), J)


def test_motion_patterns_are_mr_order():
    """Cartesian patterns: angular rows (0:3) zero, motion in linear rows."""
    from robot_tools.trajectory.motion_patterns import MotionPatterns
    m = MotionPatterns()
    for t in np.linspace(0.1, 9.9, 23):
        for v in (m.fourier_circle(t), m.figure8(t), m.spiral(t),
                  m.zigzag(t), m.square_xz(t, 40, 10)):
            assert np.allclose(v[0:3], 0.0), "angular rows must be zero"
        assert np.any(np.abs(m.fourier_circle(t)[3:6]) > 0)


def test_sigma_min_detects_singularity():
    """q5 = 0 (physical zero of joint 5) aligns the wrist axes 4/6."""
    q_sing = np.radians([20.0, -30.0, 25.0, 15.0, 0.0, 40.0])
    q_ok = q_sing + np.array([0, 0, 0, 0, 0.7, 0])
    s_sing = jk.sigma_min(arm.jacobian_space(q_sing))
    s_ok = jk.sigma_min(arm.jacobian_space(q_ok))
    assert s_sing < 1e-6 * s_ok, f"expected singular: {s_sing} vs {s_ok}"


if __name__ == "__main__":
    import sys, traceback
    fails = 0
    for name, fn in sorted({k: v for k, v in globals().items()
                            if k.startswith("test_")}.items()):
        try:
            fn()
            print(f"PASS  {name}")
        except Exception:
            fails += 1
            print(f"FAIL  {name}")
            traceback.print_exc()
    sys.exit(1 if fails else 0)
