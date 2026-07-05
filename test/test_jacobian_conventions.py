"""Convention regression tests for robot_tools.kinematics.jacobians.

These tests are the durable guard against [omega;v]/[v;omega] and
spatial/geometric mixing. Reference robot: Puma560-like 6R defined by
DH parameters, built identically in roboticstoolbox (DHRobot) and in
MR form (Slist, M) derived from the same DH table at q = 0.
"""
import numpy as np
import modern_robotics as mr
import roboticstoolbox as rtb
from spatialmath import SE3

import robot_tools.kinematics.jacobians as jk

rng = np.random.default_rng(42)


# --------------------------------------------------------------- fixture
# Build the same 6R robot in both libraries from one DH table.
DH = [  # (d, a, alpha)  standard DH, all revolute
    (0.672, 0.0,   -np.pi / 2),
    (0.0,   0.432,  0.0),
    (0.15,  0.02,  -np.pi / 2),
    (0.432, 0.0,    np.pi / 2),
    (0.0,   0.0,   -np.pi / 2),
    (0.1,   0.0,    0.0),
]
robot = rtb.DHRobot([rtb.RevoluteDH(d=d, a=a, alpha=al) for d, a, al in DH])


def _mr_model_from_dh():
    """Slist and M from the DH chain at the zero configuration.

    Joint i's screw: rotation about its z-axis through its origin,
    S_i = [omega_i; -omega_i x q_i]  (MR Eq. 3.86, [omega; v] order).
    """
    T = np.eye(4)
    Slist = []
    for d, a, alpha in DH:
        w = T[0:3, 2]           # joint axis = current z, in {s}
        p = T[0:3, 3]           # point on the axis
        Slist.append(np.concatenate([w, -np.cross(w, p)]))
        A = SE3.Rz(0) * SE3.Tz(d) * SE3.Tx(a) * SE3.Rx(alpha)
        T = T @ A.A
    return np.array(Slist).T, T  # (6, n), M = T_sb(0)


Slist, M = _mr_model_from_dh()


def _rand_q():
    return rng.uniform(-np.pi, np.pi, 6)


# ----------------------------------------------------------------- tests

def test_mr_model_matches_rtb_fk():
    """Sanity: the two models are the same robot (FK agrees)."""
    for _ in range(20):
        q = _rand_q()
        T_mr = mr.FKinSpace(M, Slist, q)
        T_rtb = robot.fkine(q).A
        assert np.allclose(T_mr, T_rtb, atol=1e-10)


def test_adjoint_identity_body_vs_space():
    """MR Eq. 5.22: J_b = [Ad_{T_bs}] J_s. Internal consistency."""
    Blist = np.array([mr.Adjoint(mr.TransInv(M)) @ Slist[:, i]
                      for i in range(6)]).T  # MR Eq. 5.20
    for _ in range(20):
        q = _rand_q()
        J_s = jk.jacobian_space(Slist, q)
        J_b = jk.jacobian_body(Blist, q)
        J_b_via_s = jk.body_jacobian_from_space(J_s, Slist, M, q)
        assert np.allclose(J_b, J_b_via_s, atol=1e-10)


def test_rtb_cross_validation():
    """rtb jacob0 == mr space Jacobian after BOTH corrections:
    block swap ([v;omega] -> [omega;v]) AND re-referencing the linear
    rows from the {s} origin to the EE point. Either correction alone
    must FAIL - that failure is the convention bug this guards.
    """
    for _ in range(20):
        q = _rand_q()
        J_s = jk.jacobian_space(Slist, q)
        p_ee = mr.FKinSpace(M, Slist, q)[0:3, 3]
        J_rtb = robot.jacob0(q)

        # full conversion agrees both directions
        assert np.allclose(jk.mr_to_rtb_jacobian(J_s, p_ee), J_rtb, atol=1e-9)
        assert np.allclose(jk.rtb_to_mr_jacobian(J_rtb, p_ee), J_s, atol=1e-9)

        # block swap alone is NOT enough (spatial != geometric)
        assert not np.allclose(jk.swap_jacobian_blocks(J_s), J_rtb, atol=1e-6)


def test_converters_are_involutions():
    V = rng.standard_normal(6)
    J = rng.standard_normal((6, 6))
    p = rng.standard_normal(3)
    assert np.allclose(jk.swap_twist_blocks(jk.swap_twist_blocks(V)), V)
    assert np.allclose(jk.swap_jacobian_blocks(jk.swap_jacobian_blocks(J)), J)
    assert np.allclose(jk.geometric_to_spatial(jk.spatial_to_geometric(J, p), p), J)


def test_statics_virtual_work():
    """tau = J_b^T F_b (MR Eq. 5.26) checked by virtual work:
    tau^T dq == F_b^T V_b dt for the twist induced by dq."""
    Blist = np.array([mr.Adjoint(mr.TransInv(M)) @ Slist[:, i]
                      for i in range(6)]).T
    for _ in range(20):
        q = _rand_q()
        F_b = rng.standard_normal(6)          # [m; f] in {b}
        J_b = jk.jacobian_body(Blist, q)
        tau = jk.joint_torques_from_wrench(J_b, F_b)
        dq = rng.standard_normal(6) * 1e-6
        work_joint = tau @ dq
        work_task = F_b @ (J_b @ dq)
        assert np.isclose(work_joint, work_task, rtol=1e-9)


def test_sigma_min_detects_singularity():
    """Wrist singularity (q5 = 0 aligns joints 4 and 6 for this DH)."""
    q_sing = np.array([0.3, -0.5, 0.4, 0.7, 0.0, 0.2])
    q_ok = q_sing + np.array([0, 0, 0, 0, 0.6, 0])
    assert jk.sigma_min(jk.jacobian_space(Slist, q_sing)) < 1e-8
    assert jk.sigma_min(jk.jacobian_space(Slist, q_ok)) > 1e-3


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
