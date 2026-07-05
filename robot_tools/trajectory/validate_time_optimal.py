"""Validation for time_optimal_scaling.py"""
import numpy as np
import modern_robotics as mr
from time_optimal_scaling import (
    KinematicLimits, PathFunction, TimeOptimalTimeScaling, joint_straight_line_path,
)

np.set_printoptions(precision=4, suppress=True)

# ---------------------------------------------------------------- Test 1
# Straight-line joint path -> optimal scaling must equal the analytic
# trapezoid (MR Sec. 9.2.2.2) driven by the binding joint.
print("=" * 64)
print("Test 1: straight-line path vs analytic trapezoid")
q0 = np.zeros(6)
q1 = np.array([1.2, -0.8, 0.5, 2.0, -1.5, 0.3])
limits = KinematicLimits(
    qd_max=np.array([1.0, 1.0, 1.5, 2.0, 2.0, 3.0]),
    qdd_max=np.array([2.0, 2.0, 3.0, 4.0, 4.0, 6.0]),
)
path = joint_straight_line_path(q0, q1)
planner = TimeOptimalTimeScaling(path, limits, n_grid=4000)
prof = planner.plan()

# Analytic: per joint, path-velocity limit v_i = qd_i/|dq_i|, a_i = qdd_i/|dq_i|
dq = np.abs(q1 - q0)
v = np.min(limits.qd_max / dq)   # max s_dot
a = np.min(limits.qdd_max / dq)  # max s_ddot
if v**2 / a >= 1.0:  # triangular
    T_analytic = 2.0 * np.sqrt(1.0 / a)
else:                # trapezoid, Eq. 9.24 rearranged for s in [0,1]
    T_analytic = (a + v**2) / (v * a)
print(f"  planner T = {prof.total_time:.6f}  analytic T = {T_analytic:.6f}"
      f"  rel.err = {abs(prof.total_time-T_analytic)/T_analytic:.2e}")
print(f"  switches at s = {prof.switches}")
assert abs(prof.total_time - T_analytic) / T_analytic < 1e-3

# ---------------------------------------------------------------- Test 2
# Curved path (nonzero theta''): feasibility + bang-bang optimality.
print("=" * 64)
print("Test 2: curved path feasibility")
A = np.array([0.8, -0.6, 0.4, 1.0, -0.7, 0.5])
B = np.array([1.0, 2.0, 1.0, 3.0, 2.0, 1.0]) * np.pi

def theta(s):  return A * np.sin(B * s)
def dtheta(s): return A * B * np.cos(B * s)
def ddtheta(s): return -A * B**2 * np.sin(B * s)

curved = PathFunction(theta, dtheta, ddtheta)
planner2 = TimeOptimalTimeScaling(curved, limits, n_grid=4000)
prof2 = planner2.plan()
rep = planner2.feasibility_report(dt=0.002)
print(f"  T = {rep['total_time']:.4f}s  qd util = {rep['qd_util_max']:.4f}"
      f"  qdd util = {rep['qdd_util_max']:.4f}  switches = {rep['n_switches']}")
assert rep["qd_util_max"] <= 1.005 and rep["qdd_util_max"] <= 1.02

# Optimality proxy: at (nearly) every s some constraint is active.
t, q, qd, qdd = planner2.sample(0.002)
util = np.maximum(np.max(np.abs(qd)/limits.qd_max, axis=1),
                  np.max(np.abs(qdd)/limits.qdd_max, axis=1))
print(f"  fraction of samples with active constraint (util>0.98): "
      f"{np.mean(util > 0.98):.3f}")

# Same path with finite-difference derivatives only (no analytic dtheta).
planner2b = TimeOptimalTimeScaling(PathFunction(theta), limits, n_grid=4000)
prof2b = planner2b.plan()
print(f"  FD-derivative T = {prof2b.total_time:.4f}s "
      f"(analytic {prof2.total_time:.4f}s)")

# ---------------------------------------------------------------- Test 3
# Speedup vs mr quintic scaling meeting the SAME limits.
print("=" * 64)
print("Test 3: vs mr.JointTrajectory quintic at same limits")
# Quintic on straight line: peak qd = 1.875*|dq|/T, peak qdd = 5.7735*|dq|/T^2
dq = np.abs(q1 - q0)
T_qd = np.max(1.875 * dq / limits.qd_max)
T_qdd = np.max(np.sqrt(10.0 / np.sqrt(3.0) * dq / limits.qdd_max))
T_quintic = max(T_qd, T_qdd)
traj = mr.JointTrajectory(q0, q1, T_quintic, 100, 5)  # sanity: runs fine
print(f"  quintic T = {T_quintic:.4f}s  time-optimal T = {prof.total_time:.4f}s"
      f"  speedup = {T_quintic/prof.total_time:.2f}x")

# ---------------------------------------------------------------- Test 4
# q_dot_func closure: integrate it and confirm we land on q1.
print("=" * 64)
print("Test 4: q_dot_func integration reaches goal")
qd_fn = planner.q_dot_func()
dt = 1e-3
q = q0.copy()
tt = 0.0
while tt < prof.total_time:
    q = q + qd_fn(tt) * dt
    tt += dt
err = np.linalg.norm(q - q1)
print(f"  ||q(T) - q1|| = {err:.5f} rad (Euler, dt={dt})")
assert err < 5e-3

print("=" * 64)
print("ALL TESTS PASSED")
