# CLAUDE.md — smallRobotArm

Project context for Claude Code. Read this before making changes.

## What this is

A 6-DOF small robot arm, stepper-motor actuated, controlled from Python
(`robot_tools` package). Hardware talks over serial to Arduino firmware.
Repo: `roachhuang/smallRobotArm` (public GitHub).

The project is deliberately built as a hands-on companion to *Modern
Robotics* (Lynch & Park) and the associated Northwestern Coursera course.
The textbook is the spec. When in doubt about convention, math, or
terminology, the book wins — not Wikipedia, not `roboticstoolbox` defaults,
not "how most industry code happens to do it."

Goal: not a toy demo. Treat this as an industrial-grade codebase — the kind
you'd hand to another engineer to maintain, not a notebook you'd hand to a
grader. See "Engineering standards" below for what that means concretely.

## Non-negotiable core principle

**Use the `modern_robotics` (MR) Python library for anything it already
implements. Do not reimplement library-covered math.**

If a task in this repo maps to something in the MR library
(`FKinSpace`, `FKinBody`, `JacobianSpace`, `JacobianBody`, `IKinBody`,
`IKinSpace`, `Adjoint`, `TransInv`, screw-axis utilities, trajectory
generation helpers, etc.), call it. Hand-derived math is a maintenance
liability and a correctness risk — we already caught one real bug this way
(`Slist_body` had most columns wrong; the fix was to stop hand-deriving and
use MR Eq. 5.20: `Blist = mr.Adjoint(mr.TransInv(M)) @ Slist`).

`roboticstoolbox-python` is used only at specific boundaries (see
"Convention boundaries" below) — not as a second implementation of core
kinematics.

## Convention: MR `[ω; v]` everywhere

All twists, Jacobians, and spatial vectors in this codebase use the Modern
Robotics `[ω; v]` (angular-first) block convention. This was a hard-switch
migration — there is no legacy `[v; ω]` code left in first-party modules.
`jacobians.py` is the canonical module for any conversion at a boundary.

### Convention boundaries (`jacobians.py`)

`roboticstoolbox`'s Jacobian requires **two independent corrections**, not
one, to become MR-compatible:

1. **Block order swap**: `[v; ω]` → `[ω; v]`
2. **Spatial-to-geometric re-referencing**

Both are required. A fix that only does one of these is wrong even if it
looks plausible. Route all `roboticstoolbox` boundary conversions through
the named converters in `jacobians.py` — don't inline ad hoc swaps
elsewhere.

## Architecture pattern: standalone module + thin wrapper

New functionality is implemented as a **standalone, robot-agnostic module**
that wraps `modern_robotics` primitives, with a **thin delegating method**
added to `SmallRbtArm` for integration. Do not grow `SmallRbtArm` into a
monolith.

Established examples of this pattern:
- `jacobians.py` — Jacobian convention conversions
- `time_optimal_scaling.py` — Ch. 9 phase-plane time-scaling
- `numerical_ik.py` — Ch. 6 numerical IK

Follow this pattern for any new chapter-driven capability.

## Naming and semantics

- `S` = space frame, `B` = body frame, per MR convention. Any attribute
  named `Slist_body` (or similar self-contradictory name) is a bug in
  waiting — flag it, don't propagate it. (This is a known open item —
  see "Open items" below.)
- Prefer `IKinBody` over `IKinSpace` when tolerance semantics matter:
  `IKinBody` tolerances are interpreted in the end-effector frame, which
  directly bounds tool-tip position/orientation error. `IKinSpace`'s
  linear tolerance is measured at the space-frame origin — not physically
  meaningful for tool-tip accuracy. `numerical_ik.py` accepts space-frame
  inputs at its API boundary but uses `IKinBody` internally for this
  reason; keep that pattern.

## Stepper-motor reality check

This arm is stepper-driven → naturally suited to **position/waypoint-based
control**, not torque-level dynamics loops. Concretely:
- Velocity control targets are joint-rate commands, not torques.
- Ch. 8 (dynamics) is useful for gravity-torque analysis and payload
  sizing, not for a real-time control loop.
- Torque-level control is technically possible with current-controlled
  drivers but is out of scope unless explicitly requested — don't propose
  it as a default solution.

## Testing philosophy

No implementation is done until it's validated against ground truth. Use
whichever applies:
- Closed-form solutions (e.g., trapezoidal profile vs. time-optimal scaling)
- Book examples (e.g., MR Example 6.1 reproduction)
- FK→IK round trips on a known arm (e.g., UR5-like 6R)
- Adversarial cases, not just happy path (e.g., seed/convergence failure
  demonstrations for numerical IK)

**Precision matters for reproducibility**: Newton–Raphson basin boundaries
are sharp. Rounding a failing seed to 3 decimal places can move it into a
convergent basin. Test seeds must use full `repr`-precision values, not
rounded literals.

## Engineering standards ("industrial-grade" means, concretely)

- Type hints on public functions/methods; docstrings that cite the relevant
  MR chapter/equation number where the math comes from a book result
  (reinforces curriculum alignment and gives a citation trail for review).
- No bare `except:`. Serial/hardware I/O failures should raise or return
  explicit, typed errors — never fail silently into a wrong motor command.
- Input validation at module boundaries (e.g., joint angle arrays of wrong
  shape, NaN twists) should fail loudly before reaching hardware-facing
  code.
- No new hand-derived screw axes, Jacobians, or transforms without a
  validation test against a closed-form or book-example ground truth.
- Logging (not print) for anything in the control/serial path.
- Unit tests live alongside the standalone module they test; regression
  tests must pass before merging chapter-migration work.

## Current status (update as milestones land)

Completed and merged:
- **Ch. 5 Jacobian standardization** — full `[ω; v]` migration, canonical
  `jacobians.py`, `Slist_body` bug fixed via MR Eq. 5.20.
- **Ch. 9 time-optimal scaling** — `time_optimal_scaling.py`, phase-plane
  pass in `w = ṡ²`, validated against trapezoidal ground truth, streams via
  `q_dot_func()` closure.
- **Ch. 6 numerical IK** — `numerical_ik.py` (`solve_ik`,
  `solve_ik_with_restarts`, `ik_pose_error`), `IKinBody`-internal design,
  tested against MR Example 6.1 + adversarial seeds + FK→IK round trips.

Known gap (flagged, not yet fixed):
- **No singularity handling in the velocity controller.** The
  Jacobian-based velocity control path (`jacobian_vel_ctrl.py` /
  `VelocityController`) currently computes `q_dot = np.linalg.pinv(Js) @
  spatial_twist` with no condition-number check, no manipulability measure
  (MR §5.4), and no damped-least-squares fallback. Near a singularity this
  can request an unbounded step rate. This is a real correctness/safety
  gap for stepper hardware, not just a nice-to-have.

## Open items

- `Slist_body` attribute name in `SmallRbtArm` is self-contradictory
  (S = space-frame by MR convention) — rename and audit call sites.
- Finalize `numerical_ik.py` integration into `SmallRbtArm` via a thin
  delegating method, consistent with the `jacobians.py` /
  `time_optimal_scaling.py` pattern.
- `CartesianStraightLineTrajectory`
  (`robot_tools/trajectory/cartesian_straight_line.py`) was designed but
  left mid-validation — status needs to be re-checked before relying on it.
- Velocity-controller singularity handling (see "Known gap" above) —
  candidates: damped least squares inverse, explicit rank/condition-number
  check, manipulability-ellipsoid diagnostics (MR §5.4).
- Ch. 10/12 — flagged as situationally useful, no decision made on scope.

## Working style

- Ask clarifying questions (ideally structured/multiple-choice) before
  diving into a detailed implementation, especially when a task could map
  to more than one reasonable design.
- Assume advanced Python and robotics fluency — no introductory
  explanations of standard concepts needed.
- When explaining a *new* concept (not implementing one), analogies and
  simple worked examples are preferred over abstract description.