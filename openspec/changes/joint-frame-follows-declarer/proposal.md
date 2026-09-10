# Restate this leg's two joints in the frame of the body that declares them

## Why

solid-node's framework cycle `joint-frame-follows-declarer` (ADR-097,
integrated into solid-node main 91c0b2a) changed what a class-body joint's
`axis`, `at` and `carries` mean: they are now read in the declaring body's
OWN rest frame, not the parent's, and `at` defaults to that body's own
origin. This project's prior stage B (`move-onto-motion`,
`openspec/changes/archive/2026-09-10-move-onto-motion/`) declared
`Femur.lift` and `Tibia.knee` with an `at=` that was written, correctly at
the time, against the OLD (parent-frame) rule:

    lift = Revolute(axis=(0, -1, 0),
                    at=(COXA_LENGTH, 0.0, joint.FORK_MID), unit='deg')
    knee = Revolute(axis=(0, -1, 0), at=(FEMUR_LENGTH, 0.0, 0.0), unit='deg')

Both `at=` values are the *identical expression* the immediate parent's
`render()` already translates the declaring body by
(`Coxa.render()`: `self.femur.translate([COXA_LENGTH, 0.0,
joint.FORK_MID])`; `Femur.render()`: `self.tibia.translate([FEMUR_LENGTH,
0.0, 0.0])`). Under the new own-frame rule that offset is applied a SECOND
time, on top of the body's own placement, so as of solid-node main 91c0b2a
this project is broken: measured directly, before this change,

    Ran 34 tests in 7.64 seconds: 24 passed, 10 failed

with failures that are exactly a double-counted leg offset (a leg's link
lengths measured wrong, its foot not where the solved angle puts it, feet
that never touch the ground: `LegKinematicsTest.test_the_links_are_the_measured_ones`,
`LegKinematicsTest.test_the_foot_is_where_the_solution_says`,
`JointStackTest.test_a_joint_moves_only_what_is_beyond_it`,
`WholeRobotTest.test_assembly_integrity`, four `PostureTest` scenarios and
two `PostureScenarioTest` scenarios — the full list is in `tasks.md`).

This change is the mechanical fix the rule calls for: delete both `at=`
arguments so they default to `(0, 0, 0)`, which is exactly where each
parent already places the body. It is cited in the framework change's own
evidence survey: `solid-node/openspec/changes/archive/2026-09-10-joint-frame-follows-declarer/evidence/survey.md`
§1.1, row `Robots/hexapod_spiderbot_model/simulation/leg.py:213,143`.

## What changes

- `Femur.lift` (`simulation/leg.py`): delete `at=(COXA_LENGTH, 0.0,
  joint.FORK_MID)`. The femur's own origin already IS the lift axis —
  `Coxa.render()` places it there — so the default `(0, 0, 0)` states the
  same anchor with nothing restated.
- `Tibia.knee` (`simulation/leg.py`): delete `at=(FEMUR_LENGTH, 0.0, 0.0)`.
  Same reasoning: `Femur.render()` places the tibia exactly on its own knee
  axis.
- Neither axis is rewritten. Both immediate parent placements
  (`Coxa.render()`'s and `Femur.render()`'s translate calls) are pure
  translations with an identity rest rotation, so `axis=(0, -1, 0)` is
  numerically the same vector in the parent's frame and in the declaring
  body's own frame either way.
- `Coxa.yaw` (identity parent placement — `Leg.render()` places nothing but
  the yaw servo) and `Chassis.pose` (`Free`, identity parent placement —
  `Spiderbot` has no `render()` at all) already carry no `at=`/`carries=`
  and need no change: both were already OWN-ORIGIN-ALREADY under the old
  rule and stay that way under the new one.
- The module docstring and the two per-joint comments in `leg.py` that
  narrated the OLD "the anchor restates the parent's translate" convention
  are rewritten: they described an `at=` argument that no longer exists
  and would otherwise mislead a future reader about why the axis and
  anchor are what they are.
- No hand-written `rotate`/`translate` in this project is a candidate for
  becoming a declared joint under this rule (survey §1.2, "a joint that
  cannot be declared at all today"): none was found anywhere in
  `simulation/`, confirmed by two independent greps
  (`= Revolute(`/`= Prismatic(`/`= Orbit(`/`= Free(` and a broader pattern
  covering multi-line calls and bare mentions).

Nothing else changes. No port, no `simulate()`, no test, and no
`openspec/specs/` requirement is touched: the geometric requirements in
`leg-kinematics/spec.md` (link lengths, axis parallelism, the computed
tibia reach, the two-link solution) describe where the axes stand in
space, and this change moves nothing in space — it only moves where the
same two numbers are read from. The pose evidence below proves that.

## Impact

- Affected code: `simulation/leg.py` only (two joint declarations, three
  comments, the module docstring).
- Affected specs: none. No requirement's stated numbers or geometry
  change.
- Affected capability: none new; this restates an existing declaration,
  it does not add or remove an interface.
