Run everything from the project root with `PYTHONPATH=.` and the workspace
venv (`/home/asa/devel/libresolid-studio/.venv/bin/python`,
`.venv/bin/solid`). `.env` selects the faceted kernel. Never run two heavy
processes at once. No test file is edited by this change.

## 0. The working state

- [x] 0.1 `git rev-parse --show-toplevel` names this project
      (`/home/asa/devel/libresolid-studio/projects/Robots/hexapod_spiderbot_model`)
      and `git status` was clean before any edit: nothing modified,
      nothing staged, no untracked tracked-looking file.

## 1. Evidence — the broken baseline

- [x] 1.1 Confirmed the framework rule (ADR-097, solid-node main 91c0b2a)
      is already active for this project and, unfixed, breaks it.
      `solid test` on the unfixed source:

          Ran 34 tests in 7.64 seconds: 24 passed, 10 failed
          (faceted kernel, volume epsilon 0 mm³)

      The 10 failures, all one cause (the doubled `Femur.lift`/`Tibia.knee`
      offset), none a pre-existing condition:
      `JointStackTest.test_a_joint_moves_only_what_is_beyond_it`,
      `LegKinematicsTest.test_the_foot_is_where_the_solution_says`,
      `LegKinematicsTest.test_the_links_are_the_measured_ones`,
      `PostureTest.test_attitude_moves_the_body_and_not_the_feet`,
      `PostureTest.test_crouching_lowers_the_body_and_keeps_the_feet_down`,
      `PostureTest.test_standing_is_level_and_planted`,
      `PostureTest.test_the_tripod_gait_always_has_three_feet_down`,
      `WholeRobotTest.test_assembly_integrity`,
      `PostureScenarioTest.test_the_feet_stay_planted_through_a_look_around`,
      `PostureScenarioTest.test_walking_keeps_a_tripod_down`.
      This confirms the proposal's "Why": the project is genuinely broken
      on main, not merely stylistically out of date.

## 2. The fix — `simulation/leg.py`

- [x] 2.1 `Tibia.knee`: deleted `at=(FEMUR_LENGTH, 0.0, 0.0)` — the
      identical expression `Femur.render()` already translates this body
      by. New declaration: `knee = Revolute(axis=(0, -1, 0), unit='deg')`.
      Axis unchanged: the parent placement is a pure translation (identity
      rest rotation), so `(0, -1, 0)` is the same vector in both frames.
- [x] 2.2 `Femur.lift`: deleted `at=(COXA_LENGTH, 0.0, joint.FORK_MID)` —
      the identical expression `Coxa.render()` already translates this
      body by. New declaration:
      `lift = Revolute(axis=(0, -1, 0), unit='deg')`. Axis unchanged, same
      reasoning.
- [x] 2.3 `Coxa.yaw` and `Chassis.pose` (`Free`, in `simulation/spiderbot.py`):
      read both bodies' immediate parents in full
      (`Leg.render()` places only `self.yaw_servo`, never `self.coxa`;
      `Spiderbot` declares no `render()` at all) and confirmed both are
      already placed at the identity transform. No change to either.
- [x] 2.4 Rewrote the module docstring (lines 6-14) and the `knee` and
      `lift` per-joint comments: they narrated the OLD "the anchor
      restates the parent's translate, read in the parent's frame"
      convention, which after this change describes an argument
      (`at=`) that is no longer written. The new prose says the anchor
      is read in the joint's OWN declaring body's frame and states
      why both joints need none (each parent already places the body
      exactly on the axis it turns about).
- [x] 2.5 Searched the whole `simulation/` package
      (`grep -rn "= Revolute(\|= Prismatic(\|= Orbit(\|= Free(" simulation/`
      and a broader `Revolute\|Prismatic\|Orbit\|Free(\|motion.joints`
      pattern) for any hand-written `rotate`/`translate` that exists only
      because one class is placed at several points (survey §1.2,
      "a joint that cannot be declared at all today"). None found: the
      four joint sites in this project (`Coxa.yaw`, `Femur.lift`,
      `Tibia.knee`, `Chassis.pose`) are the only motion declarations in
      the package, and no other placement call is a declaration-site
      candidate. This is itself evidence for the framework change: this
      project has nothing the frame rule leaves unable to be stated.

## 3. Evidence again

- [x] 3.1 Reused the BEFORE poses captured by the framework cycle on the
      pre-change framework from the untouched source (21 poses, 172
      leaves).
- [x] 3.2 Captured AFTER on solid-node main with this fix applied:

          PYTHONPATH=. python capture_poses.py capture simulation.spiderbot:Spiderbot <after.json>
          captured 21 poses, 172 leaves

      and compared:

          max deviation 0.000e+00 over 21 poses

      Bit-identical: nothing in space moved. Confirms the proposal's claim
      that this change only moves where the same two numbers are read
      from, never what they evaluate to.
- [x] 3.3 `solid test` with the fix applied:

          Ran 34 tests in 8.70 seconds: 34 passed, 0 failed
          (faceted kernel, volume epsilon 0 mm³)

      All 10 failures from step 1.1 are gone; nothing newly red.
      Re-verified a second time after a deliberate revert/reapply round
      trip used to capture the exact broken-baseline failure list — same
      34/34 both times.
- [x] 3.4 No requirement in `openspec/specs/` needed a delta: the
      geometric claims in `leg-kinematics/spec.md` (link lengths, axis
      parallelism, computed tibia reach, six feet on one plane) describe
      where the axes and the foot stand in space, and the 0.000e+00 pose
      comparison proves none of that moved. **Tooling note:** `openspec
      validate joint-frame-follows-declarer` refuses any change with zero
      deltas ("Change must have at least one delta"), even a `--strict`
      pass; there is no capability or requirement this change adds,
      modifies or removes, so no `specs/` directory is written rather
      than fabricating a delta with nothing behavioural to say. This
      change directory therefore does not pass `openspec validate` by
      tool design, the same as the prior stage-B change in this project
      (`archive/2026-09-10-move-onto-motion/`, also proposal.md +
      tasks.md only, no `specs/`).
- [x] 3.5 No deviation from this proposal: both `at=` deletions, the axis
      invariance, and the OWN-ORIGIN-ALREADY classification of `Coxa.yaw`
      and `Chassis.pose` all landed exactly as the framework change's own
      survey (§1.1, §1.2) and this project's evidence predicted.
- [x] 3.6 Committed as
      `refactor(simulation): restate leg joints in the declaring body's own frame (ADR-097)`
      with the pose comparison and the test counts in the body.

## 4. Report

- [x] 4.1 Reported: the exact sentences changed, the broken-baseline test
      list, the pose comparison (0.000e+00 over 21 poses), the suite
      counts before (24/34, framework-broken) and after (34/34), the
      commit, and that nothing the frame rule could not state was found
      in this project. Not synced and not archived: the orchestrator
      reviews first.
