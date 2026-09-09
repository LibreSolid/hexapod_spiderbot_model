Behaviour-preserving throughout: the acceptance is that every leaf's world
matrix is unchanged at every pose, so evidence is captured before the first
edit and compared after the last. Run everything from the project root with
`PYTHONPATH=.` and the workspace venv
(`/home/asa/devel/libresolid-studio/.venv/bin/python`,
`.venv/bin/solid`). `.env` selects the faceted kernel; leave it untracked.
Never run two suites at once. Never edit a test — if one blocks you, stop,
name the assertion and the reason, and wait.

## 0. The working state

- [x] 0.1 `git rev-parse --show-toplevel` names this project
      (`/home/asa/devel/libresolid-studio/projects/Robots/hexapod_spiderbot_model`),
      and `git status` was clean: nothing modified or staged, the only
      untracked paths the ignored `.env`/`_build/`/`_build.lock`/
      `screenshot.png` and the not-yet-committed
      `openspec/changes/move-onto-motion/` proposal directory itself.
      Confirms the proposal's "Pre-existing state": clean, nothing to
      commit.

## 1. Evidence — the baseline

- [x] 1.1 Fixed the imports only, in the two modules that have them:
      `simulation/spiderbot.py` (`SignalPort`) and `simulation/leg.py`
      (`RotationalPort`), each split into
      `from solid_node.node import AssemblyNode` and
      `from solid_node.motion.ports import SignalPort` /
      `from solid_node.motion.ports import RotationalPort`. Nothing else
      touched; `body.py`, `printed.py`, `sourced.py` keep their
      `solid_node.node` imports (`AssemblyNode`, `StlNode`,
      `CadQueryNode` — not ports, untouched).
- [x] 1.2 `solid test --faceted simulation/spiderbot.py:Spiderbot`:

          Ran 34 tests in 7.80 seconds: 34 passed, 0 failed
          (faceted kernel, volume epsilon 0 mm³)

      All 34 test methods across the 9 classes named in the proposal's
      test table passed; none failed. (The proposal's "Tests" section
      says 28 methods; the file actually declares 34 — `grep -c "def
      test_" simulation/test_spiderbot.py` — a miscount in the proposal,
      not a stage A finding; every one of the 34 is green.) This is the
      baseline stage B would have been measured against.
- [x] 1.3 Captured poses:

          PYTHONPATH=. python /home/asa/devel/libresolid-studio/docs/motion-general-refactor/capture_poses.py capture simulation.spiderbot:Spiderbot /tmp/hexapod-spiderbot-before.json /tmp/hexapod-poses/extra.json

      with an `extra` pose file (`{driver_id: value}` per instruction's
      target dict, from `Spiderbot.instructions` in `spiderbot.py`) for
      the seven named instructions (`Stand`, `Crouch`, `Tiptoe`, `Sit`,
      `Wave`, `LookAround`, `Walk`). Result: **captured 28 poses, 172
      leaves -> /tmp/hexapod-spiderbot-before.json** (21 range/default/
      time poses + 7 named-instruction poses).
- [x] 1.4 Committed as
      `refactor(simulation): import ports from solid_node.motion`,
      with the baseline in the message body, including this change
      directory. **Deferred after this commit** (see proposal's "Known
      gaps" 1 and its "Deferred (2026-09-09)" section): the chassis's
      four freedoms on one body have no stated composition order in the
      framework yet, so stages 2-5 below are NOT performed in this
      change. They stay unchecked as the record of what stage B would
      do once the framework states that order.

## 2. Joints — `simulation/leg.py`

- [ ] 2.1 Import `Revolute` from `solid_node.motion.joints`; drop the
      `RotationalPort` import once step 2.3 has removed the last one.
- [ ] 2.2 Declare the three freedoms on the bodies that have them:
      `Coxa.yaw = Revolute(axis=(0, 0, 1), unit='deg')` (no `at`: the
      coxa's rest placement is empty);
      `Femur.lift = Revolute(axis=(0, -1, 0), at=(COXA_LENGTH, 0.0, joint.FORK_MID), unit='deg')`;
      `Tibia.knee = Revolute(axis=(0, -1, 0), at=(FEMUR_LENGTH, 0.0, 0.0), unit='deg')`.
      No `range` on any of them — the design record measures no servo
      travel limit. The negated y axes carry the model's sign convention
      (positive lift raises the femur, positive knee raises the shin)
      into the declaration; say so in a comment where the old `simulate()`
      comment said it.
- [ ] 2.3 Delete the six forwarding ports — `Leg.coxa_angle`, `Leg.lift`,
      `Leg.knee`, `Coxa.lift`, `Coxa.knee`, `Femur.knee` — and delete
      `Leg.simulate()`, `Coxa.simulate()` and `Femur.simulate()`
      entirely. Leave all three `render()` methods untouched, and leave
      `Leg.LINKS` alone.
- [ ] 2.4 Update `Leg`'s class docstring: the three angles no longer
      arrive as ports; the root solves them and binds the joints the
      three bodies declare.

## 3. Relations and bindings — `simulation/spiderbot.py`

- [ ] 3.1 In `Chassis.simulate()`, bind the joints by path instead of the
      ports: `leg.coxa.yaw`, `leg.coxa.femur.lift`,
      `leg.coxa.femur.tibia.knee`. Leave `_solve`, `_foot_target`,
      `_to_chassis`, `station()` and `_rectify()` untouched, line for
      line.
- [ ] 3.2 State the eight driver relations in the `Spiderbot` class body,
      after the driver declarations and before `instructions`:
      `height.drives(chassis.height)`, `reach.drives(chassis.reach)`,
      `stride.drives(chassis.stride)`, `roll.drives(chassis.roll)`,
      `pitch.drives(chassis.pitch)`, `yaw.drives(chassis.yaw)`,
      `gait_phase.drives(chassis.gait_phase)`,
      `wave.drives(chassis.wave)`. No `ratio`, no `offset`, no `law`:
      the units match at both ends of all eight.
- [ ] 3.3 Delete the eight hand-down assignments from
      `Spiderbot.simulate()`. Keep the four attitude operations exactly
      as they are, in the same order — `rotate(roll, x)`,
      `rotate(pitch, y)`, `rotate(yaw, z)`,
      `translate([0, 0, height])` — and keep the comment saying the legs
      are solved against exactly this composition. Add one line saying
      why these four are not joints (four freedoms on one body compose in
      binding order; see the proposal's Known gaps).
- [ ] 3.4 `Chassis`'s eight `SignalPort`s stay. They are read by the
      chassis's own arithmetic, not forwarded.

## 4. Evidence again

- [ ] 4.1 Re-capture to `/tmp/hexapod-spiderbot-after.json` (and the
      `extra` instruction poses) and run
      `capture_poses.py compare before after`. Record the leaf count, the
      pose count and the maximum deviation. Expected: 0.
- [ ] 4.2 `solid test --faceted simulation/spiderbot.py:Spiderbot` again.
      Record the per-test result against 1.2: the same tests green, none
      newly red. Pay particular attention to
      `JointStackTest.test_a_joint_moves_only_what_is_beyond_it` and to
      `LegKinematicsTest.test_the_foot_is_where_the_solution_says` — those
      two are what catch a joint that composed in the wrong frame.
- [ ] 4.3 Record every deviation from the proposal here, with the
      framework behaviour that forced it.
- [ ] 4.4 One sentence in `README.md`, in "What it does": the three leg
      freedoms are declared joints on the bodies that carry them, the
      eight drivers reach the chassis by relation, and the chassis's own
      attitude is still four operations by hand because four freedoms on
      one body have no stated composition order.
- [ ] 4.5 Commit as
      `refactor(simulation): move the SpiderBot onto solid-node joints and couplings`,
      with the pose comparison and the test result in the body.

## 5. Report

- [ ] 5.1 Report the two commit hashes, the pose comparison line, the
      test counts before and after, every deviation from the proposal,
      and every test you believe needs a change with the reason. Do not
      sync or archive this change; the orchestrator does that after
      review.
