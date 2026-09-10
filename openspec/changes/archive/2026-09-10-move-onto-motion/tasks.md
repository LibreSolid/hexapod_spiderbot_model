Behaviour-preserving throughout: the acceptance is that every leaf's world
matrix is unchanged at every pose, so evidence is captured before the first
edit and compared after the last. Run everything from the project root with
`PYTHONPATH=.` and the workspace venv
(`/home/asa/devel/libresolid-studio/.venv/bin/python`,
`.venv/bin/solid`). `.env` selects the faceted kernel; leave it untracked.
Never run two suites at once. Never edit a test — if one blocks you, stop,
name the assertion and the reason, and wait.

**Stage B done on `Free` (2026-09-10).** The primitive stage B waited for
landed on solid-node main `ef379d2`: ADR-093 (the joints of one class compose
in DECLARATION order, innermost first) and ADR-095 (`Free`, whose own design
record and fixture use this chassis as their worked example). On the
orchestrator's direction the chassis is stated as ONE `Free` joint rather
than the four joints the proposal's "Known gaps" 1 asked for, so sections 2
to 4 below are recorded as executed, with the deviations that follow from
`Free` marked **DEVIATION**.

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

- [x] 2.1 Import `Revolute` from `solid_node.motion.joints`; drop the
      `RotationalPort` import once step 2.3 has removed the last one.
      **Done (2026-09-10).** `leg.py` now imports nothing from
      `solid_node.motion.ports` at all.
- [x] 2.2 Declare the three freedoms on the bodies that have them:
      `Coxa.yaw = Revolute(axis=(0, 0, 1), unit='deg')` (no `at`: the
      coxa's rest placement is empty);
      `Femur.lift = Revolute(axis=(0, -1, 0), at=(COXA_LENGTH, 0.0, joint.FORK_MID), unit='deg')`;
      `Tibia.knee = Revolute(axis=(0, -1, 0), at=(FEMUR_LENGTH, 0.0, 0.0), unit='deg')`.
      No `range` on any of them — the design record measures no servo
      travel limit. The negated y axes carry the model's sign convention
      (positive lift raises the femur, positive knee raises the shin)
      into the declaration; say so in a comment where the old `simulate()`
      comment said it.
      **Done, exactly as proposed**, all three, with the sign convention
      stated in a comment on each. Three declarations, eighteen realized
      freedoms over the six legs. Each is the only joint its class
      declares, and each carries the whole run at its own declaration slot.
- [x] 2.3 Delete the six forwarding ports — `Leg.coxa_angle`, `Leg.lift`,
      `Leg.knee`, `Coxa.lift`, `Coxa.knee`, `Femur.knee` — and delete
      `Leg.simulate()`, `Coxa.simulate()` and `Femur.simulate()`
      entirely. Leave all three `render()` methods untouched, and leave
      `Leg.LINKS` alone.
      **Done.** All six ports and all three `simulate()` methods are gone;
      no `render()` and no line of `Leg.LINKS` was touched. The two hand
      negations (`rotate(-self.lift.value, [0, 1, 0])` and
      `rotate(-self.knee.value, [0, 1, 0])`) went with them, into the
      declared axes.
- [x] 2.4 Update `Leg`'s class docstring: the three angles no longer
      arrive as ports; the root solves them and binds the joints the
      three bodies declare.
      **Done**, and the module docstring with it: the leg is now three
      nested assemblies each carrying the `Revolute` that turns it, and
      the framework rather than the file is what gets each body onto its
      axis.

## 3. Relations and bindings — `simulation/spiderbot.py`

- [x] 3.1 In `Chassis.simulate()`, bind the joints by path instead of the
      ports: `leg.coxa.yaw`, `leg.coxa.femur.lift`,
      `leg.coxa.femur.tibia.knee`. Leave `_solve`, `_foot_target`,
      `_to_chassis`, `station()` and `_rectify()` untouched, line for
      line.
      **Done.** The six-line loop is unchanged but for the three binding
      targets. `_solve`, `_foot_target`, `station()` and `_rectify()` are
      untouched line for line; `_to_chassis` changed only in the four
      value reads named at 3.3 below. The deep-path binding of the
      proposal's "Known gaps" 4 behaved: no femur and no tibia moved (4.1).
- [x] 3.2 State the eight driver relations in the `Spiderbot` class body,
      after the driver declarations and before `instructions`:
      `height.drives(chassis.height)`, `reach.drives(chassis.reach)`,
      `stride.drives(chassis.stride)`, `roll.drives(chassis.roll)`,
      `pitch.drives(chassis.pitch)`, `yaw.drives(chassis.yaw)`,
      `gait_phase.drives(chassis.gait_phase)`,
      `wave.drives(chassis.wave)`. No `ratio`, no `offset`, no `law`:
      the units match at both ends of all eight.
      **Done — eight relations, no ratio, no offset, no law — but four of
      them end on a `Free` coordinate. DEVIATION**, and the direct
      consequence of stating the chassis as a joint:

          roll.drives(chassis.pose.roll)
          pitch.drives(chassis.pose.pitch)
          yaw.drives(chassis.pose.yaw)
          height.drives(chassis.pose.z)
          reach.drives(chassis.reach)
          stride.drives(chassis.stride)
          gait_phase.drives(chassis.gait_phase)
          wave.drives(chassis.wave)

      The proposal wrote `roll.drives(chassis.roll)` because the chassis
      then forwarded `roll` into a hand-written `rotate()`. With the
      attitude declared, `chassis.pose.roll` IS that coordinate, and a
      `Chassis.roll` port beside it would be a second copy of the same
      number — exactly the plumbing this change deletes. Units still match
      at both ends of all eight (deg to deg, mm to mm, turn to turn,
      dimensionless to dimensionless).
      **Relations were chosen over assignment in the root's `simulate()`**:
      a relation is stated in the class body where a reader finds it, it
      needs no `simulate()` at all (the root now has none), and its
      ordering is the one this model depends on — a relation stated on an
      ancestor and reaching a descendant's coordinate by path is solved at
      the end of the ancestor's simulate phase, which is before
      `Chassis.simulate()` reads those values. All four dotted paths
      resolved as relation ends with no ceremony.
      They are stated AFTER `instructions` rather than before it, because
      `drives` reads the class-body names and the block reads as the last
      word on what drives what.
- [x] 3.3 Delete the eight hand-down assignments from
      `Spiderbot.simulate()`. Keep the four attitude operations exactly
      as they are, in the same order — `rotate(roll, x)`,
      `rotate(pitch, y)`, `rotate(yaw, z)`,
      `translate([0, 0, height])` — and keep the comment saying the legs
      are solved against exactly this composition. Add one line saying
      why these four are not joints (four freedoms on one body compose in
      binding order; see the proposal's Known gaps).
      **Superseded. DEVIATION — this is the whole point of stage B on
      `Free`.** The four attitude operations are NOT kept: they are the
      joint. `Chassis` declares, first in its class body,

          pose = Free(angle_unit='deg', length_unit='mm')

      and `Spiderbot.simulate()` is deleted outright — the root has no
      `simulate()` at all now. No `at` (the chassis has no rest placement
      of its own, so the parent frame's origin is also its own placed
      origin and is the point the three rotations pass through); no `axis`
      and no `range`, which a `Free` does not take.
      The framework composes it `T(x,y,z) · Rz(yaw) · Ry(pitch) · Rx(roll)`,
      roll innermost — exactly the chain lines 280-284 wrote by hand and
      `Chassis._to_chassis` inverts — and that order is now the contract
      rather than a comment. `pose.x` and `pose.y` are never bound and
      contribute nothing: a robot that walks in place does not slide
      sideways, and it does not have to say so with a zero.
      `_to_chassis` reads `self.pose.z.value`, `self.pose.yaw.value`,
      `self.pose.pitch.value` and `self.pose.roll.value` in place of the
      four deleted ports; its docstring says the chassis's own `pose`
      joint is what it inverts.
- [x] 3.4 `Chassis`'s eight `SignalPort`s stay. They are read by the
      chassis's own arithmetic, not forwarded.
      **Four stay, four go. DEVIATION**, same cause as 3.2/3.3.
      `Chassis.stride`, `reach`, `gait_phase` and `wave` stay: each is
      read and consumed by the chassis's own arithmetic and has no joint
      to be. `Chassis.height`, `roll`, `pitch` and `yaw` are deleted,
      because `pose.z`, `pose.roll`, `pose.pitch` and `pose.yaw` are those
      four values. Ten ports leave this change in all, not six. None of
      the ten is named by a test, by `README.md` or by any spec under
      `openspec/specs/`.

## 4. Evidence again

- [x] 4.1 Re-capture to `/tmp/hexapod-spiderbot-after.json` (and the
      `extra` instruction poses) and run
      `capture_poses.py compare before after`. Record the leaf count, the
      pose count and the maximum deviation. Expected: 0.
      **Done (2026-09-10): `max deviation 0.000e+00 over 28 poses`,
      172 leaves.** The baseline was RE-captured at `e5d1147` on framework
      `ef379d2` first, so both files are the same framework, and with the
      same seven-instruction `extra` pose file stage A used. Bit-exact —
      not 1e-12 of trig residue, exactly zero — at every one of
      `defaults`, `gait_phase@{0.4,1.0}`, `height@{0.4,1.0}`,
      `pitch@{0.4,1.0}`, `reach@{0.4,1.0}`, `roll@{0.4,1.0}`,
      `stride@{0.4,1.0}`, `wave@{0.4,1.0}`, `yaw@{0.4,1.0}`, `all@0.63`,
      `time@{0.25,0.5,0.75}` and the seven named instructions `Stand`,
      `Crouch`, `Tiptoe`, `Sit`, `Wave`, `LookAround`, `Walk`.
      The port column cannot compare across a rename, so the eighteen
      joint values were compared separately against the forwarding ports
      they replace (`Leg.coxa_angle` → `coxa.yaw`, `Leg.lift` →
      `coxa.femur.lift`, `Leg.knee` → `coxa.femur.tibia.knee`) over all
      28 poses: **504 comparisons, maximum deviation 0.0.**
- [x] 4.2 `solid test --faceted simulation/spiderbot.py:Spiderbot` again.
      Record the per-test result against 1.2: the same tests green, none
      newly red. Pay particular attention to
      `JointStackTest.test_a_joint_moves_only_what_is_beyond_it` and to
      `LegKinematicsTest.test_the_foot_is_where_the_solution_says` — those
      two are what catch a joint that composed in the wrong frame.
      **Done: `Ran 34 tests in 8.03 seconds: 34 passed, 0 failed`**
      (faceted kernel, volume epsilon 0 mm³) — the same 34 as the 1.2
      baseline, none newly red, none newly green. Both named tests passed,
      as did `PostureTest.test_attitude_moves_the_body_and_not_the_feet`
      (the one that would catch a wrong attitude composition) and
      `WholeRobotTest.test_assembly_integrity`. No test file was opened
      for edit.
- [x] 4.3 Record every deviation from the proposal here, with the
      framework behaviour that forced it.
      **Done.** Four, all recorded above and all consequences of stating
      the chassis with `Free` instead of four joints:
      (a) 3.3 — the four attitude operations are gone, not kept, and
      `Spiderbot.simulate()` is deleted rather than shrunk to four
      statements;
      (b) 3.2 — four of the eight relations end on `chassis.pose.*`
      instead of on a chassis `SignalPort`;
      (c) 3.4 — ten forwarding ports leave, not six;
      (d) the relation block sits after `instructions`, not before it.
      Nothing else in the proposal was departed from: every joint in
      section 2 is the axis, anchor and unit the proposal specified.
      **One thing the contract does not carry, reported and not worked
      around:** `declared_ports(Chassis)` reports the six under dotted
      keys (`pose.roll`, …) and plain `getattr(node, 'pose.roll')` raises
      `AttributeError`. The framework offers
      `solid_node.motion.ports.set_coordinate` for WRITING a dotted name
      but no matching reader. Nothing in this project needs one; the
      shop's `capture_poses.py` does, and recorded `<AttributeError>` in
      the port column for all six — harmless, because the acceptance is
      the leaf matrices and the joint values were compared separately.
- [x] 4.4 One sentence in `README.md`, in "What it does": the three leg
      freedoms are declared joints on the bodies that carry them, the
      eight drivers reach the chassis by relation, and the chassis's own
      attitude is still four operations by hand because four freedoms on
      one body have no stated composition order.
      **Done, with the last clause no longer true.** The sentence reads:
      the chassis floats on one `Free` joint whose roll, pitch, yaw and
      lift the four attitude drivers drive directly, and each leg's yaw,
      lift and knee is a `Revolute` on the body it turns. Nothing in this
      robot moves by hand any more.
- [x] 4.5 Commit as
      `refactor(simulation): move the SpiderBot onto solid-node joints and couplings`,
      with the pose comparison and the test result in the body.
      **Done.**

## 5. Report

- [x] 5.1 Report the two commit hashes, the pose comparison line, the
      test counts before and after, every deviation from the proposal,
      and every test you believe needs a change with the reason. Do not
      sync or archive this change; the orchestrator does that after
      review.
      **Done (2026-09-10).** No test needs a change: 34/34 green before,
      34/34 green after, none touched. Not synced and not archived —
      awaiting the orchestrator's review.
