# Move the SpiderBot simulation onto solid-node's motion layer

## Why

solid-node moved ports and the declared time base out of `solid_node.node`
into `solid_node.motion.ports`, with no re-export, and added the motion
layer proper: `solid_node.motion.joints` (`Revolute`, `Prismatic`) and
`solid_node.motion.couplings` (`drives`, `Affine`, derived coordinates).
This model does not import today:

    ImportError: module 'solid_node.node' has no attribute 'SignalPort':
    ports and the declared time base moved to 'solid_node.motion.ports'.

Fixing the import is one line. It is not the point. This model states
twenty-two freedoms and declares none of them. Eighteen of them — three
per leg, six legs — are stated as a chain of six `RotationalPort`s whose
only job is to carry a number one level further down the tree, two of
them negated by hand at the point of use because the leg's sign
convention disagrees with the framework's rotation sense. The root
hands eight pose values to its one child by writing eight assignments.
None of that is the machine; all of it is plumbing that the motion layer
was built to delete.

What the machine actually is, and what this change makes it say:

- the coxa turns about the yaw servo's axis, which is the leg frame's z;
- the femur turns about the lift servo's axis, `COXA_LENGTH` out and
  `FORK_MID` up in the coxa's frame;
- the tibia turns about the knee servo's axis, `FEMUR_LENGTH` out in the
  femur's frame;
- and the eight drivers on the root drive the eight pose values the
  chassis solves from.

The inverse kinematics stays. The motion layer states where a body may
move and how one coordinate follows another; it does not solve a
two-link reach from a foot target, and this model's whole premise is
that the root steers the *body* and every joint angle follows from it.
`Chassis._solve` is that solution, deliberately written so a reader can
check it against the measured link lengths. It is arithmetic that
produces coordinate values, and it binds joints with those values —
exactly the case the joints spec names when it says an assembly's
`simulate()` may bind a child's joint from a number.

## What changes

### The joints to declare — three declarations, eighteen realized

All three are `Revolute`, declared on the body that moves, with axis and
anchor in the parent's frame, as the joints spec requires. No `range` is
declared on any of them: the design record measures no servo travel
limit anywhere (the servos are identified only as "a 40 mm-class digital
servo"), and declaring a range the record does not have would invent a
constraint and could refuse a pose that renders today.

**1. `Coxa.yaw` — the leg's yaw, in `simulation/leg.py`.**

    yaw = Revolute(axis=(0, 0, 1), unit='deg')

- Parent: `Leg`. `Leg.render()` places `yaw_servo` and nothing else, so
  the coxa's rest placement is empty and its placed origin is the leg
  frame's origin. `at` therefore defaults correctly and is omitted.
- Axis from `params.STATION_Z` / the leg-frame convention documented at
  the top of `leg.py`: the leg's z is up and is the yaw servo's shaft.
- Replaces `Leg.simulate()`'s
  `self.coxa.rotate(self.coxa_angle.value, [0, 0, 1])`. Same sign, same
  axis, same anchor: the carried local anchor is zero, so the framework
  emits one `rotate(value, [0, 0, 1])` and no centring translations —
  byte-identical to today's operation.

**2. `Femur.lift` — the lift freedom, in `simulation/leg.py`.**

    lift = Revolute(axis=(0, -1, 0), at=(COXA_LENGTH, 0.0, joint.FORK_MID),
                    unit='deg')

- Parent: `Coxa`. `Coxa.render()` places the femur with
  `translate([COXA_LENGTH, 0.0, joint.FORK_MID])`, so the anchor is that
  same point — the femur's placed origin, restated because the framework
  has no way to say "my own placed origin" (see **Known gaps**).
- The numbers: `COXA_LENGTH = COXA_FORK_X - COXA_HUB[0] = 43.5 mm`, the
  measured horn-to-fork distance in `params.py`; `joint.FORK_MID`, the
  middle of the fork derived in `joint.py` from the horn-plate and
  bearing-plate faces, which is where a servo bolted across a fork puts
  its shaft.
- The axis is the *negated* leg y. Today `Coxa.simulate()` writes
  `self.femur.rotate(-self.lift.value, [0, 1, 0])`: positive lift raises
  the femur, and a positive turn about the leg's own +y lowers it.
  Negating the axis moves that convention into the declaration, where it
  belongs, instead of leaving a minus sign at every bind site. The two
  are the same rotation matrix exactly (`sin(-θ) == -sin(θ)` in IEEE
  arithmetic and the axis components are exact), so the pose does not
  move.
- Because the local anchor carries to zero, the framework emits one
  `rotate(value, [0, -1, 0])` and no centring translations.

**3. `Tibia.knee` — the knee freedom, in `simulation/leg.py`.**

    knee = Revolute(axis=(0, -1, 0), at=(FEMUR_LENGTH, 0.0, 0.0),
                    unit='deg')

- Parent: `Femur`. `Femur.render()` places the tibia with
  `translate([FEMUR_LENGTH, 0.0, 0.0])`.
- `FEMUR_LENGTH = FEMUR_HUB_DISTAL[0] - FEMUR_HUB_PROXIMAL[0] = 80.0 mm`,
  the measured hub-to-hub distance in `params.py`.
- Axis negated for the same reason: `Femur.simulate()` writes
  `self.tibia.rotate(-self.knee.value, [0, 1, 0])`, with the comment
  "positive knee raises the shin, and a positive turn about the leg's own
  y axis lowers it, so the driver enters negated". That comment becomes
  the axis.
- Local anchor carries to zero; one rotation, no centring translations.

### The relations — eight `drives` sentences, all on `Spiderbot`

Every one names its coordinate at both ends. All eight are the identity
affine (no `ratio`, no `offset`, no `law`): a driver and the port it
feeds are the same quantity in the same unit, and there is nothing to
convert. Stated in the `Spiderbot` class body, in `simulation/spiderbot.py`:

    height.drives(chassis.height)          # Driver mm  -> Chassis.height     SignalPort mm
    reach.drives(chassis.reach)            # Driver mm  -> Chassis.reach      SignalPort mm
    stride.drives(chassis.stride)          # Driver mm  -> Chassis.stride     SignalPort mm
    roll.drives(chassis.roll)              # Driver deg -> Chassis.roll       SignalPort deg
    pitch.drives(chassis.pitch)            # Driver deg -> Chassis.pitch      SignalPort deg
    yaw.drives(chassis.yaw)                # Driver deg -> Chassis.yaw        SignalPort deg
    gait_phase.drives(chassis.gait_phase)  # Driver turn-> Chassis.gait_phase SignalPort turn
    wave.drives(chassis.wave)              # Driver ''  -> Chassis.wave       SignalPort ''

Each replaces one line of `Spiderbot.simulate()`. Each is legal where it
is written: a `Driver` may be a source, `chassis` is a plain declared
child (not repeated, not list-held), and the driven end is a path to a
port that child declares. Units match at both ends of all eight, so no
conversion is asked of the framework, which performs none anyway.

The ordering is the one the framework guarantees: a relation stated on an
ancestor and reaching a descendant's coordinate by path is solved at the
end of the ancestor's simulate phase, and a parent's simulate phase runs
before any child's. So all eight ports hold their values before
`Chassis.simulate()` runs and reads them.

### Derived coordinates — none

There is no linear formula over two coordinates anywhere in this model.
The knee is declared relative to the femur, which is what a `Revolute` on
a nested body already is; the lift is relative to the coxa; the yaw is
relative to the leg. `TIBIA_BEND` (subtracted in `_solve` to get the knee
driver out of the two-link solution) is a constant read off the shin's
own placement transform, not a coordinate, so `bend - TIBIA_BEND` is
arithmetic and not a derived coordinate. Inventing an absolute-tibia-
pitch coordinate that nothing reads would be inventing motion the model
does not have.

### The ports that go — six

All six are pure forwarding: declared only so a parent can hand a number
to a child that hands it to its child.

| Port | Class | File | Replaced by |
|---|---|---|---|
| `coxa_angle` | `Leg` | `leg.py` | `leg.coxa.yaw` bound by path |
| `lift` | `Leg` | `leg.py` | `leg.coxa.femur.lift` bound by path |
| `knee` | `Leg` | `leg.py` | `leg.coxa.femur.tibia.knee` bound by path |
| `lift` | `Coxa` | `leg.py` | the `Femur.lift` joint |
| `knee` | `Coxa` | `leg.py` | the `Tibia.knee` joint |
| `knee` | `Femur` | `leg.py` | the `Tibia.knee` joint |

With these gone, `leg.py` imports nothing from `solid_node.motion.ports`
at all: it imports `Revolute` from `solid_node.motion.joints` instead.

None of the six is named in a test, in `README.md`, or in any spec under
`openspec/specs/`. The child nodes whose names look similar —
`Coxa.lift_servo`, `Coxa.lift_adapter`, `Femur.lift_horn`,
`Femur.lift_bearing`, `Femur.knee_horn`, `Femur.knee_bearing`,
`Coxa.yaw_horn`, `Coxa.yaw_bearing`, `Coxa.yaw_adapter` — are untouched,
and none of them collides with a new joint name.

### The ports that stay — eight, and they are not forwarding

`Chassis.height`, `reach`, `stride`, `roll`, `pitch`, `yaw`,
`gait_phase` and `wave` stay exactly as they are. Each is *read and
consumed* by `Chassis`'s own arithmetic — `_foot_target` reads
`gait_phase`, `stride` and `reach`; `_to_chassis` reads `height`, `yaw`,
`pitch` and `roll`; `_solve` reads `wave` — so none of them is a
forwarding port, and there is nothing further down the tree for them to
forward to. They are the chassis's control surface, and they are what the
eight new relations drive.

### The `simulate()` methods that shrink or disappear

| Method | Today | After |
|---|---|---|
| `Leg.simulate` | 3 statements: rotate the coxa, forward `lift`, forward `knee` | **deleted** |
| `Coxa.simulate` | 2 statements: rotate the femur (negated), forward `knee` | **deleted** |
| `Femur.simulate` | 1 statement: rotate the tibia (negated) | **deleted** |
| `Spiderbot.simulate` | 12 statements: 8 hand-downs, then 4 attitude operations | 4 statements: the attitude operations only |
| `Chassis.simulate` | 6-line loop binding three ports per leg | 6-line loop binding three joints per leg, by path |

`Leg`, `Coxa` and `Femur` keep their `render()` methods unchanged: those
place printed parts at rest and have nothing to do with motion.

`Chassis.simulate()` becomes:

    def simulate(self):
        for leg, (name, x, y, heading) in zip(self.legs, STATIONS):
            coxa, lift, knee = self._solve(name, x, y, heading)
            leg.coxa.yaw = coxa
            leg.coxa.femur.lift = lift
            leg.coxa.femur.tibia.knee = knee

This is a binding by path on an instance, not a relation, and so it is
not affected by the rule that a relation may not path through a
`.repeat()` child: `self.legs` yields the six realized `Leg` instances
and `leg.coxa.femur.tibia.knee` is ordinary attribute access on them. It
is the only way the six legs can be reached, because each leg's three
values come out of a different call of `_solve` — different station,
different heading, different tripod phase — and no relation and no law
can express that: a law relates one driver coordinate to one driven
coordinate, and this maps eight pose values plus `$t` onto eighteen.

### The hand-written frame inversions that go — two

`Coxa.simulate`'s `rotate(-self.lift.value, [0, 1, 0])` and
`Femur.simulate`'s `rotate(-self.knee.value, [0, 1, 0])`: both a hand
negation standing in for a sign convention. Both become the joint's
declared axis.

There are no hand-written *anchor* inversions to remove in this model.
The original design already gave every rigid body its own frame with its
origin on the axis that drives it (`leg.py`'s docstring calls this "the
whole trick"), which is exactly the arrangement the joints layer makes
unnecessary. The gain here is that the arrangement is now stated rather
than relied on: `at=(COXA_LENGTH, 0.0, joint.FORK_MID)` says out loud
where the lift axis is, and the framework — not the author — is
responsible for getting the body onto it.

## What does not change

- **The eight drivers**, their defaults, ranges, units and ids.
- **The seven instructions** — `Stand`, `Crouch`, `Tiptoe`, `Sit`,
  `Wave`, `LookAround`, `Walk` — and their targets and durations.
- **Every rest placement.** No `render()` is touched anywhere in the
  project. No printed or sourced part moves.
- **The inverse kinematics.** `Chassis._foot_target`, `_to_chassis`,
  `_solve`, `station()` and `_rectify()` are unchanged, line for line,
  including the tripod phasing, the `$t` term, the swing lift and the
  wave.
- **The chassis attitude.** `Spiderbot.simulate()` keeps its four
  operations — `rotate(roll, x)`, `rotate(pitch, y)`, `rotate(yaw, z)`,
  `translate([0, 0, height])` — in that exact order. See **Known gaps**.
- **`params.py`, `joint.py`, `printed.py`, `sourced.py`, `seats.py`,
  `body.py`.** None of them declares a port or moves anything.
- **Every test**, every assertion and every tolerance in
  `simulation/test_spiderbot.py`.
- **Every spec** under `openspec/specs/`: the six capabilities describe
  the interfaces between parts, not how a value reaches a joint.
- **`README.md`** gains one sentence in "What it does" saying the leg
  freedoms are declared joints; nothing it claims stops being true.
- **The pose at every instant**, which is the acceptance criterion:
  maximum deviation 0 over every leaf at every captured pose.

## Known gaps

### 1. Four freedoms on one body: the chassis's ground attitude

This is the second known limit — *two joints on one body compose in
binding order, which a relation cannot see* — and it is the reason
`Spiderbot.simulate()` does not disappear.

The chassis is one rigid body with four freedoms against the ground:
roll, pitch, yaw and lift. The model's whole premise depends on their
composition being exactly `R_roll · R_pitch · R_yaw · T_height`, because
`Chassis._to_chassis` hand-inverts precisely that composition to bring a
foot target on the ground into the chassis's frame. Stating the four as
four joints would put the pose at the mercy of the order the four
coordinates happen to be bound in, which the joints spec fixes as
*binding* order and the couplings spec explicitly declines to make
depend on the order the author wrote the relations in. A reader of the
class would have no way to see the order, and a later derived coordinate
or an extra relation could silently change it.

The sentence the project wants:

    class Chassis(AssemblyNode):
        roll  = Revolute(axis=(1, 0, 0), unit='deg')
        pitch = Revolute(axis=(0, 1, 0), unit='deg')
        yaw   = Revolute(axis=(0, 0, 1), unit='deg')
        lift  = Prismatic(axis=(0, 0, 1), unit='mm')

composed innermost-first in **declaration** order — roll inside pitch
inside yaw inside lift — whatever order the four coordinates are bound
in, so that the four drivers could then be eight relations instead of
four relations and four hand-written operations.

This is the same primitive OpenCycloid is deferred on (its orbit needs
spin inside orbit), reached from the other side: not a compound joint but
a floating body's attitude. **Recommendation: record it as a second
sighting and do NOT defer this project.** Deferring would hold eighteen
cleanly statable joints, six forwarding ports and two hand negations
hostage to four lines in the root that are correct today and stay
correct. The orchestrator decides; if the campaign's rule is read
strictly — a known limit leaving motion hand-written defers the project
— then this is the trigger, and stage A alone is the answer.

### 2. `at` restates the parent's `translate` — third and fourth sighting

`Femur.lift` must write `at=(COXA_LENGTH, 0.0, joint.FORK_MID)` and
`Tibia.knee` must write `at=(FEMUR_LENGTH, 0.0, 0.0)`, and both anchors
are exactly the node's own placed origin: `Coxa.render()` and
`Femur.render()` translate by those same vectors, three lines away. The
numbers are written twice in the same file. This is the finding already
recorded from Poseidon and OpenMANIPULATOR-X, from a third direction: not
a shared catalogue class, but a single-use class whose parent's placement
is repeated as class metadata a few lines below the placement itself.

The sentence the project wants:

    lift = Revolute(axis=(0, -1, 0), unit='deg')    # anchored at my own placed origin

It does not defer anything: the motion is fully stated either way, and
the cost is duplication.

### 3. Not a framework gap: the inverse kinematics

`_to_chassis` un-does the chassis's own placement to express a ground
point in the chassis's frame, and `_solve` runs a two-link reach. Neither
is a motion the motion API declines to state — they are a solution the
motion API does not claim to compute. The motion layer is forward:
coordinates place bodies. A machine steered by the pose of its body
inverts by hand, and this model does so deliberately and legibly. No wart
is proposed for it.

### 4. Behavioural risk for the implementer, not a gap

`Chassis.simulate()` binds joints on nodes two and three levels below it
(`leg.coxa.femur.lift`, `leg.coxa.femur.tibia.knee`), during the
chassis's own simulate phase and therefore before those nodes' own
lifecycle runs. The joints spec covers this — joint operations are
placed as motion whatever phase is current, tagged with the assembly
that bound them, and swept before *that* assembly's next run — and Thor
does the same thing by relation three levels down with maximum deviation
0. It is nonetheless the one place this refactor could move a pose. If
the pose comparison shows any deviation on the femur or the tibia, stop
and report it rather than working around it: the fallback would be
wirings or per-leg ports, and reintroducing a port is a decision for the
orchestrator.

## Pre-existing state

`git status` in this repository is **clean** on branch `main` at the time
of writing: no modified, staged or untracked tracked-path changes. The
only untracked artefacts are build output already covered by
`.gitignore` (`_build/`, `_build.lock`) and the local `.env` selecting
the faceted test kernel (`SOLID_TEST_KERNEL=faceted`), which stays
untracked.

Stage 0 therefore has nothing to commit. The implementer should confirm
this with `git status` before stage A and record "clean, nothing to
commit" in `tasks.md` rather than skipping the check.

## Tests

`simulation/test_spiderbot.py` is the whole suite: 9 classes, 28 test
methods, run by `solid test --faceted` against the one declared model
`simulation.spiderbot:Spiderbot` (`pyproject.toml`, `[tool.solid-node]`).
There are no plain `pytest` tests. `.env` selects the faceted kernel.

| Class | Asserts | Touched by this change? |
|---|---|---|
| `PublishedPartsTest` (2) | every STL closes; the servo flange matches the holders | No — geometry only |
| `JointStackTest` (5) | fork gap = the stack; rib agrees; horn/bearing coaxial at all 18 joints; 18 bearings; a joint moves only what is beyond it | Only through the pose, which does not move |
| `LegKinematicsTest` (3) | link lengths 43.5 / 80.0; lift and knee axes parallel; the foot is where the solution says | Only through the pose |
| `StationLayoutTest` (4) | six legs at the published headings; every yaw axis vertical; holder count; stance symmetric | Only through the pose |
| `BodyStackTest` (5) | carapace closes and stands; battery inside; controller on its plate; body parts share no volume | No — the body has no joints |
| `FootTest` (3) | shin bolted to bracket; claw lowest; only claws touch the ground | Only through the pose |
| `PostureTest` (5) | standing level and planted; attitude moves the body and not the feet; crouch; tripod always three down; no claw below the plane; waving keeps five standing | Only through the pose |
| `WholeRobotTest` (3) | `assert_solid_integrity`; `assert_assembly_integrity` / `seats.assert_inventory`; the published body files overlap where the plate seats | Only through the pose |
| `PostureScenarioTest` (3) | the instructions land where they say; feet planted through a look-around; walking keeps a tripod down | No — drivers and instructions unchanged |

**Tests I expect to need a change: none.** No test names any of the six
deleted ports, none calls `declared_ports`, none binds a port or a joint
directly, and every driver id and instruction name survives. The three
tests that reach into the leg by attribute path
(`leg.coxa.femur.lift_horn`, `leg.coxa.femur.tibia.claw`, …) reach
*children*, whose names are untouched.

Two things to watch, neither of which this change causes, and neither of
which I am deciding:

1. **`WholeRobotTest.test_assembly_integrity`** runs
   `seats.assert_inventory` over every overlapping pair. Thor's migration
   found this contract already red on the current framework tree for
   reasons predating the motion branch (an exact-boolean or seats change).
   This project runs the *faceted* kernel, so it may well be unaffected,
   but stage A's baseline is what settles it: if it is red before the
   refactor it must be red the same way after, and it is not this
   change's business to fix.
2. **`test_a_joint_moves_only_what_is_beyond_it`** is the one test whose
   subject is the joint layer itself: it drives `reach` and asserts the
   coxa and the lift servo stay put while the femur moves more than
   5 mm. If the deep-path binding of gap 4 misbehaves, this is the test
   that catches it, and its failure would mean a real regression, not a
   test that needs changing.

If any assertion goes red that was green in the stage A baseline, the
implementer stops and reports it. Changing a test is the orchestrator's
call.

## Deferred (2026-09-09)

The orchestrator deferred stage B of this refactor. The proposal's "Known
gaps" 1 names the reason: the chassis is one rigid body with four freedoms
against the ground — roll, pitch, yaw and lift — and the framework does not
yet state how several joints on one body compose (or offer a floating/free
joint for a body attitude like this one). Until the framework states that
composition order — recorded as a finding in `solid-node/workflow/warts.md`
— stage B does not proceed, so no joint or relation is declared for the
chassis attitude in this change.

Stage A (imports fixed, baseline captured, poses captured) is committed so
the project runs meanwhile: `solid_node.motion.ports` supplies the port
classes the two importing modules need, with no other line touched, and
the model imports and its 34-test faceted suite passes exactly as it did
before the motion layer moved ports out of `solid_node.node`.

## Undeferred: stage B on `Free` (2026-09-10)

The gap "Known gaps" 1 named is closed. solid-node main `ef379d2` carries
**ADR-093** — the joints of one class compose in DECLARATION order, innermost
first — and **ADR-095**, the `Free` joint: one declaration for a floating
body, six coordinates (`roll`, `pitch`, `yaw`, `x`, `y`, `z`), composed
`T(x,y,z) · Rz(yaw) · Ry(pitch) · Rx(roll)`. That composition is fixed by the
framework, not by binding order, and it is exactly the chain this file's
"What does not change" pinned by hand. ADR-095's own design record and
fixture use this chassis as their worked example and measured the
`_to_chassis` round trip at 1.4e-14 mm.

So the chassis is stated as **one `Free`**, not the four joints this section
asked for:

    class Chassis(AssemblyNode):
        pose = Free(angle_unit='deg', length_unit='mm')

`Spiderbot.simulate()` disappears entirely rather than shrinking to four
statements, and four of the eight relations end on `chassis.pose.roll`,
`.pitch`, `.yaw` and `.z` instead of on a forwarding `SignalPort`, which
takes ten ports out of the model rather than six. `pose.x` and `pose.y` stay
unbound and contribute nothing. Every joint in "The joints to declare" is
unchanged — same axes, same anchors, same units, no `range`.

Known gap 2 (`at` restating the parent's `translate`) is unchanged and still
costs this model two duplicated vectors. Known gap 3 is still not a gap.
Known gap 4's risk did not materialise: the deep-path bindings moved nothing.

Evidence: maximum deviation **0.000e+00** over 28 poses and 172 leaves, and
0.0 over 504 joint-coordinate comparisons; 34/34 faceted green before and
after, no test touched. See `tasks.md` sections 2-5.
