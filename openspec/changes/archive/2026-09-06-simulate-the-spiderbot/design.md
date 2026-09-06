# Reading the SpiderBot out of its print files

## The problem this design solves

The repository publishes meshes and photographs. Neither carries a
placement. So every number in `simulation/params.py` had to come from the
meshes themselves, and the whole design question is *which* features in a
triangle soup are trustworthy enough to build a machine on.

The answer used here is: the turned surfaces. A bore, a bearing seat, a
counterbore, a horn circle and a screw hole are all cylinders, and a
cylinder is the one feature a mesh gives up without a heuristic -- its faces
fall into a single smooth group, every normal in that group is perpendicular
to one direction, and that direction, a circle fit across it, and the sign of
the normals give the axis, the centre, the radius and whether it is a hole or
a boss. `simulation/tools/probe.py` is that reader. Nothing in this design
rests on a silhouette, a bounding box or a guess about the designer's intent
where a cylinder could have been asked instead.

## What the cylinders said

Three numbers recur across parts that were drawn independently, and their
recurrence is the evidence that they mean something.

| number | where it appears |
| --- | --- |
| 10.0 mm | the pitch between two holes in every servo flange pad: the frame side holder, the frame centre holder, the coxa's distal fork, the tibia bracket's arms |
| 48.0 / 49.0 mm | the span between the two flange pads of a holder: 48.0 on the side holder and the tibia bracket, 49.0 on the centre holder |
| 48.6 mm | the length of `LegRib`, the spacer bolted between the two sides of the coxa and of the femur |

A pattern of four holes, ten apart in pairs and forty-eight or forty-nine
apart between pairs, is a standard 40 mm-class hobby servo's mounting
flange. The rib that spaces the two plates of a link is the same length. So
one servo stands in every fork, and the fork gap is the servo.

That settles the joint. On the far side of each fork, one plate carries a
disc of eight Ø3.5 holes on a Ø14 circle around a Ø6 centre, with a Ø21.5
recess behind it -- a servo horn, screwed to the plate. The opposite plate
carries a seat on the same axis, and the file names it: `CoxaSide1_625ZZ`,
`FemurSide1_625ZZ`. Horn on one side, bearing on the other, servo between:
that is one joint, and the robot is six copies of three of them.

### The joints, as measured

Coordinates below are local to each part, measured from its bounding-box
minimum, which is the frame a wrapper's placement works in.

| part | feature | at (x, z) | axis |
| --- | --- | --- | --- |
| `CoxaSide2_8holes` | eight-hole horn disc, Ø21.5 recess | (10.75, 10.75) | y |
| `CoxaSide1_625ZZ` | bearing seat | (10.75, 10.75) | y |
| `CoxaSide*` | rib bolts, 8.0 apart | (37.85, 6.75) and (37.85, 14.75) | y |
| `CoxaSide*` | distal fork, flange bolts 10.0 apart | (49.25, ·) and (59.25, ·) | z |
| `FemurSide2_8Holes` | horn disc, proximal | (10.75, 10.75) | y |
| `FemurSide2_8Holes` | horn disc, distal | (90.75, 10.75) | y |
| `FemurSide1_625ZZ` | bearing seats | (10.75, 10.75) and (90.75, 10.75) | y |
| `TibiaTop` | servo arm holes, 10.0 within, 48.0 between | (32.07, 30.82) … (70.58, 61.16) | y |
| `TibiaTop` | shin bolts, 8.0 apart | (4.5, 7.71) | z |
| `TibiaBottomLong` | bracket bolts, 8.0 apart | (1.99, 4.0) and (1.99, 12.0) | x |
| `TibiaBottomLong` | claw pin, Ø2 | (90.28, 4.93) and (90.28, 11.03) | y |

From which the link lengths follow directly, with no fitting:

- **coxa** 43.5 mm, from the horn at x 10.75 to the fork's flange centre at
  x 54.25, the two axes perpendicular.
- **femur** 80.0 mm, from horn to horn at x 10.75 and x 90.75, the two axes
  parallel.
- **tibia** the shin bolted square to the bracket, foot pin 88.3 mm along
  the shin from that joint.

### The six stations

The frame is a closed ring, 110 mm across, 155 mm fore and aft and 16 mm
thick, and it is printed lying down: the 16 mm axis is the robot's vertical.
Its perimeter carries the holes each servo holder bolts through, and their
axes give each station's heading directly.

| station | frame-local hole | axis | heading |
| --- | --- | --- | --- |
| middle | (101.75, 4 and 12, 77.7) and (101.75, 8, 97.6) | x | 0° |
| front | (94.01, 4 and 12, 16.76) | (−0.707, 0, 0.707) | 45° |
| rear | (93.24, 4 and 12, 139.01) | (−0.707, 0, −0.707) | −45° |

Two holes eight apart in the frame's thickness, matching the two holes eight
apart in a holder's bolt pad, is what identifies a station rather than any
other hole in the ring. The middle stations bolt through the centre holder,
whose bolts run along its long axis; the four corner stations bolt through
the side holder, whose bolts run along its length. The README's counts agree
-- two centre holders, four side holders.

## The decisions, and what decided them

**Local is not the file's own frame.** The probe reports every feature from
a part's bounding-box minimum, because that is what a photograph can be
checked against. A placement works in the file's own coordinates. The two
differ by the bounds minimum, and using one for the other put the coxa's
plates eighteen millimetres off their own hubs -- with every joint still
perfectly coaxial, because the error was common to both plates and every
contract that could have caught it was asking about the horns instead.
`params.py` now carries an `ORIGIN_*` constant per part and derives every
hub through it, so the conversion is written once and cannot be forgotten.

**The fork gap is the stack, not the rib.** The rib measures 48.6 mm, and
that was the first answer. But a horn face, a servo case and a 625ZZ on one
axis come to 49.5, and 0.9 mm of a bearing is not a rounding: seat the
bearing at the rib's spacing and it is inside the plate that holds it. So
the stack sets the gap and the rib is held against it, within the 1.2 mm a
printed spacer and its bolt seats are worth. Both numbers are in the record
and a contract holds them together, which is the only way a later
measurement of either can be seen to disagree.

**The fork gap is the servo, not the servo's shaft.** The first reading of
`LegRib` put its 48.6 mm along the joint axis, which would have made the coxa
yaw axis and the femur lift axis parallel and the robot impossible. The
measurement that decided it is the direction of the flange bolts: in the
coxa's distal fork they run along the part's z while the fork gap is along
its y, and a flange bolt is always parallel to the output shaft. So the shaft
is along z, the servo's *length* spans the gap, and the two axes are
perpendicular -- which is what a hexapod needs.

**The joint sits on the fork's centreline.** A servo of this class puts its
output shaft about 10 mm off the centre of its flange span, so strictly the
lift and knee axes sit 10 mm off the leg's mid-plane. Nothing in the meshes
says which way round each servo is fitted, and the two answers differ by
20 mm of lateral leg offset. Rather than pick one and present it as
measured, the model puts each joint on the fork's centreline and says so
here. The consequence is a planar leg; the cost is that a real build's legs
are offset by that amount, which the contracts do not claim otherwise.

**The bearing is the file's, not the fit's.** `CoxaSide1_625ZZ` and
`FemurSide1_625ZZ` are named for a 625ZZ -- Ø16 outside, Ø5 bore, 5 wide.
The probe reads only a Ø13.5 × 1.5 relief at the outer face of each seat,
because the seat itself is a raised collar the circle fit sees edge-on. The
name is better evidence than the relief, so the model fits a 625ZZ and
records the disagreement rather than inventing a Ø13.5 bearing that no
supplier sells.

**Flange span 48.0, not 49.5.** Two independent parts read 48.0 (the side
holder, the tibia bracket) and one reads 49.0 (the centre holder). A
standard MG996R-class servo is 49.5. The model takes 48.0 -- the majority of
the measurements and the tighter of the two -- and treats the centre
holder's extra millimetre as clearance rather than as a different servo.

**Where the solids touch is written down, not tuned away.** A model built by
placing published STLs cannot promise that nothing overlaps. The repository's
own body exports interpenetrate, a bracket bolted flat to a frame has no
rebate in either file, and the tip, claw and foot switch have no measured
seat at all. The alternative to recording that is nudging measured parts off
their measured positions until a contract goes green, which would make the
contract a decoration. So `simulation/seats.py` names every pair that shares
volume and how much, split into the three kinds, and the whole-model contract
is that the inventory is exactly right: a new overlap fails, a changed one
fails, and a fixed one fails too and has to be taken off the list
deliberately.

**Rejected: modelling the fasteners as a stack.** Every joint's screws,
nuts and nut traps are visible in the meshes and could be placed. They were
not, because a screw through two plates and a nut trap proves nothing the
plates' own coaxiality does not already prove, and 24 legs' worth of M3.5
hardware would triple the build time of the model for no contract. The
screws the model does draw are the ones a *measured hole* names and a moving
part depends on: the claw pin and the foot switch.

**Rejected: a `FusionNode` per link.** The two sides of a coxa really are
bolted into one rigid body, and fusing them would make one printed solid of
the pair. They are not fused, because they are two separately printed
pieces, and the whole-model contract `assertNoDisconnectedSolids` is more
useful reading them as the two printed solids they are.

## Posture and gait

The robot is demonstrated from a body pose, not from raw joint angles. The
root carries `height`, `reach`, `stride`, `roll`, `pitch`, `yaw`,
`gait_phase` and `wave`, and each leg's three joints are solved from the foot
position that pose asks for. The solution is the ordinary planar one -- yaw
from the foot's azimuth, then a two-link reach in the leg's own plane -- and
it is written where a reader can check it against the measured link lengths
rather than hidden behind a numerical solver.

**The root's frame is the ground.** The chassis -- body and legs together --
is one child of the root, lifted to `height` and turned by roll, pitch and
yaw, and every leg is solved against exactly that composition. That is what
makes the attitude drivers mean anything: a model whose fixed frame is the
body can only tilt the world, and "rolling the robot does not move its feet"
would be a claim about nothing. Here it is a contract, and it passes to a
millimetre.

**The second link is not a length.** The shin does not continue the tibia's
own x; it hangs down and forward of the knee, and the claw's contact point
comes out 184.4 mm away at 75.3° below that line. Declaring a tibia length
would have been a number to keep in step with a placement, so `leg.py`
computes both the reach and the bend from the placement itself, and the
inverse kinematics takes the bend out of the knee angle. The first version
declared 88.3 mm and stood the robot ten millimetres off its own floor.

**The gait's conditional is a square root.** A tripod gait lifts a foot
while a sine is positive and leaves it down otherwise, and a conditional
cannot be written symbolically -- the drivers are `$t` expressions in the
build and the viewer. `(u + sqrt(u² + e²)) / 2` is `max(u, 0)` smoothly, in
one expression every runtime evaluates, and it is never negative, so a
supporting foot is never driven through the floor. `e` is 0.02 against the
sine's unit scale, which puts a planted foot within a third of a millimetre
of the ground.
