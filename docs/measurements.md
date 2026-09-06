# Reading the SpiderBot out of its print files

Every number in `simulation/params.py` came from one of the readings below.
`simulation/tools/probe.py` takes them; `docs/measurements-raw.txt` is its
output, verbatim, and this file is what it means.

Run `python -m simulation.tools.probe` to reprint it.

## Coordinates

The probe reports **local** coordinates, measured from each part's own
bounding-box minimum, because that is the frame a photograph can be checked
against. A placement works in the file's own frame, and `params.py` carries
the offset between them for each part — the `ORIGIN_*` constants. Getting
that conversion wrong is not a small error: it moved the coxa's plates
eighteen millimetres off their own hubs before it was caught.

## The finding that saved the body

**The body's STL files are already in one shared coordinate system.**

| part | y (up) | z (along the robot) |
| --- | --- | --- |
| `PowerCompartment` | −27 … 0 | −77.4 … 77.5 |
| `Frame` | 0 … 16 | −77.5 … 77.5 |
| `ElectronicsPlate` | −1 … 4.5 | −77.5 … 77.7 |
| `CarapaceFront` | 16 … 67 | −101.5 … −31.4 |
| `CarapaceMid` | 16 … 67 | −66.8 … 56.8 |
| `CarapaceBack` | 16 … 67 | 31.4 … 89.9 |

The compartment ends exactly where the frame begins; the carapace begins
exactly where the frame ends; the three carapace pieces overlap in z
precisely where their tongues and grooves meet. That is an assembly, not a
coincidence, and the build guide never says so. Only `ControllerPlate` is
exported in its own print orientation, and only the six servo brackets have
to be positioned, because where a bracket goes depends on which station it
is at.

The same files also **interpenetrate**: the electronics plate's lower
millimetre is inside the compartment's ceiling, 2081 mm³ of it. Lift the
plate clear of the compartment and it goes into the frame instead. There is
no placement that clears both, so the model leaves it where it is published
and `simulation/seats.py` records it.

## The three numbers that identify a joint

| number | where it appears |
| --- | --- |
| 10.0 mm | the pitch between the two holes of a servo flange pad: `FrameSideServoHolder`, `FrameCenterServoHolder`, the coxa's distal fork, `TibiaTop`'s arms |
| 48.0 / 49.0 mm | the span between two such pads: 48.0 on the side holder and the tibia bracket, 49.0 on the centre holder |
| 48.6 mm | the length of `LegRib`, the spacer bolted between the two plates of a link |

Four holes, ten apart in pairs and forty-eight or forty-nine apart between
pairs, is a 40 mm-class hobby servo's mounting flange. The rib that spaces
the two plates of a link is that length too. So one servo stands in every
fork, and the fork gap is the servo.

## The joints, as read

Local coordinates, from each part's bounding-box minimum.

| part | feature | at (x, z) | axis |
| --- | --- | --- | --- |
| `CoxaSide2_8holes` | eight-hole horn disc, Ø21.5 × 4 recess behind a 2.5 mm web | (10.75, 10.75) | y |
| `CoxaSide1_625ZZ` | bearing seat, Ø13.5 × 1.5 relief at the face | (10.75, 10.75) | y |
| `CoxaSide*` | rib bolts, 8.0 mm apart | (37.85, 6.75) and (37.85, 14.75) | y |
| `CoxaSide*` | distal fork bolts, 10.0 mm apart, nut trap between | (49.25, ·) and (59.25, ·) | z |
| `FemurSide2_8Holes` | horn discs, both ends | (10.75, 10.75) and (90.75, 10.75) | y |
| `FemurSide1_625ZZ` | bearing seats, both ends | (10.75, 10.75) and (90.75, 10.75) | y |
| `FemurSide*` | rib bolts | (56.75, 6.75) and (56.75, 14.75) | y |
| `TibiaTop` | servo arm holes, 10.0 within an arm, 48.0 between arms | (32.07, 30.82) … (70.58, 61.16) | y |
| `TibiaTop` | shin bolts, 8.0 mm apart | (4.5, 7.71) | z |
| `TibiaBottomLong` | bracket bolts, 8.0 mm apart | (1.99, 4.0) and (1.99, 12.0) | x |
| `TibiaBottomLong` | claw pin, Ø2 | (90.28, 4.93) and (90.28, 11.03) | y |
| `LegRib` | end bolts, 8.0 mm apart, nut trap in each | (·, 4.0) and (·, 12.0) | x |

The link lengths follow with no fitting at all:

- **coxa** 43.5 mm, horn at x 10.75 to fork centre at x 54.25, the two axes
  perpendicular.
- **femur** 80.0 mm, horn to horn at x 10.75 and x 90.75, the two axes
  parallel.
- **tibia** not a length along its own x: the shin hangs down and forward of
  the knee, and the claw's contact point comes out 184.4 mm away at 75.3°
  below the tibia's x. `simulation/leg.py` computes both from the placement
  rather than declaring them, so they cannot drift apart.

## The six stations

The frame is a closed ring, 110 × 155 in plan and 16 tall, printed lying
down: its 16 mm axis is the robot's vertical. Its perimeter carries the
holes each bracket bolts through, and a station is identified by a **pair**
of holes 8.0 mm apart in the frame's thickness — which is the pair a
bracket's own bolt pad carries — not by any other hole in the ring.

| station | frame-local holes | axis | heading |
| --- | --- | --- | --- |
| middle | (101.75, 4 and 12, 77.7) and (101.75, 8, 97.6) | x | 0° |
| front | (94.01, 4 and 12, 16.76) | (−0.707, 0, 0.707) | 45° |
| rear | (93.24, 4 and 12, 139.01) | (−0.707, 0, −0.707) | −45° |

The middle stations take `FrameCenterServoHolder`, whose frame bolts run
along its long axis; the four corners take `FrameSideServoHolder`, whose
bolts run along its length into the 45° chamfer. Two and four is exactly
what the build guide's own table says to print.

Both brackets put the servo shaft at the centre of a 48 mm pad span, so both
stand the coxa axis the same 27.5 mm out from their mounting face. That one
number is `STATION_REACH`, and it is why the six legs are one leg placed six
times.

## What the probe could not settle

- **The bearing.** `CoxaSide1_625ZZ` and `FemurSide1_625ZZ` are named for a
  625ZZ — Ø16 outside, Ø5 bore, 5 wide. The probe reads only a Ø13.5 × 1.5
  relief at the face of each seat, because the seat is a raised collar the
  circle fit sees edge-on. The file name is the better evidence.
- **The servo's shaft offset.** A servo of this class puts its output shaft
  about 10 mm off the centre of its flange span, so strictly the lift and
  knee axes sit 10 mm off the leg's mid-plane. Nothing in the meshes says
  which way round each of the eighteen servos is fitted, and the two answers
  differ by 20 mm of leg offset. The model draws the shaft on the flange
  span's centre and says so.
- **The tip and the claw.** `Tip` is a fork with one bore through both of
  its bars and a ball at one end; `Claw` has one bore near its top. Which
  way round they go on the shin's pin, and how far the tip stands off it,
  the files do not say. Those placements are fits, marked as such in
  `simulation/leg.py`.
- **The fastener stack.** Every joint's screws, nuts and nut traps are
  visible and were not placed. A screw through two plates proves nothing the
  plates' own coaxiality does not, and eighteen joints' worth of M3.5
  hardware would triple the model for no contract.
- **The rib against the stack.** `LegRib` measures 48.6 mm; a horn, a servo
  and a 625ZZ on one axis come to 49.5 mm. The 0.9 mm difference is a
  printed spacer's own tolerance, and `test_the_rib_agrees_with_the_stack`
  holds it there.
