## Why

This repository is a build guide. It publishes twenty-five STL files, a
photograph of each one, a table saying how many to print and at what infill,
and a photograph of the finished robot. That is everything it says about the
SpiderBot.

What it does not say is where any of those parts goes. An STL carries no
placement, no mating face and no axis, so the repository cannot answer the
questions a builder actually has: which way up the coxa plates go, how far
apart the two sides of a leg stand, where the servo sits between them, which
of the three joints in a leg turns about which axis, how far the leg can
reach before the femur hits the body, whether the six legs can stand the
robot up at all, and whether a tripod gait swings a leg through its own
carapace. The build guide answers none of them, and nothing in the
repository can be wrong about them, because nothing in the repository states
them.

That is what physically goes wrong today. A builder assembling from these
files has to infer every interface from the photographs, and a change to any
printed part -- a longer tibia, a taller carapace, a different servo -- has
no model to be checked against.

## What Changes

- A solid-node project in `simulation/` that assembles the published STLs
  into the moving robot. Every printed part is its own STL (`StlNode`) and
  none is redesigned: the layer places what the repository already
  publishes. The only geometry modelled here is what the repository does not
  publish at all, because it is bought rather than printed -- the servos,
  their horns, the 625ZZ bearings, the LiPo pack, the controller board and
  the foot microswitches.
- Every placement is measured out of the meshes, not guessed.
  `simulation/tools/probe.py` is the instrument: it reads the turned
  surfaces of a part (a bore, a bearing seat, a horn circle, a screw hole)
  and the voids inside it, and `docs/measurements.md` records what it read.
  The constants in `simulation/params.py` are those readings and nothing
  else.
- The leg as three driven joints. The repository's own naming is the
  design: a part named `..._8holes` carries a servo horn, a part named
  `..._625ZZ` carries the bearing on the far side of the same axis, and the
  two stand apart by the length of the servo between them. Coxa yaw, femur
  lift and tibia knee are those three axes, in the frame of the plates that
  carry them.
- A robot steered by the pose of its body rather than by eighteen sliders.
  The root stands in the *ground's* frame: the chassis is lifted and tilted
  over six planted feet, and each leg's three angles are solved from where
  its foot has to be. Instructions demonstrate it -- stand, crouch, tiptoe,
  sit, a wave, a look around on planted feet -- and a tripod gait walks it,
  running off the viewer's own animation time so the model walks with no
  button pressed.
- Contracts that prove the interfaces the build guide leaves open: the fork
  gap is the stack that stands in it, the horn and the bearing are coaxial,
  the link lengths are the measured ones, the six legs reach the ground at
  the stance the drivers claim, attitude moves the body and not the feet,
  and every place two solids share volume is named and sized in a seat
  inventory -- so a new overlap is a failure and so is one that has quietly
  changed.
- Colours for illustration: the printed parts in the robot's own black,
  the leg shields and claws in its red, and each bought part in a colour
  that separates it from the plastic around it.

## Capabilities

### New Capabilities
- `leg-joint-stack`: what one servo joint is -- the fork gap, the horn
  circle, the bearing seat opposite it, and the axis the two define.
- `leg-kinematics`: the three joint axes of a leg in the frame of its
  station, the link lengths between them, the drivers that turn them, and
  the reach they allow.
- `body-station-layout`: the six leg stations on the frame -- where each
  servo holder bolts, which way it faces, and the shaft position that
  follows.
- `body-stack`: the frame, carapace, power compartment, electronics and
  controller plates as one stacked body, and the bought parts inside it.
- `foot-assembly`: the tibia's shin, tip, claw and contact microswitch,
  and where the foot's contact point is.
- `robot-posture`: the ground the robot stands on, the poses and gait it
  demonstrates, and what has
  to stay true through them.

### Modified Capabilities
- (none; this is the project's first design change)

## Impact

- New `pyproject.toml` naming `simulation.spiderbot:Spiderbot`, new package
  `simulation/`, `openspec/` as the design record, `docs/measurements.md`
  as the reading the constants came from, `README.md` gaining a section on
  the simulation, `.gitignore` covering the build directory.
- The published `stl/` files and `media/` photographs are not edited. Where
  the model has to disagree with the build guide it says so in the design
  and in a contract.
- A seat inventory, `simulation/seats.py`, recording every overlap: the
  ones the repository publishes, the ones its exports have no rebate for,
  and the model's own fits.
- Out of scope, recorded for a later change: the printed stand and its leg
  (`Stand.stl`, `StandLeg.stl`), which hold the robot in the air for bench
  testing rather than being part of the robot; the wiring loom and its
  sleeving; the fastener stack, which proves nothing the plates' own
  coaxiality does not; and the servo horn's splines, which no measured hole
  names.
- Uses solid-node 0.6 with the declarative API as published; no framework
  change is needed.
