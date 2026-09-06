# leg-joint-stack Specification

## ADDED Requirements

### Requirement: A joint is a servo standing in a fork

Every driven joint in a leg SHALL be built as one fork: two printed plates
held apart along the joint axis, one servo standing between them, a servo
horn screwed into the eight-hole disc of the plate on the output side, and a
625ZZ bearing on the same axis in the plate opposite, carrying the servo's
other end.

The stack SHALL be built from the parts and not from a gap: the horn's disc
face sits in the Ø21.5 recess its plate carries, and the bearing fills the
space between the plate opposite and the servo's case, so the fork gap is
what those parts come to -- 49.5 mm -- rather than a number chosen for it.

The joint axis SHALL pass through the centre of the horn disc, through the
centre of the bearing opposite it, and through the servo's output shaft, all
three within 0.2 mm.

#### Scenario: The fork gap is the stack in it
- **WHEN** the model is built at rest
- **THEN** in every leg the coxa's two plates stand apart, measured at the faces the horn and the bearing sit against, by what the stack asks, within 0.6 mm

#### Scenario: Horn, bearing and shaft are one axis
- **WHEN** the model is built at rest
- **THEN** for each of the eighteen joints the centre of the horn, the centre of the bearing and the servo's output shaft lie on one line within 0.2 mm, and that line is the axis the joint turns about

### Requirement: The measured rib agrees with the stack

`LegRib`, the spacer bolted between the two plates of a link, measures
48.6 mm. The stack the same fork has to hold measures 49.5 mm. The model
SHALL record both and SHALL hold the difference below 1.2 mm, which is a
printed spacer's own tolerance plus its bolt seats.

#### Scenario: The two numbers stay together
- **WHEN** the model is loaded
- **THEN** the difference between the rib's measured length and the stack's own height is less than 1.2 mm, and the rib's recorded length is still 48.6 mm

### Requirement: A driven joint turns only its own side of the fork

Turning a joint SHALL move the distal plate pair, everything bolted to it
and everything beyond it, and SHALL leave the proximal plate pair and the
servo body standing in it where they were.

#### Scenario: The servo body stays with the link that holds it
- **WHEN** a leg's reach is changed so its femur lifts
- **THEN** that leg's coxa plates and the femur servo's body have not moved, and the femur's plates have

### Requirement: The bearing that the file names is the bearing fitted

Each plate whose file name carries `625ZZ` SHALL be fitted with a 625ZZ deep
groove bearing -- 16.0 mm outside diameter, 5.0 mm bore, 5.0 mm wide -- on
the joint axis of that seat, with a running clearance either side.

The Ø13.5 × 1.5 relief the meshes actually read at each seat SHALL be
recorded as the disagreement it is, and not treated as the bearing's size.

#### Scenario: Every named seat carries one
- **WHEN** the model is built at rest
- **THEN** the model holds one 625ZZ for each `625ZZ`-named seat in the tree: one per coxa and two per femur, eighteen in all, each 16.0 mm across
