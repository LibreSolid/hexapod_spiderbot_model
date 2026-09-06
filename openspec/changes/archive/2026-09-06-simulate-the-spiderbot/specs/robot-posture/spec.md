# robot-posture Specification

## ADDED Requirements

### Requirement: The root's frame is the ground

The robot's root SHALL stand in the ground's frame: z = 0 is the floor the
six claws touch. The body and its legs SHALL be one child of that root,
lifted to `height` and turned by roll, pitch and yaw, and each leg SHALL be
solved so its foot stays where the ground put it.

That is what makes the attitude drivers mean anything. A model whose fixed
frame is the body can tilt only the world, and the claim "rolling the robot
does not move its feet" would be vacuous.

#### Scenario: Attitude moves the body and not the feet
- **WHEN** the robot is rolled 15°, then pitched 15°, then yawed 20°
- **THEN** the six contact points stay where they were within 1.0 mm at every step

### Requirement: The body pose drives the legs

The root SHALL declare `height` (mm of body above the floor, 50 to 170),
`reach` (mm from each leg's own yaw axis out to its foot, 130 to 240),
`stride` (mm, 0 to 60), `roll`, `pitch` and `yaw` (degrees, ±20, ±20 and
±25), `gait_phase` (one walking cycle, 0 to 1) and `wave` (0 to 1).

At `height` 110 and `reach` 185 with every other driver at zero, the six
feet SHALL stand on the floor and the body SHALL be level.

#### Scenario: Standing is level and planted
- **WHEN** the robot is at its standing pose
- **THEN** the six contact points lie on one plane within 0.5 mm, on the floor within 3.0 mm

#### Scenario: Crouching keeps the feet down
- **WHEN** `height` is taken to 60 and `reach` to 200
- **THEN** the six contact points are still on one plane within 0.5 mm and still on the floor within 3.0 mm

### Requirement: The instructions demonstrate the robot

The root SHALL declare `Stand`, `Crouch`, `Tiptoe`, `Sit`, `Wave`,
`LookAround` and `Walk`. `Wave` SHALL lift one front leg and swing it while
the other five stand.

#### Scenario: Waving keeps the robot standing
- **WHEN** `wave` is taken to 1
- **THEN** the other five legs' contact points have moved by less than 1.0 mm and the waving leg's claw is at least 20 mm off the floor

### Requirement: A tripod gait walks the robot

`gait_phase` SHALL drive a tripod gait: the front-right, middle-left and
rear-right legs swing while the other three support, and the two sets
exchange at half a cycle. The gait SHALL also run off the viewer's own
animation time, so the model walks without an instruction being triggered,
and SHALL stand still when `stride` is zero.

A supporting foot SHALL stay on the floor and travel backwards; a swinging
foot SHALL leave it, travel forwards and return.

#### Scenario: Three feet are always down
- **WHEN** a cycle is swept in twelve steps at a 40 mm stride
- **THEN** at least three claws are on the floor at every step, and whatever is off it is one tripod's worth -- at the two changeover instants that is nothing at all, and all six are down
