# leg-kinematics Specification

## Purpose
The three axes of a leg in the frame of its station, the link lengths between them, how the angles reach the leg, and where the foot goes.

## Requirements

### Requirement: A leg is three axes at the measured link lengths

Each leg SHALL take three angles -- coxa yaw, femur lift and tibia knee, in
degrees -- as ports, fed by the robot's root from the body pose it is
steered by. A leg SHALL NOT declare its own drivers: the robot is steered by
where its body stands, not by eighteen sliders.

The axes SHALL stand in the leg's own frame, whose x points away from the
body along the station's heading, whose z is up and whose y completes a
right-handed set. The coxa axis SHALL be that frame's z through its origin.
The lift axis SHALL be parallel to the leg's y, 43.5 mm along the coxa's x
from the coxa axis -- the measured distance from the horn disc at local
x 10.75 to the fork's centre at local x 54.25 -- and at the middle of the
coxa's own fork. The knee axis SHALL be parallel to the lift axis, 80.0 mm
along the femur's x from it, the measured distance between the femur's two
horn discs.

#### Scenario: The link lengths are the measured ones
- **WHEN** the model is built at rest
- **THEN** in every leg the perpendicular distance from the coxa axis to the lift axis is 43.5 mm within 0.05 mm, the distance from the lift axis to the knee axis is 80.0 mm within 0.05 mm, and the coxa axis is square to the lift axis

#### Scenario: Lift and knee are parallel
- **WHEN** the model is built at rest
- **THEN** each leg's lift and knee axes are parallel within 3 arcminutes, so the leg reaches in one plane once its yaw is set

### Requirement: The tibia's reach is read, not declared

The distance from the knee axis to the claw's contact point, and the angle
that vector makes with the tibia's own x, SHALL be computed from the
placement of the shin, the tip and the claw rather than written down. A leg
at zero knee therefore has its foot 75.3° below the line the femur
continues, at 184.4 mm.

#### Scenario: Forward and inverse agree
- **WHEN** the robot is standing
- **THEN** in every leg the claw's contact point stands at the tibia's own computed reach from the knee axis

### Requirement: The foot goes where the pose asks

Each leg's three angles SHALL be solved by the ordinary planar solution: the
yaw from where the foot lies round the station, and the lift and the knee
from a two-link reach at the measured 80.0 mm femur and the tibia's own
computed reach, with the tibia's bend taken out of the knee angle.

#### Scenario: Six feet on one plane
- **WHEN** the robot is standing
- **THEN** the six claws' contact points lie on one horizontal plane within 0.5 mm
