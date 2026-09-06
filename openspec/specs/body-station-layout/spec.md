# body-station-layout Specification

## Purpose
Where the six legs meet the body: which of the frame's holes is a station, which bracket goes on it, which way it faces, and where the coxa axis stands as a result.

## Requirements

### Requirement: Six stations at the frame's own holes

The robot SHALL carry six leg stations, at the bolt holes the frame itself
provides. A station is identified by a **pair** of holes 8.0 mm apart in the
frame's thickness, which is the pair a bracket's own bolt pad carries, and
not by any other hole in the ring.

In the robot's own frame -- x across to the right, y forward, z up, origin at
the ring's centre on its lower face -- the stations' holes SHALL be at the
following positions and headings, mirrored on the left.

| station | hole (x, y) mm | heading |
| --- | --- | --- |
| middle right | (46.75, −0.2) | 0° |
| front right | (39.01, 60.74) | 45° |
| rear right | (38.24, −61.51) | −45° |

A middle station SHALL be built with `FrameCenterServoHolder` and a corner
station with `FrameSideServoHolder`, so the robot holds two centre holders
and four side holders -- which is the count the build guide prints.

#### Scenario: The count is the build guide's
- **WHEN** the model is built at rest
- **THEN** the tree holds exactly two `FrameCenterServoHolder` and four `FrameSideServoHolder`, and one leg at each of the six stations

### Requirement: A station's shaft is vertical, one reach out

At every station the coxa servo SHALL stand with its output shaft vertical,
so that the coxa's yaw axis is the robot's z. Both brackets put that shaft
at the centre of a 48 mm pad span, so the axis SHALL stand the same 27.5 mm
out from the station's hole along its heading, whichever bracket is used.

#### Scenario: All six yaw axes are vertical
- **WHEN** the model is built at rest
- **THEN** the six coxa axes are parallel to the robot's z within 3 arcminutes

#### Scenario: Each axis is one reach out from its hole
- **WHEN** the model is built at rest
- **THEN** each leg's coxa axis stands at its station's hole carried 27.5 mm along that station's heading, within 0.2 mm

### Requirement: The six legs are one leg, placed six times

The six stations SHALL be reached by rotation about the robot's z alone, and
every leg SHALL be built from the same printed parts in the same
arrangement. There is no left leg and no right leg: turning a leg through
180° leaves its horn plate uppermost, which is what a mirror would not do
and what the printed parts require.

The stance SHALL nonetheless be symmetric about the robot's long axis.

#### Scenario: The stance is symmetric
- **WHEN** the robot is standing
- **THEN** each left leg's foot stands at the mirror of its right partner's within 0.5 mm
