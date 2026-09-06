# body-stack Specification

## Purpose
The frame and everything stacked on it, the bought parts inside it, and the record of where the published solids touch each other.

## Requirements

### Requirement: The body is the assembly the files already are

The body's published STL files stand in one shared coordinate system: the
power compartment ends where the frame begins, the carapace begins where the
frame ends, and the three carapace pieces overlap in the robot's length
exactly where their tongues and grooves meet. The model SHALL place seven of
the eight printed body parts at that published position and SHALL NOT move
them.

Only `ControllerPlate`, which is exported in its own print orientation,
SHALL be positioned -- laid down and turned a quarter turn, because across
the robot it is wider than both the carapace and the compartment and there
would be nowhere in the body for it to go.

#### Scenario: The stack closes
- **WHEN** the model is built at rest
- **THEN** the three carapace pieces meet end to end along the robot with no gap, and each stands on the frame's upper face within 0.5 mm

#### Scenario: The published stack is not moved
- **WHEN** the model is built at rest
- **THEN** the frame, the compartment, the door, the electronics plate and the three carapace pieces stand exactly where their files put them

### Requirement: The published body files overlap where the plate seats

As exported, the electronics plate's lower millimetre lies inside the power
compartment's ceiling: the two files were drawn without the rebate that
would let them meet, and no placement can fix it -- lift the plate clear of
the compartment and it goes into the frame.

The model SHALL leave the plate where it is published and SHALL record the
overlap at its measured size, so that a revised export which fixes it is
noticed rather than absorbed.

#### Scenario: The overlap is held to its size
- **WHEN** the model is built at rest
- **THEN** the compartment and the electronics plate share 2081 mm³, within 100 mm³

### Requirement: The bought parts are inside the body they need

The body SHALL carry the parts the build guide names but does not print: a
2S LiPo pack inside the power compartment, a controller board on the
controller plate, and one servo at each of the six coxa stations.

#### Scenario: The battery fits the bay
- **WHEN** the model is built at rest
- **THEN** the pack's extent lies inside the compartment's on every axis

#### Scenario: The controller is carried
- **WHEN** the model is built at rest
- **THEN** the controller board rests on the controller plate's upper face within 0.5 mm

### Requirement: Where solids touch is written down

A model built by placing published STLs cannot promise that no two solids
share volume: the exports carry no rebates for the faces they meet at, and
several of the model's own placements are fits. The model SHALL therefore
carry a **seat inventory** naming every pair of solids that shares volume
and how much, and the contract SHALL be that the inventory is exactly right
-- a new overlap fails, and so does one that has changed size.

The inventory SHALL distinguish three kinds: overlaps the repository
publishes, seats the exports have no rebate for, and the model's own fits.

#### Scenario: Nothing overlaps that is not written down
- **WHEN** every pair of the model's solids is compared at rest
- **THEN** each pair that shares volume is named in the inventory, and shares what the inventory says within a fifth
