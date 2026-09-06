# foot-assembly Specification

## ADDED Requirements

### Requirement: The foot is the shin, the tip, the claw and the switch

Each leg SHALL end in `TibiaBottomLong` bolted to `TibiaTop` through the two
holes 8.0 mm apart that both parts carry, `Tip` and `Claw` hanging on the
shin's Ø2 pin, and `MicroswitchHolderRound` with `SwitchCover` carrying one
microswitch on the shin above them.

The shin and the bracket SHALL meet face to face at the bracket's own end
and SHALL NOT pass into one another: a bracket 16 mm thick cannot tongue
into a shin 9 mm thick, so they butt.

#### Scenario: The shin meets the bracket
- **WHEN** the model is built at rest
- **THEN** the shin and the bracket share no volume and stand no more than 3.0 mm apart

### Requirement: The claw is the part that touches the ground

`Claw` SHALL be coloured and identified as the soft part -- the build guide
prints it in TPU -- and SHALL be the lowest part of every leg at every pose
the robot demonstrates.

Because the claw is a rigid foot that tilts with the tibia, its lowest point
moves a little across the pose range; the contact point SHALL stay within
3.0 mm of the stance plane at any standing pose and SHALL never go below it
by more than 2.5 mm through a walking cycle.

#### Scenario: Only claws touch down
- **WHEN** the robot is standing
- **THEN** the six claws are on the stance plane and no other part of any leg comes within 5.0 mm of it

#### Scenario: Nothing is driven into the floor
- **WHEN** a walking cycle is swept in twelve steps at a 40 mm stride
- **THEN** no claw goes more than 2.5 mm below the stance plane at any step

### Requirement: Where the foot's parts sit is recorded as a fit

`Tip` is a fork with one bore through both of its bars and a ball at one
end; `Claw` has one bore near its top. Which way round they go on the pin,
and how far the tip stands off it, the published files do not say. Those
placements SHALL be marked in the source as fits, and the volume they share
with the parts around them SHALL be recorded in the model's seat inventory
rather than hidden.

#### Scenario: The fits are on the record
- **WHEN** the model is built at rest
- **THEN** every place the tip, the claw, the switch or its housing shares volume with another solid is named in the seat inventory at the size it is
