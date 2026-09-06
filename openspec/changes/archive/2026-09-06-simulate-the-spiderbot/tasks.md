# Tasks

## 1. Read the parts

- [x] 1.1 Carry the shop's cylinder and occupancy readers into
      `simulation/tools/probe.py`, pointed at `stl/`.
- [x] 1.2 Print the inventory, the turned surfaces of every part that
      carries a joint, and the voids of the holders and the compartment.
- [x] 1.3 Record the reading in `docs/measurements.md`, and put only what it
      says into `simulation/params.py`.

## 2. The parts, as published

- [x] 2.1 One `StlNode` wrapper per published STL in `simulation/printed.py`,
      each naming its file, its colour and nothing else.
- [x] 2.2 A contract that every wrapper's mesh is watertight, and an
      explicit, named admission for `FrameCenterServoHolder`, which is not.

## 3. The parts the repository does not print

- [x] 3.1 `simulation/sourced.py`: the servo, its horn, the 625ZZ bearing,
      the LiPo pack, the controller board and the microswitch, each exact
      and each dimensioned from what the measured seats demand. The claw pin
      was drawn and then removed: see the design record.
- [x] 3.2 A failing contract that the servo's flange spans the fork gap and
      its four holes fall on the holder's four measured holes; then the
      servo geometry that satisfies it.

## 4. One joint

- [x] 4.1 A failing contract that horn, bearing and shaft are coaxial and
      that the fork gap is 48.6 mm; then `simulation/joint.py` that builds
      one joint and satisfies it.
- [x] 4.2 A failing contract that turning the driver moves only the distal
      side; then the `simulate()` that satisfies it.

## 5. One leg

- [x] 5.1 Failing contracts for the link lengths, the axis directions and
      the forward/inverse agreement; then `simulation/leg.py`.
- [x] 5.2 The foot: shin, tip, claw, switch holder and cover, with the
      contact point contract.
- [x] 5.3 The travel contracts: no shared volume at the range corners, no
      leg into the body.

## 6. The body

- [x] 6.1 Failing contracts for the six stations' positions and headings and
      for the holder counts; then `simulation/body.py`.
- [x] 6.2 The stack: compartment, door, electronics plate, controller plate,
      three carapace pieces, with the closing and clearance contracts.
- [x] 6.3 The bought parts inside it, with the enclosure contracts.

## 7. The robot

- [x] 7.1 Failing contracts for the standing pose and for attitude holding
      the feet; then the pose solution on the root.
- [x] 7.2 The instructions, with a scenario test that each lands and none
      collides.
- [x] 7.3 The tripod gait, with the three-feet-down and no-collision
      scenario tests.

## 8. Close

- [x] 8.1 Whole-model contracts: no disconnected solids, and every place
      two solids share volume named and sized in `simulation/seats.py`.
- [x] 8.2 Snapshots of the robot standing, from the front, the side and
      above, checked by eye against `media/robot.png`.
- [x] 8.3 `README.md` gains the simulation section; `.gitignore` covers
      `_build/`; archive the change.
