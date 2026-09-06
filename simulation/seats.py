# SpiderBot simulation -- where the model's solids touch, and by how much.
# SPDX-License-Identifier: MIT

"""The seats: every place two solids in this model share volume.

A model built by placing published STLs cannot promise that no two solids
overlap, and pretending otherwise would mean nudging measured parts off
their measured positions until a contract went green. This module says the
truth instead.

There are three kinds of entry here.

**Published overlaps.** The repository's own body exports interpenetrate:
the electronics plate seats a millimetre into the power compartment's
ceiling, and no placement can clear it -- lift the plate off the compartment
and it goes into the frame. Nothing the model does causes these, and moving
a part to hide one would be a lie about what the build guide publishes.

**Seats the exports have no rebate for.** A servo bracket bolted flat to a
frame, a rib bolted between two plates, a bracket bolted to a shin: in a
real build these meet at faces that are relieved for each other, and the
published solids are not. So a placement that puts the bolt holes where the
measurements say they are also puts a few hundred cubic millimetres of one
part inside another.

**Fits.** The tip, the claw, the switch and its housing have no measured
seat at all -- see `leg.py` -- so where they sit is chosen, and where they
touch is the cost of that choice.

Every one is named and sized. The contract in `test_spiderbot.py` holds each
to the size recorded here and refuses any pair not on the list, so a new
overlap is a failure and a fixed one is a failure too. That is what makes
this a record rather than a tolerance.
"""

#: Every pair of solids that shares volume in the model at rest, by the two
#: parts' tree names, with the volume in mm³ it shares. Read off the built
#: model; see the module docstring for what each kind means.
SEATS = {
    ('compartment', 'electronics'): 2081.5,
    ('centre_holders-1', 'yaw_servo'): 1611.2,
    ('centre_holders-0', 'yaw_servo'): 1491.1,
    ('centre_holders-0', 'frame'): 1344.6,
    ('frame', 'side_holders-1'): 1316.4,
    ('frame', 'side_holders-0'): 974.2,
    ('frame', 'side_holders-2'): 899.1,
    ('electronics', 'side_holders-0'): 896.8,
    ('electronics', 'side_holders-3'): 856.2,
    ('frame', 'side_holders-3'): 735.1,
    ('electronics', 'side_holders-1'): 719.5,
    ('electronics', 'side_holders-2'): 667.1,
    ('bracket', 'servo'): 603.3,
    ('centre_holders-1', 'frame'): 515.7,
    ('side_holders-0', 'side_holders-2'): 318.2,
    ('centre_holders-1', 'rib'): 286.9,
    ('centre_holders-0', 'rib'): 286.8,
    ('shin', 'tip'): 282.8,
    ('side_holders-1', 'side_holders-3'): 269.5,
    ('switch', 'switch_holder'): 260.0,
    ('bearing_side', 'rib'): 133.3,
    ('horn_side', 'rib'): 126.0,
    ('rib', 'yaw_servo'): 83.9,
    ('bearing_side', 'bracket'): 22.6,
    ('carapace_mid', 'yaw_servo'): 13.5,
    ('door', 'electronics'): 12.5,
    ('frame', 'yaw_servo'): 9.7,
    ('carapace_front', 'yaw_servo'): 9.4,
    ('carapace_back', 'yaw_servo'): 9.4,
    ('switch_cover', 'switch_holder'): 4.0,
    ('compartment', 'yaw_servo'): 0.5,
    ('carapace_front', 'carapace_mid'): 0.1,
    ('electronics', 'yaw_servo'): 0.1,
    ('carapace_back', 'carapace_mid'): 0.0,
}


#: How far a seat may drift from its recorded size before the contract
#: calls it a change: a fifth, plus a cubic millimetre for the very
#: small ones, which is wide enough for a tessellation to differ and
#: narrow enough that a moved part is caught.
def tolerance(volume):
    """The band a recorded seat is held in."""
    return max(volume * 0.2, 1.0)
