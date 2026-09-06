# SpiderBot simulation -- what one servo joint is.
# SPDX-License-Identifier: MIT

"""The stack every joint in this robot is built from.

The robot has three kinds of joint and one construction. A servo stands
between two printed plates. Its horn is screwed into the eight-hole disc of
the plate on the output side. Its opposite end is carried by a 625ZZ seated
in the plate facing it, with `ServoJoint` bolted to the outside of that plate
to give the bearing something to hold. Turn the servo and the two plates
turn together about the line through horn, shaft and bearing.

Everything below is that stack written once, as offsets along the joint axis
from one reference: **the servo's own case bottom**, which is z = 0 in the
servo's frame. A joint anywhere in the robot is then this stack put on an
axis, and the three joints differ only in where the axis is and what is
bolted to the servo's flange.

The offsets are derived, not chosen. The horn disc has to land in the Ø21.5
recess the plate carries, the recess is 4.0 mm deep and opens 6.5 mm into
the plate, and the horn sits on the servo's output boss: that fixes the horn
plate. The bearing is 5.0 mm wide and sits in its seat: that fixes the
bearing plate, once the gap between the plates is known.

The gap is the one number here that is read rather than derived, and it is
read from `LegRib`, the 48.6 mm spacer bolted between the two plates of a
link. See the design record for why the rib is believed over the plates'
own bolt depths, which would put the plates 38 mm apart and leave the rib
with nowhere to go.
"""

from .params import (
    BEARING_W, DISC_RECESS_T, DISC_WEB_T, FORK_GAP,
    HORN_BOSS_H, SERVO_BODY_H, SERVO_FLANGE_T, SERVO_FLANGE_Z,
)

#: Where the servo's output boss ends and the horn begins, mm above the
#: case bottom.
HORN_Z = SERVO_BODY_H

#: Where the horn's disc face sits -- and therefore where the horn plate's
#: recess has to open.
HORN_FACE_Z = HORN_Z + HORN_BOSS_H

#: The horn plate's inner face, mm above the servo's case bottom. The disc
#: region of the plate is `DISC_WEB_T + DISC_RECESS_T` thick and the horn
#: goes into the far `DISC_RECESS_T` of it, so the plate's inner surface is
#: the horn's face.
HORN_PLATE_FACE_Z = HORN_FACE_Z

#: The horn plate's outer face.
HORN_PLATE_BACK_Z = HORN_FACE_Z + DISC_WEB_T + DISC_RECESS_T

#: The bearing's own lower face, and with it the bearing plate's inner one.
#: The bearing is 5.0 mm wide and it is what the servo's opposite end runs
#: in, so it fills the space between the plate and the case: the plate's
#: inner face is one bearing below the case bottom, and nothing floats.
#: The running clearance either side of the bearing, mm. Small, but not
#: zero: a bearing whose faces touch both the plate and the case exactly
#: cannot turn, and an assertion asked about two solids in exact contact is
#: asked a question with no answer.
CLEARANCE = 0.02

BEARING_Z = -(BEARING_W + CLEARANCE)
BEARING_PLATE_FACE_Z = BEARING_Z - CLEARANCE

#: What the stack therefore asks of the fork: horn face to bearing face.
STACK_GAP = HORN_PLATE_FACE_Z - BEARING_PLATE_FACE_Z

#: The middle of the fork, measured from the servo's own case bottom. A
#: servo whose flange bolts across a fork stands centred in it, not on its
#: floor, so this is where the next joint's axis falls.
FORK_MID = (HORN_PLATE_FACE_Z + BEARING_PLATE_FACE_Z) / 2.0

#: How far that is from the rib the plates are actually spaced by, mm.
#: `LegRib` measures 48.6 and the stack wants 49.5 -- 0.9 mm, which is a
#: printed spacer's own tolerance plus its bolt seats, and is the size of
#: the disagreement between what the parts demand and what the one part
#: that sets the distance is. It is recorded rather than absorbed.
RIB_SHORTFALL = STACK_GAP - FORK_GAP

#: How much thinner a plate is at its horn disc than at its block: the
#: coxa's horn plate is 10.0 mm through its block and 6.5 mm through its
#: disc, so a gap measured between the plates' *block* faces is this much
#: wider than one measured at the discs the fork gap is set from.
DISC_GAP_ALLOWANCE = 3.5

#: The flange -- the plate that bolts the servo to whatever holds it -- as a
#: pair of offsets, for the parts that have to sit against it.
FLANGE_FACE_Z = SERVO_FLANGE_Z
FLANGE_BACK_Z = SERVO_FLANGE_Z + SERVO_FLANGE_T
