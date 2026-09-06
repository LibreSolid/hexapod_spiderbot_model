# SpiderBot simulation -- the body, which the repository already assembled.
# SPDX-License-Identifier: MIT

"""The body stack, and the six servo holders bolted round it.

There is a finding in this module, and it is the reason the body took an
afternoon and the leg took a week. **The body's STL files are already in one
shared coordinate system.** The frame stands at y 0 to 16; the power
compartment at y −27 to 0, directly under it; the door at y −27 to 0 at the
compartment's back end; the electronics plate through the frame's lower
edge; and the three carapace pieces at y 16 to 67, directly above, their z
ranges overlapping exactly where their tongues and grooves meet. Nothing
about that is a coincidence, and none of it had to be measured: the build
guide exported the body already assembled and nobody said so.

Only the controller plate is exported lying in its own print orientation,
and only the servo holders have to be positioned, because a holder's place
depends on which of the six stations it is at.

So the body is built in the files' own frame, which has y up, and the whole
of it is stood upright once by the robot's root. The leg stations are
published in the same frame -- see `params.STATIONS` -- so the two agree
without a conversion anywhere.
"""

import math

from solid_node.math import cos, sin

from solid_node.node import AssemblyNode

from . import printed, sourced
from .params import (
    BATTERY_H, FRAME_H, STATIONS, STATION_REACH,
)

#: How much of itself the electronics plate shares with the power
#: compartment as published, mm³. The plate's lower millimetre lies inside
#: the compartment's ceiling: the two files were exported without the
#: rebate that would let them meet. The model does not move the plate to
#: hide it -- the layer places what the repository publishes -- so the
#: number is recorded here and held by a contract.
PUBLISHED_SEAT_OVERLAP = 2081.5

#: How far a corner station's bracket stands off the frame hole it bolts
#: through, mm. A fit: the hole is measured, but the chamfered face it sits
#: against is not a plane the probe reads, and this is what clears it.
SIDE_HOLDER_STANDOFF = 4.0

#: Where the middle stations' holder meets the frame, in the files' own x.
#: The centre holder's frame-side face; its 55 mm length runs outward from
#: there, putting the coxa shaft `STATION_REACH` beyond it.
_CENTRE_FACE = 48.0


class Body(AssemblyNode):
    """The frame, everything stacked on it, and the six coxa brackets.

    Built in the published files' own frame: x across, y up, z along the
    robot with the front at negative z. The root turns it a quarter turn to
    stand it up.
    """

    frame = printed.Frame()
    compartment = printed.PowerCompartment()
    door = printed.PowerCompDoor()
    electronics = printed.ElectronicsPlate()
    controller_plate = printed.ControllerPlate()
    carapace_front = printed.CarapaceFront()
    carapace_mid = printed.CarapaceMid()
    carapace_back = printed.CarapaceBack()

    side_holders = printed.FrameSideServoHolder().repeat(4)
    centre_holders = printed.FrameCenterServoHolder().repeat(2)

    battery = sourced.LipoPack()
    board = sourced.ControllerBoard()

    def render(self):
        # Seven of the eight printed body parts need no placement at all:
        # the files already stand in one frame. This is the whole of the
        # body stack.
        for part in (self.frame, self.compartment, self.door,
                     self.electronics, self.carapace_front,
                     self.carapace_mid, self.carapace_back):
            part.translate([0.0, 0.0, 0.0])

        # The controller plate is the exception, exported flat. Laid down
        # and set above the frame, where the build guide's own note puts it
        # -- on the electronics plate or under the middle carapace.
        # Laid down and turned a quarter turn: its 110.78 mm runs along
        # the robot, not across it. Across it the plate would be wider than
        # both the 106 mm carapace and the 109.58 mm compartment, and there
        # would be nowhere in the body it could go.
        (self.controller_plate
         .rotate(-90, [1, 0, 0]).rotate(90, [0, 1, 0])
         .translate([0.0, FRAME_H + 5.0, 0.0]))

        # The battery lies in the compartment, its length along the robot.
        (self.battery
         .rotate(-90, [0, 1, 0]).rotate(-90, [0, 0, 1])
         .translate([0.0, -BATTERY_H - 1.0, 0.0]))

        # The controller sits on the plate that is named for it.
        (self.board
         .rotate(-90, [1, 0, 0]).rotate(90, [0, 1, 0])
         .translate([0.0, FRAME_H + 4.0, 0.0]))

        # The six brackets. A middle station takes the centre holder, whose
        # long axis runs outward; a corner station takes the side holder,
        # whose length runs outward from the frame's 45-degree chamfer. The
        # two share a heading and a reach, so the arithmetic is written once.
        sides = iter(self.side_holders)
        centres = iter(self.centre_holders)
        for _, x, y, heading in STATIONS:
            hole = (x, -y)                       # back into the files' frame
            if abs(heading % 180.0) < 1e-6:
                self._place_centre(next(centres), hole, heading)
            else:
                self._place_side(next(sides), hole, heading)

    def _place_centre(self, holder, hole, heading):
        """A middle station's bracket: its x = 0 face against the frame."""
        holder.translate([_CENTRE_FACE, 0.0, hole[1]
                          - self._centre_offset()]).rotate(0, [0, 1, 0])
        if abs(heading - 180.0) < 1e-6:
            holder.rotate(180, [0, 1, 0])

    @staticmethod
    def _centre_offset():
        """The z shift that lands the holder's two bolt columns on the
        frame's two holes -- 8.8 mm, from holder z 9.0 and 28.9 against
        frame z 0.2 and 20.1.
        """
        return 8.8

    def _place_side(self, holder, hole, heading):
        """A corner station's bracket: its z = 0 face against the chamfer,
        its length running out along the station's heading.
        """
        radians = math.radians(heading)
        (holder
         .translate([-9.5, 0.0, -SIDE_HOLDER_STANDOFF])
         .rotate(90.0 + heading, [0, 1, 0])
         .translate([hole[0], 0.0, hole[1]]))

    @staticmethod
    def station_origin(heading, x, y, z):
        """Where a leg's own frame stands, in the robot's upright frame.

        The station's frame hole, carried `STATION_REACH` outward along its
        heading -- the distance from a holder's mounting face to the coxa
        shaft, which is 27.5 mm on both holders because both put the servo
        at the centre of a 48 mm pad span 3.5 mm in from the face.
        """
        return (x + STATION_REACH * cos(heading),
                y + STATION_REACH * sin(heading),
                z)


#: Where the top of the frame stands, in the upright robot.
FRAME_TOP = FRAME_H
