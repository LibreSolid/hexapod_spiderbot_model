# SpiderBot simulation -- the published parts, wrapped and not redrawn.
# SPDX-License-Identifier: MIT

"""One wrapper per file in `stl/`.

This is the whole of the layer's relationship with the repository's own
geometry: name the file, give it a colour, and place it elsewhere. Nothing
here reshapes a published part, and nothing here adds a feature to one. When
the build guide revises an STL, the model follows without an edit.

Each class's docstring says what the part is for and, where the model
depends on it, which measured feature it carries. The measurements
themselves live in `params.py`.
"""

from solid_node.node import StlNode

from .params import BLACK, BLACK_LIGHT, RED


class Published(StlNode):
    """A part exactly as the repository publishes it.

    The subclass supplies `stl_source`; everything else -- the colour and
    the promise that no geometry is touched -- is here so that a reader can
    see at a glance that this module places parts and does not make them.
    """

    color = BLACK


# ------------------------------------------------------------------ body

class Frame(Published):
    """The central ring every other part hangs off.

    110 x 155 in plan and 16 tall, printed lying down, so the file's 16 mm
    axis is the robot's vertical. Its perimeter carries the bolt holes that
    identify the six leg stations.
    """

    stl_source = '../stl/Frame.stl'


class FrameCenterServoHolder(Published):
    """The middle legs' coxa servo bracket, two per robot.

    Its four servo screw holes are 49.0 apart in its length and 10.0 apart
    across it; its frame bolts run along its long axis. This is the one
    published part that is not watertight, which the model admits by name
    rather than repairing.
    """

    stl_source = '../stl/FrameCenterServoHolder.stl'
    require_watertight = False


class FrameSideServoHolder(Published):
    """The four corner legs' coxa servo bracket.

    Its two servo pads stand 48.0 apart along its length with two holes
    10.0 apart across each; its frame bolts run along the same axis, into
    the frame's 45-degree corner chamfer.
    """

    stl_source = '../stl/FrameSideServoHolder.stl'


class CarapaceFront(Published):
    """The front third of the shell."""

    stl_source = '../stl/CarapaceFront.stl'


class CarapaceMid(Published):
    """The middle third of the shell, the tallest of the three."""

    stl_source = '../stl/CarapaceMid.stl'


class CarapaceBack(Published):
    """The back third of the shell."""

    stl_source = '../stl/CarapaceBack.stl'


class PowerCompartment(Published):
    """The battery bay under the frame."""

    stl_source = '../stl/PowerCompartment.stl'
    color = BLACK_LIGHT


class PowerCompDoor(Published):
    """The lid that closes the battery bay."""

    stl_source = '../stl/PowerCompDoor.stl'
    color = RED


class ElectronicsPlate(Published):
    """The tray between the battery bay and the frame."""

    stl_source = '../stl/ElectronicsPlate.stl'
    color = BLACK_LIGHT


class ControllerPlate(Published):
    """The board mount above the electronics tray."""

    stl_source = '../stl/ControllerPlate.stl'
    color = BLACK_LIGHT


# ------------------------------------------------------------------- leg

class CoxaSide1(Published):
    """The coxa's bearing side.

    Carries the 625ZZ seat at local (10.75, 10.75) -- the same place its
    partner carries the horn disc -- and the coxa's distal fork block.
    7.0 mm thick.
    """

    stl_source = '../stl/CoxaSide1_625ZZ.stl'


class CoxaSide2(Published):
    """The coxa's horn side.

    Carries the eight-hole horn disc at local (10.75, 10.75): eight Ø3.5
    holes on a Ø14 circle around a Ø6 bore, with the Ø21.5 horn recess
    behind. 10.0 mm thick.
    """

    stl_source = '../stl/CoxaSide2_8holes.stl'


class FemurSide1(Published):
    """The femur's bearing side: 625ZZ seats at both ends, 80.0 apart."""

    stl_source = '../stl/FemurSide1_625ZZ.stl'


class FemurSide2(Published):
    """The femur's horn side: eight-hole discs at both ends, 80.0 apart."""

    stl_source = '../stl/FemurSide2_8Holes.stl'


class LegRib(Published):
    """The spacer bolted between the two sides of a link.

    48.6 mm long, and that length is the fork gap the servo stands in.
    """

    stl_source = '../stl/LegRib.stl'


class LegShield(Published):
    """The cover over the femur's mechanics -- the robot's red."""

    stl_source = '../stl/LegShield.stl'
    color = RED


class ServoJoint(Published):
    """The adapter that gives a servo a stub to pivot on.

    Bolted to the servo's back, its boss rides in the 625ZZ opposite the
    horn, so the servo is supported at both ends of its own axis.
    """

    stl_source = '../stl/ServoJoint.stl'
    color = BLACK_LIGHT


class TibiaTop(Published):
    """The knee bracket.

    Carries the tibia servo on two arms 48.0 apart, and the shin square to
    it through two bolts 8.0 apart.
    """

    stl_source = '../stl/TibiaTop.stl'


class TibiaBottomLong(Published):
    """The shin, 102 mm of it, ending in the Ø2 claw pin."""

    stl_source = '../stl/TibiaBottomLong.stl'


class Tip(Published):
    """The hard tip the claw hangs from."""

    stl_source = '../stl/Tip.stl'


class Claw(Published):
    """The soft foot.

    The build guide prints this in TPU, and it is the only part of the robot
    that touches the ground.
    """

    stl_source = '../stl/Claw.stl'
    color = RED


class MicroswitchHolderRound(Published):
    """The foot's contact-switch housing."""

    stl_source = '../stl/MicroswitchHolderRound.stl'
    color = BLACK_LIGHT


class SwitchCover(Published):
    """The lid over the foot's contact switch."""

    stl_source = '../stl/SwitchCover.stl'
    color = BLACK_LIGHT


# ----------------------------------------------------------------- bench

class Stand(Published):
    """The bench stand that holds the robot in the air.

    Published for testing, not part of the robot, and not assembled here.
    Wrapped so the model's inventory covers every file the repository
    prints.
    """

    stl_source = '../stl/Stand.stl'
    color = BLACK_LIGHT


class StandLeg(Published):
    """The bench stand's foot. Not part of the robot; see `Stand`."""

    stl_source = '../stl/StandLeg.stl'
    color = BLACK_LIGHT
