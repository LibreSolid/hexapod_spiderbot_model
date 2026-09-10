# SpiderBot simulation -- one leg, and the three joints in it.
# SPDX-License-Identifier: MIT

"""A leg: coxa, femur, tibia, and the servo standing in each fork.

The leg is written as three nested assemblies, one per rigid body, each
carrying the `Revolute` that turns it: the coxa's `yaw`, the femur's `lift`
and the tibia's `knee`. Each declares its axis and its anchor in its
parent's frame, so where the line is is stated rather than arranged, and the
framework -- not this file -- is what gets the body onto it.

Each body's own origin still sits on that axis, which is why every anchor
here is either the frame origin or the same vector the parent's `render()`
translates by.

The leg's own frame has x pointing away from the body along the station's
heading, z up, and y to the left. At every angle of zero the coxa points
straight out, the femur continues out and level, and the tibia continues
straight on -- a leg stretched out flat, which is not a pose the robot ever
stands in but is the one an assertion can state without ambiguity.

Every offset in here is either a measurement from `params.py` or a stack
offset from `joint.py`, except six placements that the meshes do not pin --
the leg shield, the tip, the claw, the switch housing and its cover. Those
are fits, marked `# fit`, and each is checked by eye against the
photograph the repository publishes of the part.
"""

import math

from solid_node.node import AssemblyNode
from solid_node.motion.joints import Revolute

from . import joint, printed, sourced
from .params import (
    COXA_HUB, COXA_LENGTH, COXA_RIB_X, COXA_SIDE1_T, FEMUR_HUB_DISTAL,
    FEMUR_LENGTH, FEMUR_RIB_X, FEMUR_SIDE1_T, RIB_BOLT_Z, SHIN_PIN_X,
    SHIN_PIN_Z, TIBIA_KNEE,
)

#: How thick a plate is at its horn disc: the 2.5 mm web the eight screws
#: pass through plus the 4.0 mm recess behind it. Both the coxa's and the
#: femur's horn plates read 6.5 mm here, whatever the block behind them is.
DISC_FACE_T = 6.5

#: The coxa plate's hub, spelled out because every coxa placement starts by
#: bringing it to the origin.
_CX, _CZ = COXA_HUB

#: The femur is placed on its *distal* hub and then turned end for end, so
#: the part's own x runs back along the leg. The two hubs are 80.0 apart and
#: the plate is symmetric between them, so this costs nothing and saves a
#: mirror, which a rotation cannot be.
_FX, _FZ = FEMUR_HUB_DISTAL

#: `TibiaTop`'s knee, likewise, with the bracket's own mid-thickness for
#: the third coordinate: the knee axis runs through the middle of its 16 mm.
_TX, _TZ = TIBIA_KNEE
_TY = 4.5


#: Where the shin's own x = 0 end face sits along the bracket's z. The two
#: parts are bolted end to end through the pair of holes 8.0 mm apart that
#: both carry, and a bracket 16 mm thick cannot tongue into a shin 9 mm
#: thick, so they meet face to face at the bracket's own end -- which is
#: 9.7 mm past where the bolt holes alone would put them.
SHIN_SEAT = -60.95

#: The other two components of that seat: what lands the shin's own bolt
#: pair on the bracket's, across the bolt rather than along it.
SHIN_ACROSS = (5.83, 10.1)


def shin_point(x, y, z):
    """One point of `TibiaBottomLong`, in the tibia's own frame.

    The shin's placement is two rotations and two translations, and the foot
    has to be put where the shin's far end actually ends up rather than
    where a length says it should. So the transform is written once, here,
    and applied to a point.

    The rotations are `rotate(90, y)` then `rotate(90, z)`, which take the
    shin's x onto the bracket's -z, its y onto the bracket's -x and its z
    onto the bracket's y -- the mapping the two parts' own bolt holes force:
    the bolt runs along the shin's x and along the bracket's z, and the pair
    8.0 mm apart is spread along the shin's z and along the bracket's y.
    """
    px, py, pz = -y, z, -x
    px = px + SHIN_ACROSS[0]
    py = py + SHIN_ACROSS[1]
    pz = pz + SHIN_SEAT
    px, py, pz = px - _TX, py - _TY, pz - _TZ
    return (-px, -py, pz)


#: The claw's contact point, in the tibia's own frame: the shin's Ø2 pin,
#: carried down the shin by the tip and the claw hanging on it. Reading it
#: off the transform rather than declaring a length is what keeps the
#: inverse kinematics honest -- the shin is not a link along the tibia's x,
#: it is a link down and forward of it, and the numbers say so.
#: Where the shin's Ø2 pin stands, in the tibia's own frame.
PIN = shin_point(SHIN_PIN_X, 4.5, SHIN_PIN_Z[0] + 3.05)

#: How far below the pin the claw's own bore hangs, mm. The tip's two
#: measured bores are 21.17 mm apart, and hanging the claw on the lower of
#: them would put the claw's own shoulder through the tip's fork. So the
#: claw is hung where its top face meets the tip's bottom -- a fit, and the
#: one place the model puts a part where a measured hole is not.
TIP_SPAN = 28.95

#: How far the claw hangs below the bore it swings on: 18.91 mm, from the
#: claw's own bore to its lowest face.
CLAW_DROP = 18.91

FOOT = (PIN[0], PIN[1], PIN[2] - TIP_SPAN - CLAW_DROP)

#: How far the foot stands from the knee axis, mm, and at what angle from
#: the tibia's own x. The knee driver reads zero when the tibia continues
#: the femur, so a leg at zero has its foot `TIBIA_BEND` below that line.
TIBIA_REACH = math.hypot(FOOT[0], FOOT[2])
TIBIA_BEND = math.degrees(math.atan2(FOOT[2], FOOT[0]))


class Tibia(AssemblyNode):
    """Everything beyond the knee: the bracket, the shin and the foot.

    Its origin is the knee axis, its y is that axis, and at rest its x
    continues the femur outward.

    The knee servo lives here rather than in the femur, because it is bolted
    to `TibiaTop`'s two arms and travels with them; the horn it drives is
    screwed to the femur's distal disc, on the other side of the joint. That
    is the opposite hand from the other two joints, and it is what the
    printed parts say.
    """

    #: The knee: the tibia turns about the knee servo's shaft, which stands
    #: `FEMUR_LENGTH` out along the femur's own x (where `Femur.render()`
    #: places this node) and runs along the leg's y. The axis is the leg's
    #: *negated* y because a positive knee raises the shin and a positive
    #: turn about +y lowers it -- the sign convention, stated once here
    #: instead of negated at every bind site.
    knee = Revolute(axis=(0, -1, 0), at=(FEMUR_LENGTH, 0.0, 0.0), unit='deg')

    bracket = printed.TibiaTop()
    servo = sourced.Servo()
    adapter = printed.ServoJoint()
    shin = printed.TibiaBottomLong()
    tip = printed.Tip()
    claw = printed.Claw()
    switch_holder = printed.MicroswitchHolderRound()
    switch_cover = printed.SwitchCover()
    switch = sourced.Microswitch()

    def render(self):
        # The bracket, brought to its knee and laid into the leg's plane.
        # The half turn about z is what sends the shin outward and down
        # rather than back under the femur.
        self.bracket.translate([-_TX, -_TY, -_TZ]).rotate(180, [0, 0, 1])

        # The knee servo: shaft along the leg's y, case bottom on the joint
        # origin, bolted flat to the bracket's two arms.
        (self.servo
         .rotate(-90, [0, 1, 0]).rotate(-90, [0, 0, 1]))
        (self.adapter
         .translate([-9.0, -10.0, -30.6])
         .rotate(90, [1, 0, 0])
         .translate([0.0, joint.BEARING_PLATE_FACE_Z - FEMUR_SIDE1_T - 6.6,
                     0.0]))

        # The shin, bolted square to the bracket through the two holes
        # 8.0 mm apart that both parts carry.
        (self.shin
         .rotate(90, [0, 1, 0]).rotate(90, [0, 0, 1])
         .translate([SHIN_ACROSS[0], SHIN_ACROSS[1], SHIN_SEAT])
         .translate([-_TX, -_TY, -_TZ]).rotate(180, [0, 0, 1]))

        # The foot. The shin's Ø2 pin is measured; how far the tip hangs
        # below it, and which way round the tip and the claw sit on it, are
        # fits -- the published tip is a fork with one bore and the file
        # says nothing about which way it points.
        (self.tip
         .translate([0.0, 0.0, -1.43]).rotate(180, [1, 0, 0])
         .translate(list(PIN)))                                  # fit
        (self.claw
         .translate([0.0, 0.0, 3.19])
         .translate([PIN[0], PIN[1], PIN[2] - TIP_SPAN]))        # fit

        # The contact switch, in its round housing on the shin above the
        # foot, with its cover over it and the switch between them. The
        # housing and the cover are published in one frame -- the cover's
        # 1.5 mm plate caps the housing's own face -- so one placement
        # carries both. Where on the shin it sits is a fit.
        for part in (self.switch_holder, self.switch_cover):
            part.rotate(180, [1, 0, 0]).translate(
                [FOOT[0] + 22.0, 0.0, -95.0])                    # fit
        self.switch.rotate(180, [1, 0, 0]).translate(
            [FOOT[0] + 14.0, 0.0, -103.0])                       # fit


class Femur(AssemblyNode):
    """The femur: two plates 80.0 mm between their hubs, and the tibia.

    Its origin is the lift axis and its y is that axis. The plates carry a
    horn disc and a bearing seat at *both* ends, so the same pair serves the
    lift joint at one end and the knee joint at the other.
    """

    #: The lift: the femur turns about the lift servo's shaft, which stands
    #: `COXA_LENGTH` out and `FORK_MID` up in the coxa's frame (where
    #: `Coxa.render()` places this node) and runs along the leg's y. The
    #: axis is negated for the same reason the knee's is.
    lift = Revolute(axis=(0, -1, 0),
                    at=(COXA_LENGTH, 0.0, joint.FORK_MID), unit='deg')

    horn_side = printed.FemurSide2()
    bearing_side = printed.FemurSide1()
    rib = printed.LegRib()
    shield = printed.LegShield()

    lift_horn = sourced.ServoHorn()
    lift_bearing = sourced.Bearing625ZZ()
    knee_horn = sourced.ServoHorn()
    knee_bearing = sourced.Bearing625ZZ()

    tibia = Tibia()

    def render(self):
        # Both plates are placed on their distal hub and turned end for end,
        # so the part's x runs back along the leg and the femur's proximal
        # hub lands on the origin.
        (self.horn_side
         .translate([-_FX, 0.0, -_FZ]).rotate(180, [0, 0, 1])
         .translate([0.0, joint.HORN_PLATE_FACE_Z + DISC_FACE_T, 0.0]))
        (self.bearing_side
         .translate([-_FX, 0.0, -_FZ]).rotate(180, [0, 0, 1])
         .translate([0.0, joint.BEARING_PLATE_FACE_Z, 0.0]))

        # The rib spaces the two plates, bolted through the pair of holes
        # 8.0 mm apart that both carry at local x 56.75.
        (self.rib
         .rotate(180, [1, 1, 0])
         .translate([_FX - FEMUR_RIB_X + RIB_BOLT_Z[0],
                     joint.BEARING_PLATE_FACE_Z, -8.0]))

        # The shield covers the femur's outer face, its 116 mm running the
        # length of the 101.5 mm plate. Where along it, and how far out, is
        # a fit.
        self.shield.translate(
            [-10.0, joint.HORN_PLATE_FACE_Z + DISC_FACE_T + 0.5,
             20.0])                                              # fit

        # The two joints this link carries. The lift horn and its bearing
        # are screwed to the proximal hubs; the knee horn and its bearing to
        # the distal ones, 80.0 mm out.
        self.lift_horn.rotate(-90, [1, 0, 0]).translate(
            [0.0, joint.HORN_Z, 0.0])
        self.lift_bearing.rotate(-90, [1, 0, 0]).translate(
            [0.0, joint.BEARING_Z, 0.0])
        self.knee_horn.rotate(-90, [1, 0, 0]).translate(
            [FEMUR_LENGTH, joint.HORN_Z, 0.0])
        self.knee_bearing.rotate(-90, [1, 0, 0]).translate(
            [FEMUR_LENGTH, joint.BEARING_Z, 0.0])

        self.tibia.translate([FEMUR_LENGTH, 0.0, 0.0])


class Coxa(AssemblyNode):
    """The coxa: the yaw output, and the fork the lift servo stands in.

    Its origin is the yaw axis and its z is that axis, pointing up. The two
    plates lie above and below the body's frame, 48.6 mm apart, and their
    far ends are the fork that holds the femur's servo.
    """

    #: The leg's yaw: the coxa turns about the yaw servo's shaft, which is
    #: the leg frame's own z through its origin. `Leg.render()` places
    #: nothing but the servo, so this node's placed origin is the leg
    #: frame's origin and `at` defaults correctly.
    yaw = Revolute(axis=(0, 0, 1), unit='deg')

    horn_side = printed.CoxaSide2()
    bearing_side = printed.CoxaSide1()
    rib = printed.LegRib()

    yaw_horn = sourced.ServoHorn()
    yaw_bearing = sourced.Bearing625ZZ()
    yaw_adapter = printed.ServoJoint()

    lift_servo = sourced.Servo()
    lift_adapter = printed.ServoJoint()

    femur = Femur()

    def render(self):
        # The horn plate above, the bearing plate below, both lying flat.
        (self.horn_side
         .translate([-_CX, 0.0, -_CZ]).rotate(-90, [1, 0, 0])
         .translate([0.0, 0.0, joint.HORN_PLATE_FACE_Z + 6.5]))
        (self.bearing_side
         .translate([-_CX, 0.0, -_CZ]).rotate(-90, [1, 0, 0])
         .translate([0.0, 0.0, joint.BEARING_PLATE_FACE_Z]))

        # The rib between them, at the pair of holes both plates carry at
        # local x 37.85.
        (self.rib
         .rotate(-90, [0, 1, 0]).rotate(-90, [0, 0, 1])
         .translate([COXA_RIB_X - _CX - 4.0, 8.0,
                     joint.BEARING_PLATE_FACE_Z - 1.5]))

        # The yaw joint's own hardware, screwed to these plates: the horn
        # above, the bearing below, and the adapter outside the bearing
        # plate whose boss reaches through it to the servo.
        self.yaw_horn.translate([0.0, 0.0, joint.HORN_Z])
        self.yaw_bearing.translate([0.0, 0.0, joint.BEARING_Z])
        (self.yaw_adapter
         .translate([-9.0, -10.0, -30.6])
         .translate([0.0, 0.0, joint.BEARING_PLATE_FACE_Z
                     - COXA_SIDE1_T - 6.6]))

        # The lift servo, standing in the fork with its shaft along the
        # leg's y and its flange bolted to the fork's two blocks.
        # The lift servo stands in the fork, centred in it: its flange
        # bolts across the two blocks, not onto one of them, so its own
        # middle is the fork's middle -- and that is where the femur's axis
        # falls.
        (self.lift_servo
         .rotate(-90, [0, 1, 0]).rotate(-90, [0, 0, 1])
         .translate([COXA_LENGTH, 0.0, joint.FORK_MID]))
        (self.lift_adapter
         .translate([-9.0, -10.0, -30.6])
         .rotate(90, [1, 0, 0])
         .translate([COXA_LENGTH,
                     joint.BEARING_PLATE_FACE_Z - FEMUR_SIDE1_T - 6.6,
                     joint.FORK_MID]))

        self.femur.translate([COXA_LENGTH, 0.0, joint.FORK_MID])


class Leg(AssemblyNode):
    """One whole leg, from the station's servo out to the claw.

    The yaw servo is here rather than in the coxa because it is bolted to
    the body's holder and does not turn with the leg; the coxa's plates and
    everything past them do.

    The three angles are not drivers, because the robot is steered by the
    pose of its body: the root works out where each foot has to be and what
    angles put it there, and binds the three joints -- `coxa.yaw`,
    `coxa.femur.lift` and `coxa.femur.tibia.knee` -- by path. See
    `spiderbot.py`.
    """

    #: How long each link is, so a reader and a test can ask the leg rather
    #: than the parameter table. The tibia's is not a length along its own
    #: x but the distance out to `FOOT`.
    LINKS = (COXA_LENGTH, FEMUR_LENGTH, TIBIA_REACH)

    yaw_servo = sourced.Servo()
    coxa = Coxa()

    def render(self):
        self.yaw_servo.translate([0.0, 0.0, 0.0])
