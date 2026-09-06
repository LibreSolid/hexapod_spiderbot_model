# SpiderBot simulation -- the parts the repository does not print.
# SPDX-License-Identifier: MIT

"""The bought parts.

The build guide prints twenty-five things and buys the rest, and the rest is
what actually makes the robot a machine: eighteen servos, eighteen bearings,
a battery, a controller and six foot switches. None of them has an STL here,
so this module is the only place in the layer where geometry is drawn rather
than placed.

Each part is drawn to the seat that receives it, and every dimension it
takes comes from `params.py` -- that is, from a measured hole. A servo's
flange is 48.0 mm across because four printed parts put their mounting holes
48.0 mm apart; the 625ZZ is a 625ZZ because two file names say so. Nothing
here is styled beyond what a seat demands, except the colours, which are
chosen so a viewer can tell steel from plastic from board.

All of these are exact solids, so the assertions that use them are answered
on geometry rather than on tessellation.
"""

import cadquery as cq

from solid_node.node import CadQueryNode

from .params import (
    BATTERY, BATTERY_H, BATTERY_L, BATTERY_W,
    BEARING_ID, BEARING_OD, BEARING_W,
    BOARD_L, BOARD_T, BOARD_W,
    FASTENER, HORN_BOLT_CIRCLE_R, HORN_BOLT_COUNT, HORN_BOLT_D,
    HORN_BOSS_D, HORN_BOSS_H, HORN_D, HORN_T,
    PCB, SERVO_BODY_H, SERVO_BODY_L, SERVO_BODY_W, SERVO_CASE,
    SERVO_FLANGE_L, SERVO_FLANGE_SPAN, SERVO_FLANGE_T, SERVO_FLANGE_Z,
    SERVO_HOLE_PITCH, SERVO_HORN, SERVO_SHAFT_D, SERVO_SHAFT_H,
    SHIN_PIN_D, STEEL, SWITCH, SWITCH_H, SWITCH_L, SWITCH_W,
)

#: How much of the case is below the output boss, mm.
_BOSS_H = 2.0


class Servo(CadQueryNode):
    """One 40 mm-class digital servo.

    Its frame is the frame the whole robot's motion is built on: the output
    shaft is the +z axis, the origin sits on the axis at the case's bottom
    face, and the flange -- the plate that bolts it to whatever holds it --
    is a horizontal slab partway up.

    Everything about it is a consequence of the four holes in that flange.
    The 10.0 mm pitch across and the 48.0 mm span along are measured on four
    printed parts; the case is what fits between them; the shaft stands on
    the span's centre, for the reason `params.py` gives.
    """

    color = SERVO_CASE

    def render(self):
        case = (cq.Workplane('XY')
                .box(SERVO_BODY_L, SERVO_BODY_W, SERVO_BODY_H - _BOSS_H,
                     centered=(True, True, False)))

        flange = (cq.Workplane('XY')
                  .workplane(offset=SERVO_FLANGE_Z)
                  .box(SERVO_FLANGE_L, SERVO_BODY_W, SERVO_FLANGE_T,
                       centered=(True, True, False))
                  .faces('>Z').workplane()
                  .pushPoints([(x, y)
                               for x in (-SERVO_FLANGE_SPAN / 2,
                                         SERVO_FLANGE_SPAN / 2)
                               for y in (-SERVO_HOLE_PITCH / 2,
                                         SERVO_HOLE_PITCH / 2)])
                  .hole(3.5))

        boss = (cq.Workplane('XY')
                .workplane(offset=SERVO_BODY_H - _BOSS_H)
                .circle(12.5 / 2).extrude(_BOSS_H))

        shaft = (cq.Workplane('XY')
                 .workplane(offset=SERVO_BODY_H)
                 .circle(SERVO_SHAFT_D / 2).extrude(SERVO_SHAFT_H))

        return case.union(flange).union(boss).union(shaft)


class ServoHorn(CadQueryNode):
    """The round horn screwed to an eight-hole disc.

    Sized by the seat that takes it: the Ø21.5 recess measured in
    `CoxaSide2_8holes` and `FemurSide2_8Holes`, and the eight Ø3.5 holes on
    a Ø14 circle around it. Its origin is on the joint axis at the top of
    the servo's output boss, so the boss below and the disc above stack
    upward from there.
    """

    color = SERVO_HORN

    def render(self):
        # Bored for the shaft it is pressed onto: the horn is a collar, and
        # a solid one would share the output shaft's own volume.
        hub = (cq.Workplane('XY')
               .circle(HORN_BOSS_D / 2).circle(SERVO_SHAFT_D / 2 + 0.1)
               .extrude(HORN_BOSS_H))
        disc = (cq.Workplane('XY')
                .workplane(offset=HORN_BOSS_H)
                .circle(HORN_D / 2).extrude(HORN_T)
                .faces('>Z').workplane()
                .polarArray(HORN_BOLT_CIRCLE_R, 0, 360, HORN_BOLT_COUNT)
                .hole(HORN_BOLT_D))
        return hub.union(disc)


class Bearing625ZZ(CadQueryNode):
    """A 625ZZ deep-groove ball bearing: Ø16 outside, Ø5 bore, 5 wide.

    Two printed parts are named for this bearing -- `CoxaSide1_625ZZ` and
    `FemurSide1_625ZZ` -- and that name is the evidence it is fitted. Its
    axis is +z and its origin is on the axis at its lower face. The shields
    are drawn as the recessed faces they are, so it reads as a bearing and
    not as a washer.
    """

    color = STEEL

    def render(self):
        ring = (cq.Workplane('XY')
                .circle(BEARING_OD / 2).circle(BEARING_ID / 2)
                .extrude(BEARING_W))
        for face in ('>Z', '<Z'):
            ring = (ring.faces(face).workplane()
                    .circle(BEARING_OD / 2 - 1.2).circle(BEARING_ID / 2 + 1.2)
                    .cutBlind(-0.4))
        return ring


class LipoPack(CadQueryNode):
    """The 2S pack the power compartment is drawn around.

    A hard-case 5000 mAh cell of the size the compartment's bay takes, with
    its lead stub so the compartment's cable slot has something to be for.
    Centred in x and y on its own origin, standing up from z = 0.
    """

    color = BATTERY

    def render(self):
        pack = (cq.Workplane('XY')
                .box(BATTERY_L, BATTERY_W, BATTERY_H,
                     centered=(True, True, False))
                .edges('|Z').fillet(3.0))
        lead = (cq.Workplane('YZ')
                .workplane(offset=BATTERY_L / 2)
                .center(0, BATTERY_H / 2)
                .circle(3.0).extrude(9.0))
        return pack.union(lead)


class ControllerBoard(CadQueryNode):
    """The controller the build guide names for the controller plate.

    A Servo 2040-class board: the PCB, its headers and the block of
    connectors along one edge, enough that the plate under it and the
    carapace over it can be checked for clearance.
    """

    color = PCB

    def render(self):
        board = (cq.Workplane('XY')
                 .box(BOARD_L, BOARD_W, BOARD_T, centered=(True, True, False)))
        headers = (cq.Workplane('XY')
                   .workplane(offset=BOARD_T)
                   .box(BOARD_L - 12.0, 6.0, 8.5,
                        centered=(True, True, False)))
        module = (cq.Workplane('XY')
                  .workplane(offset=BOARD_T)
                  .center(-BOARD_L / 4, -BOARD_W / 4)
                  .box(24.0, 18.0, 4.0, centered=(True, True, False)))
        return board.union(headers).union(module)


class Microswitch(CadQueryNode):
    """One foot's contact switch.

    A subminiature lever switch, sized to the housing that takes it:
    `MicroswitchHolderRound` with `SwitchCover` over it. Its body lies along
    x, its lever stands off one long face, and its origin is at the body's
    centre on its lower face.
    """

    color = SWITCH

    def render(self):
        body = (cq.Workplane('XY')
                .box(SWITCH_L, SWITCH_W, SWITCH_H,
                     centered=(True, True, False)))
        lever = (cq.Workplane('XY')
                 .workplane(offset=SWITCH_H)
                 .box(SWITCH_L - 1.0, 1.0, 0.4, centered=(True, True, False)))
        pins = (cq.Workplane('XY')
                .center(0, 0)
                .rarray(4.5, 1, 3, 1)
                .box(0.8, 0.5, 4.0, centered=(True, True, False))
                .translate((0, 0, -4.0)))
        return body.union(lever).union(pins)


class ClawPin(CadQueryNode):
    """The Ø2 pin the claw swings on.

    The one fastener the model draws, because a measured hole names it --
    two Ø2 bores through the shin's far end -- and because the part it
    carries moves. Its axis is +z and its origin is at its lower face.
    """

    color = FASTENER

    def render(self):
        length = 14.0
        return (cq.Workplane('XY')
                .circle(SHIN_PIN_D / 2).extrude(length)
                .faces('>Z').workplane().circle(1.8).extrude(1.2))
