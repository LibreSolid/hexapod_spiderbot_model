# SpiderBot simulation -- the whole robot, and what it does.
# SPDX-License-Identifier: MIT

"""The robot: a chassis on six legs, standing on the ground.

The root does not steer eighteen joints. It steers the *body*: how high it
stands, how far it strides, how it is rolled, pitched and yawed, and where it
is in a walking cycle. Every joint angle follows from that, through the
ordinary planar solution written in `Chassis._solve`, which a reader can
check against the measured link lengths rather than against a solver.

**The root's frame is the ground, not the robot.** That is what makes the
attitude drivers mean anything: the chassis -- body and legs together -- is
lifted to `height` and turned by roll, pitch and yaw, and each leg is solved
so its foot stays exactly where it was on the ground. Roll the robot and the
feet do not move; that is a claim a test can catch, and it would be
vacuous if the body were the fixed frame.

The robot walks in place. The feet slide backwards under a chassis that
stays over the origin, which is what a viewer wants: a robot that walks off
the screen is a robot you cannot watch.

The gait is a tripod -- front-right, middle-left and rear-right swing while
the other three carry, changing over at half a cycle. It runs off `$t` as
well as its own driver, so the model walks in the viewer with no button
pressed, and stands still when the stride is zero.
"""

import math

from solid_node.math import atan2, cos, sin, sqrt
from solid_node.mechanisms import triangle_angle

from solid_node.node import AssemblyNode
from solid_node.motion.ports import SignalPort
from solid_node.simulation import Driver, Instruction

from .body import Body
from .joint import FORK_MID
from .leg import Leg, TIBIA_BEND, TIBIA_REACH
from .params import (
    COXA_LENGTH, FEMUR_LENGTH, STAND_HEIGHT, STAND_REACH, STATIONS,
    STATION_REACH, STATION_Z, SWING_LIFT, TRIPOD_A,
)

#: How many walking cycles one sweep of the viewer's `$t` runs.
GAIT_CYCLES = 2.0

#: The stride the swing lift is scaled against: at this stride the feet lift
#: `SWING_LIFT`, and at zero stride they do not lift at all, so a robot
#: standing still really stands still.
STRIDE_REF = 40.0

#: One whole turn, in the degrees every angle here is in.
_TURN = 360.0


def _rectify(value, softness=0.02):
    """`max(value, 0)`, smoothly and without a branch.

    A conditional cannot be written symbolically, and the swing half of a
    gait cycle is exactly a conditional: lift while the sine is positive,
    stay down while it is not. This is the standard smooth stand-in --
    ``(u + sqrt(u^2 + e^2)) / 2`` -- which is one expression in every
    runtime and never leaves a foot below the ground, because it is never
    negative. `softness` is small against the sine's own unit scale, so a
    supporting foot is on the ground to well under a millimetre.
    """
    return (value + sqrt(value * value + softness * softness)) / 2.0


class Chassis(AssemblyNode):
    """The body and its six legs: everything that moves together.

    Its own frame is the robot's -- x right, y forward, z up, origin at the
    frame ring's centre on its lower face -- and the root carries it about
    over the ground. The foot targets it solves for are worked out on the
    ground and brought into this frame, so a leg always knows where its foot
    has to be no matter how the chassis is standing.
    """

    body = Body()
    legs = Leg().repeat(6)

    height = SignalPort(unit='mm')
    stride = SignalPort(unit='mm')
    reach = SignalPort(unit='mm')
    roll = SignalPort(unit='deg')
    pitch = SignalPort(unit='deg')
    yaw = SignalPort(unit='deg')
    gait_phase = SignalPort(unit='turn')
    wave = SignalPort(unit='')

    def render(self):
        # The body is built lying down, in its files' own frame; one quarter
        # turn stands the robot up.
        self.body.rotate(90, [1, 0, 0])

        # Each leg is carried out to its station and turned to face along
        # its heading. Everything after that happens in the leg's own frame.
        for leg, (_, x, y, heading) in zip(self.legs, STATIONS):
            leg.rotate(heading, [0, 0, 1]).translate(
                list(station(x, y, heading)))

    def simulate(self):
        for leg, (name, x, y, heading) in zip(self.legs, STATIONS):
            coxa, lift, knee = self._solve(name, x, y, heading)
            leg.coxa_angle = coxa
            leg.lift = lift
            leg.knee = knee

    # ------------------------------------------------------------ geometry

    def _foot_target(self, name, x, y, heading):
        """Where this leg's foot has to be, in the chassis's own frame.

        Worked out on the ground and then brought up into the chassis: the
        nominal stance is a ring of six feet at `reach` from their stations,
        the gait slides each along its heading and lifts it, and the
        chassis's own attitude is undone so the foot stays where the ground
        put it.
        """
        radians = math.radians(heading)
        along = (math.cos(radians), math.sin(radians))

        # The phase this leg walks on: one tripod half a cycle behind the
        # other, and the whole thing carried by `$t` as well as by its
        # driver, so the model walks on its own in the viewer.
        turn = self.gait_phase.value + self.time * GAIT_CYCLES
        if name not in TRIPOD_A:
            turn = turn + 0.5
        angle = turn * _TURN

        stride = self.stride.value
        travel = stride / 2.0 * cos(angle)
        lift = SWING_LIFT * _rectify(sin(angle)) * stride / STRIDE_REF

        # `reach` is measured from the leg's own yaw axis, not from the
        # frame hole the holder bolts through, because that is the distance
        # the two-link solution actually works in. The holder's own
        # `STATION_REACH` carries it out from the hole.
        radius = STATION_REACH + self.reach.value + travel
        ground = (x + radius * along[0], y + radius * along[1], lift)
        return self._to_chassis(ground)

    def _to_chassis(self, point):
        """A point on the ground, expressed in the chassis's frame.

        The root lifts the chassis by `height` and turns it by roll, then
        pitch, then yaw, so this is that composition run backwards.
        """
        px, py, pz = point[0], point[1], point[2] - self.height.value

        cy, sy = cos(-self.yaw.value), sin(-self.yaw.value)
        px, py = px * cy - py * sy, px * sy + py * cy

        cp, sp = cos(-self.pitch.value), sin(-self.pitch.value)
        px, pz = px * cp + pz * sp, -px * sp + pz * cp

        cr, sr = cos(-self.roll.value), sin(-self.roll.value)
        py, pz = py * cr - pz * sr, py * sr + pz * cr

        return (px, py, pz)

    def _solve(self, name, x, y, heading):
        """The three joint angles that put this leg's foot on its target.

        The ordinary planar solution, in the leg's own frame: the yaw is
        where the foot lies round the station, and the lift and the knee are
        a two-link reach at the measured 80.0 mm femur and the tibia's own
        `TIBIA_REACH` out to the claw.
        """
        target = self._foot_target(name, x, y, heading)
        origin = station(x, y, heading)
        vx = target[0] - origin[0]
        vy = target[1] - origin[1]
        vz = target[2] - origin[2]

        # Into the leg's frame: undo the station's heading.
        radians = math.radians(heading)
        ch, sh = math.cos(radians), math.sin(radians)
        lx = vx * ch + vy * sh
        ly = -vx * sh + vy * ch

        coxa = atan2(ly, lx)

        # The two-link reach in the leg's vertical plane. The lift axis
        # stands `COXA_LENGTH` out from the yaw axis at the leg frame's own
        # height, so the reach starts there.
        span = sqrt(lx * lx + ly * ly) - COXA_LENGTH
        drop = vz - FORK_MID
        distance = sqrt(span * span + drop * drop)

        # The second link is not along the tibia's own x: the shin hangs
        # down and forward of the knee, and `TIBIA_BEND` says by how much.
        # So the two-link solution gives the angle of *that vector*, and the
        # knee driver is what is left when the bend is taken out.
        lift = (atan2(drop, span)
                + triangle_angle(TIBIA_REACH, FEMUR_LENGTH, distance))
        bend = triangle_angle(distance, FEMUR_LENGTH, TIBIA_REACH) - 180.0
        knee = bend - TIBIA_BEND

        # The wave: one front leg leaves the ground and swings, and the
        # driver is how much of it. The other five stand.
        if name == 'front_right':
            wave = self.wave.value
            lift = lift + 45.0 * wave
            knee = knee + 30.0 * wave
            coxa = coxa + 25.0 * sin(self.time * 4 * _TURN) * wave

        return coxa, lift, knee


def station(x, y, heading):
    """A leg frame's origin in the chassis's frame.

    The station's frame hole carried outward by the holder's own reach, at
    the height the yaw servo's flange lands when it is bolted to the
    holder's upper face.
    """
    radians = math.radians(heading)
    return (x + STATION_REACH * math.cos(radians),
            y + STATION_REACH * math.sin(radians),
            STATION_Z)


class Spiderbot(AssemblyNode):
    """The SpiderBot, standing on the ground.

    The root's frame is the ground: z = 0 is the floor the six claws touch,
    x is to the right and y is forward. Everything the robot is hangs off
    one child, and the drivers say where that child stands.
    """

    chassis = Chassis()

    height = Driver(default=STAND_HEIGHT, range=(50.0, 170.0), unit='mm')
    reach = Driver(default=STAND_REACH, range=(130.0, 240.0), unit='mm')
    stride = Driver(default=0.0, range=(0.0, 60.0), unit='mm')
    roll = Driver(default=0.0, range=(-20.0, 20.0), unit='deg')
    pitch = Driver(default=0.0, range=(-20.0, 20.0), unit='deg')
    yaw = Driver(default=0.0, range=(-25.0, 25.0), unit='deg')
    gait_phase = Driver(default=0.0, range=(0.0, 1.0), unit='turn')
    wave = Driver(default=0.0, range=(0.0, 1.0), unit='')

    instructions = {
        'Stand': Instruction({'height': STAND_HEIGHT, 'reach': STAND_REACH,
                              'stride': 0.0, 'roll': 0.0, 'pitch': 0.0,
                              'yaw': 0.0, 'wave': 0.0}, duration=1.5),
        'Crouch': Instruction({'height': 60.0, 'reach': 200.0,
                               'stride': 0.0, 'wave': 0.0}, duration=1.5),
        'Tiptoe': Instruction({'height': 165.0, 'reach': 155.0,
                               'stride': 0.0, 'wave': 0.0}, duration=2.0),
        'Sit': Instruction({'height': 70.0, 'pitch': -15.0, 'reach': 195.0,
                            'stride': 0.0, 'wave': 0.0}, duration=2.0),
        'Wave': Instruction({'wave': 1.0, 'height': STAND_HEIGHT,
                             'stride': 0.0}, duration=1.5),
        'LookAround': Instruction({'yaw': 25.0, 'roll': 12.0,
                                   'pitch': 8.0, 'stride': 0.0},
                                  duration=2.0),
        'Walk': Instruction({'stride': 45.0, 'height': STAND_HEIGHT,
                             'reach': STAND_REACH, 'wave': 0.0},
                            duration=1.5),
    }

    def simulate(self):
        # Every pose value the chassis needs, handed down.
        self.chassis.height = self.height
        self.chassis.reach = self.reach
        self.chassis.stride = self.stride
        self.chassis.roll = self.roll
        self.chassis.pitch = self.pitch
        self.chassis.yaw = self.yaw
        self.chassis.gait_phase = self.gait_phase
        self.chassis.wave = self.wave

        # And where it stands: turned about its own centre, then lifted off
        # the ground. The legs are solved against exactly this composition,
        # so the feet stay planted through all of it.
        (self.chassis
         .rotate(self.roll, [1, 0, 0])
         .rotate(self.pitch, [0, 1, 0])
         .rotate(self.yaw, [0, 0, 1])
         .translate([0.0, 0.0, self.height]))
