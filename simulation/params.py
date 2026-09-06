# SpiderBot simulation -- the numbers, and where each one came from.
# SPDX-License-Identifier: MIT

"""Everything the model measures, in one place.

The repository publishes meshes and photographs and no placements, so every
number here was read out of a mesh by `simulation/tools/probe.py`. Each
constant says which reading it is. A number that is a *fit* rather than a
reading -- something the meshes constrain but do not pin -- says so in its
own comment, because the difference matters when a contract fails.

Local coordinates are a part's own, measured from its bounding-box minimum,
because that is the frame a placement works in.
"""

# ---------------------------------------------------------------- colours

#: The robot's own black: the frame, the carapace, the leg plates.
BLACK = '#26262a'

#: The robot's own red: the leg shields and the soft claws.
RED = '#cf2b24'

#: A slightly lighter black, so a plate stacked on a plate reads as two.
BLACK_LIGHT = '#3a3a40'

#: The servos. Dark, but blue-shifted, so plastic case reads as not-plastic.
SERVO_CASE = '#2f3540'

#: The servo horn and output spline: white nylon, as they ship.
SERVO_HORN = '#d8d8d2'

#: Bearing steel.
STEEL = '#9aa2ad'

#: The LiPo pack.
BATTERY = '#1d4e89'

#: The controller board.
PCB = '#1d7a49'

#: The microswitch body.
SWITCH = '#c9c2ab'

#: Screws and pins.
FASTENER = '#7f8790'


# ------------------------------------------------------ the servo, sourced

#: Pitch between the two holes of one servo mounting pad, mm. Read on the
#: frame side holder, the frame centre holder, the coxa's distal fork and
#: the tibia bracket -- four parts drawn apart, one number.
SERVO_HOLE_PITCH = 10.0

#: Span between the two mounting pads, mm. 48.0 on `FrameSideServoHolder`
#: and on `TibiaTop`; 49.0 on `FrameCenterServoHolder`. The majority and the
#: tighter of the two is taken, and the centre holder's extra millimetre is
#: read as clearance.
SERVO_FLANGE_SPAN = 48.0

#: The gap a servo stands in, mm: the measured length of `LegRib`, the
#: spacer bolted between the two plates of the coxa and of the femur.
FORK_GAP = 48.6

#: Case, mm. A 40 mm-class digital servo -- the class this flange pattern
#: belongs to. Length along the flange span, width across it, height from
#: the case bottom to the top of the boss the shaft comes out of.
SERVO_BODY_L = 40.2
SERVO_BODY_W = 20.0
SERVO_BODY_H = 40.5

#: The flange plate, mm: its outline, its thickness, and how far its
#: underside stands above the case bottom.
SERVO_FLANGE_L = 53.0
SERVO_FLANGE_T = 2.7
SERVO_FLANGE_Z = 27.3

#: The output shaft: diameter and how far it stands above the case top.
#: A servo of this class puts its shaft about 10 mm off the centre of its
#: flange span. Nothing in the meshes says which way round each of the
#: eighteen servos is fitted, and the two answers differ by 20 mm of leg
#: offset, so the model draws the shaft on the flange span's centre and
#: says so rather than picking a side and presenting it as measured. See
#: the design record.
SERVO_SHAFT_D = 5.8
SERVO_SHAFT_H = 4.0

#: The round horn screwed to an eight-hole disc: the Ø21.5 recess measured
#: in `CoxaSide2_8holes` and `FemurSide2_8Holes` is what sizes it.
HORN_D = 21.0
HORN_T = 2.5
HORN_BOSS_D = 12.0
HORN_BOSS_H = 4.0

#: The eight horn screws: their circle and their size, read as eight Ø3.5
#: holes at radius 7.0 around each disc centre.
HORN_BOLT_CIRCLE_R = 7.0
HORN_BOLT_D = 3.5
HORN_BOLT_COUNT = 8

#: The disc's central clearance and its recess, read on `CoxaSide2_8holes`:
#: Ø6.0 through 2.5 mm of web, then Ø21.5 for 4.0 mm behind it.
DISC_BORE_D = 6.0
DISC_WEB_T = 2.5
DISC_RECESS_D = 21.5
DISC_RECESS_T = 4.0


# ------------------------------------------------------------- the bearing

#: 625ZZ: what `CoxaSide1_625ZZ` and `FemurSide1_625ZZ` are named for.
#: The probe reads only a Ø13.5 x 1.5 relief at the outer face of each seat,
#: because the seat is a raised collar the circle fit sees edge-on; the file
#: name is the better evidence and is what the model fits.
BEARING_OD = 16.0
BEARING_ID = 5.0
BEARING_W = 5.0

#: The relief the probe did read, kept so a contract can point at it.
BEARING_SEAT_RELIEF_D = 13.5
BEARING_SEAT_RELIEF_T = 1.5


# ------------------------------------------------------- the coxa, as read

#: `CoxaSide1_625ZZ` is 63.75 x 7.00 x 21.49 and `CoxaSide2_8holes` is
#: 63.75 x 10.00 x 21.49. Both carry their joint feature at the same place.

#: Where each part's own coordinates start, so a reading taken from a
#: bounding-box minimum can be turned into the coordinates a placement
#: works in. The probe reports local; the framework places in the file's
#: own frame, and the two differ by exactly this.
ORIGIN_COXA = (-4.75, 0.0, -18.75)
ORIGIN_FEMUR = (-4.75, 0.0, -18.75)
ORIGIN_TIBIA_TOP = (-3.17, -3.5, -60.95)
ORIGIN_SHIN = (0.0, 0.0, -13.6)
ORIGIN_RIB = (0.0, 0.0, -16.0)

#: The horn disc and the bearing seat, in the plate's own (x, z).
COXA_HUB = (ORIGIN_COXA[0] + 10.75, ORIGIN_COXA[2] + 10.75)

#: The distal fork's flange bolts, at local x, running along the plate's z.
#: Their midpoint is where the femur servo's shaft stands.
COXA_FORK_BOLTS_X = (ORIGIN_COXA[0] + 49.25, ORIGIN_COXA[0] + 59.25)
COXA_FORK_X = ORIGIN_COXA[0] + 54.25

#: The rib bolts, at local x, 8.0 apart in z.
COXA_RIB_X = ORIGIN_COXA[0] + 37.85
COXA_RIB_Z = (ORIGIN_COXA[2] + 6.75, ORIGIN_COXA[2] + 14.75)

#: Plate thicknesses, mm: the bearing side and the horn side.
COXA_SIDE1_T = 7.0
COXA_SIDE2_T = 10.0

#: The coxa link: horn to fork, mm.
COXA_LENGTH = COXA_FORK_X - COXA_HUB[0]          # 43.5


# ------------------------------------------------------ the femur, as read

#: `FemurSide1_625ZZ` is 101.5 x 6.50 x 21.49, `FemurSide2_8Holes` is
#: 101.5 x 9.50 x 21.50. Both carry a joint feature at each end.
FEMUR_HUB_PROXIMAL = (ORIGIN_FEMUR[0] + 10.75, ORIGIN_FEMUR[2] + 10.75)
FEMUR_HUB_DISTAL = (ORIGIN_FEMUR[0] + 90.75, ORIGIN_FEMUR[2] + 10.75)
FEMUR_RIB_X = ORIGIN_FEMUR[0] + 56.75
FEMUR_RIB_Z = (ORIGIN_FEMUR[2] + 6.75, ORIGIN_FEMUR[2] + 14.75)
FEMUR_SIDE1_T = 6.5
FEMUR_SIDE2_T = 9.5

#: The femur link: horn to horn, mm.
FEMUR_LENGTH = FEMUR_HUB_DISTAL[0] - FEMUR_HUB_PROXIMAL[0]      # 80.0


# ------------------------------------------------------- the tibia, as read

#: `TibiaTop` is 75.96 x 16.00 x 73.17. Its two servo arms carry the tibia
#: servo: two holes 10.0 apart within an arm, 48.0 between the arms.
TIBIA_ARM_A = ((32.07, 30.82), (39.73, 24.39))
TIBIA_ARM_B = ((62.92, 67.59), (70.58, 61.16))

#: The knee axis in `TibiaTop`'s own (x, z): the midpoint of the four arm
#: holes, which is where the model's shaft stands.
TIBIA_KNEE = (ORIGIN_TIBIA_TOP[0] + 51.33, ORIGIN_TIBIA_TOP[2] + 46.00)

#: Where `TibiaBottomLong` bolts on: two holes 8.0 apart, along `TibiaTop`'s
#: z and along the shin's x.
TIBIA_SHIN_BOLT_TOP = (ORIGIN_TIBIA_TOP[0] + 4.5,
                       (ORIGIN_TIBIA_TOP[1] + 4.0,
                        ORIGIN_TIBIA_TOP[1] + 12.0),
                       ORIGIN_TIBIA_TOP[2] + 7.71)
TIBIA_SHIN_BOLT_SHIN = (ORIGIN_SHIN[0] + 1.99, 4.5,
                        (ORIGIN_SHIN[2] + 4.0, ORIGIN_SHIN[2] + 12.0))

#: `TibiaBottomLong` is 102.21 x 9.00 x 16.00. Its far end carries the Ø2
#: claw pin.
SHIN_PIN_X = ORIGIN_SHIN[0] + 90.28
SHIN_PIN_Z = (ORIGIN_SHIN[2] + 4.93, ORIGIN_SHIN[2] + 11.03)
SHIN_PIN_D = 2.0



# -------------------------------------------------- the leg rib, as read

#: `LegRib` is 48.60 x 8.00 x 16.00, with two bolt holes running its length
#: at z 4.0 and 12.0, each interrupted by a nut trap.
RIB_LENGTH = 48.6
RIB_BOLT_Z = (ORIGIN_RIB[2] + 4.0, ORIGIN_RIB[2] + 12.0)


# ------------------------------------------------- the frame, as read

#: `Frame.stl` is 110.00 x 16.00 x 155.00 and is printed lying down: its
#: 16 mm axis is the robot's vertical.
FRAME_W = 110.0
FRAME_H = 16.0
FRAME_L = 155.0

#: The six stations, in the robot's own frame -- x across to the right,
#: y forward, z up, origin at the ring's centre on its lower face. Each is
#: the frame hole the station's holder bolts through, converted out of the
#: frame's local coordinates, and the heading its hole axis gives.
#: Left stations are the mirror: x and heading negated.
STATIONS = (
    ('front_right', 39.01, 60.74, 45.0),
    ('middle_right', 46.75, -0.20, 0.0),
    ('rear_right', 38.24, -61.51, -45.0),
    ('front_left', -39.01, 60.74, 135.0),
    ('middle_left', -46.75, -0.20, 180.0),
    ('rear_left', -38.24, -61.51, -135.0),
)

#: The three tripod-A legs. The other three are tripod B.
TRIPOD_A = ('front_right', 'middle_left', 'rear_right')

#: How far outboard of its frame hole a holder puts the coxa shaft, mm.
#: A fit, not a reading: the side holder's servo pads sit at local z 3.5 and
#: 51.5 and its frame bolts run along the same axis, so the shaft stands at
#: the pad span's centre carried out from the mounting face. The value is
#: the one that seats the holder against the frame's pad without sharing
#: volume with it, checked by contract.
STATION_REACH = 27.5

#: Where a leg's own frame stands in the robot's z, mm. The leg frame's
#: origin is the yaw servo's case bottom, its flange is 27.3 mm above that,
#: and the flange is bolted to the holder's upper face, which is the frame's
#: own upper face at z = 16. So the origin sits 11.3 mm below the frame.
STATION_Z = FRAME_H - 27.3


# ------------------------------------------------ the body stack, as read

#: `PowerCompartment` is 109.58 x 27.00 x 154.91 -- the frame's own
#: footprint -- and hangs below it. `PowerCompDoor` closes its underside.
#: `ElectronicsPlate` is 109.00 x 5.50 x 155.20, `ControllerPlate` is
#: 110.78 x 66.00 x 3.00, and the three carapace pieces are each 106 x 51
#: with rising crowns.
COMPARTMENT_H = 27.0
ELECTRONICS_T = 5.5
CONTROLLER_T = 3.0

#: The carapace pieces along the robot's length, back to front, each 51 mm
#: of the body's 155 mm.
CARAPACE_PITCH = 51.0

#: The 2S LiPo the compartment is drawn around: a common 5000 mAh hard case.
BATTERY_L = 130.0
BATTERY_W = 46.0
BATTERY_H = 24.0

#: The controller board the build guide names -- a Servo 2040-class board.
BOARD_L = 84.0
BOARD_W = 51.0
BOARD_T = 1.6


# --------------------------------------------------------- the foot switch

#: `MicroswitchHolderRound` is 19.30 x 17.00 x 29.00 and `SwitchCover` is
#: 1.50 x 17.00 x 28.50. The switch between them is a standard subminiature.
SWITCH_L = 12.8
SWITCH_W = 5.8
SWITCH_H = 6.5


# -------------------------------------------------------------- posture

#: The standing pose the robot demonstrates.
STAND_HEIGHT = 110.0
STAND_REACH = 185.0

#: How far the foot lifts on a swing, mm.
SWING_LIFT = 35.0
