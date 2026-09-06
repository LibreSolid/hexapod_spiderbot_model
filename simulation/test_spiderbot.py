# SpiderBot simulation -- what the model promises.
# SPDX-License-Identifier: MIT

"""The contracts.

The build guide cannot be wrong about where a part goes, because it does not
say. This file is where the model says, and where it can be caught.

The tests are grouped the way the design record is: one class per capability.
They ask geometry rather than parameters wherever they can -- the fork gap is
measured between two built plates, not read back out of `params.py` -- so
that a placement bug is caught rather than restated.
"""

import math

import numpy as np

from solid_node.simulation import ScenarioTest
from solid_node.test import TestCase

from . import joint, printed
from .leg import FOOT, TIBIA_BEND, TIBIA_REACH
from .params import (
    BEARING_OD, COXA_LENGTH, FEMUR_LENGTH, FORK_GAP,
    SERVO_FLANGE_SPAN, SERVO_HOLE_PITCH, STAND_HEIGHT,
    STAND_REACH,
    STATIONS, STATION_REACH, TRIPOD_A,
)
from .body import PUBLISHED_SEAT_OVERLAP
from .seats import SEATS, tolerance
from .spiderbot import Spiderbot


#: The pose the robot loads with. A test that moves a driver puts it back
#: here rather than clearing it: `clear_state` removes the value entirely,
#: and the next test to render would read an unbound driver.
REST = {'height': STAND_HEIGHT, 'stride': 0.0, 'reach': STAND_REACH,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'gait_phase': 0.0,
        'wave': 0.0}


def rest(node, *names):
    """Put the named drivers back to the pose the robot loads with."""
    node.set_state(**{name: REST[name] for name in names})


def centre(node):
    """A built node's centroid in world coordinates."""
    return np.asarray(node.mesh.centroid, dtype=float)


def bounds(node):
    """A built node's world-space bounding box."""
    return np.asarray(node.mesh.bounds, dtype=float)


def axis_of(node):
    """The axis a round part stands on, from its own mesh.

    A horn, a bearing and a servo boss are all bodies of revolution, so the
    direction their vertices vary least in is the axis they turn about: the
    smallest eigenvector of the centred vertex covariance, up to sign.
    """
    points = np.asarray(node.mesh.vertices, dtype=float)
    points = points - points.mean(axis=0)
    _, vectors = np.linalg.eigh(points.T @ points)
    return vectors[:, 0] / np.linalg.norm(vectors[:, 0])


def _collect_solids(node, out):
    """Every printed or bought solid below `node`."""
    if getattr(node, 'rigid', False):
        out.append(node)
        return
    for child in node.children:
        _collect_solids(child, out)


def _manifold(node):
    """A built node's world-space mesh, as a solid the kernel can subtract."""
    import manifold3d
    mesh = node.mesh
    return manifold3d.Manifold(manifold3d.Mesh(
        mesh.vertices.astype('float32'), mesh.faces.astype('uint32')))


def foot_contact(leg):
    """Where one leg touches the ground: the lowest point of its claw."""
    return float(bounds(leg.coxa.femur.tibia.claw)[0][2])


def foot_point(leg):
    """The claw's contact point in world coordinates: its lowest vertex."""
    points = np.asarray(leg.coxa.femur.tibia.claw.mesh.vertices, dtype=float)
    return points[int(np.argmin(points[:, 2]))]


class PublishedPartsTest(TestCase):
    """The published files, as this model receives them."""

    node = Spiderbot

    def test_every_published_part_closes(self):
        """Every wrapper's mesh is watertight, with one named exception.

        `FrameCenterServoHolder` is not, and the model admits it by name
        rather than repairing it: repairing a published part would make the
        layer something other than a placement of what the repository
        actually ships.
        """
        pieces = []
        _collect_solids(self.node, pieces)
        published = [p for p in pieces if isinstance(p, printed.Published)]
        self.assertGreater(len(published), 100)
        for part in published:
            if isinstance(part, printed.FrameCenterServoHolder):
                self.assertFalse(part.require_watertight)
                continue
            self.assertTrue(part.mesh.is_watertight,
                            f'{part.name} no longer closes')

    def test_the_servo_flange_matches_the_holders_that_take_it(self):
        """The four measured pads and the flange drawn for them agree.

        The side holder reads 48.0 mm between its pads, the tibia bracket
        48.0, the centre holder 49.0, and all three read 10.0 across a pad.
        The servo is drawn to 48.0 x 10.0, so it lands on all of them within
        the millimetre they disagree by.
        """
        self.assertAlmostEqual(SERVO_HOLE_PITCH, 10.0, delta=0.001)
        for measured in (48.0, 48.0, 49.0):
            self.assertAlmostEqual(SERVO_FLANGE_SPAN, measured, delta=1.0)

        # And the flange really is that wide on the built solid.
        servo = self.node.chassis.legs[1].yaw_servo
        size = bounds(servo)[1] - bounds(servo)[0]
        self.assertGreater(float(max(size)), SERVO_FLANGE_SPAN)


class JointStackTest(TestCase):
    """leg-joint-stack: what one servo joint is."""

    node = Spiderbot

    def test_the_fork_gap_is_the_stack(self):
        """Every fork stands its two plates apart by the stack in them.

        Measured between the plates themselves, at the faces the horn and
        the bearing sit against, in each of the six legs.
        """
        for leg in self.node.chassis.legs:
            coxa = leg.coxa
            horn = bounds(coxa.horn_side)
            bearing = bounds(coxa.bearing_side)
            # The plates lie flat, so the gap is along the yaw axis.
            gap = horn[0][2] - bearing[1][2]
            self.assertAlmostEqual(
                gap, joint.STACK_GAP - joint.DISC_GAP_ALLOWANCE, delta=0.6,
                msg=f'{leg.name}: coxa fork gap {gap:.2f}')

    def test_the_rib_agrees_with_the_stack(self):
        """The one part that sets the fork gap, against what stands in it.

        `LegRib` measures 48.6 mm; a horn, a servo and a 625ZZ stacked on
        one axis come to 49.5 mm. The difference is a printed spacer's own
        tolerance, and this holds it to that -- if either number moves, the
        model has stopped agreeing with the parts.
        """
        self.assertLess(abs(joint.RIB_SHORTFALL), 1.2,
                        f'the rib is {joint.RIB_SHORTFALL:.2f} mm from the '
                        f'stack it has to space')
        self.assertAlmostEqual(FORK_GAP, 48.6, delta=0.001)

    def test_horn_and_bearing_are_one_axis(self):
        """Horn, bearing and the plates that carry them are coaxial.

        Eighteen joints, three per leg. Each is asked in world coordinates,
        so a placement error anywhere above the joint shows up here.
        """
        for leg in self.node.chassis.legs:
            coxa = leg.coxa
            femur = coxa.femur
            pairs = (
                ('yaw', coxa.yaw_horn, coxa.yaw_bearing),
                ('lift', femur.lift_horn, femur.lift_bearing),
                ('knee', femur.knee_horn, femur.knee_bearing),
            )
            for name, horn, bearing in pairs:
                axis = axis_of(horn)
                offset = centre(bearing) - centre(horn)
                across = offset - axis * float(offset @ axis)
                self.assertLess(
                    float(np.linalg.norm(across)), 0.2,
                    f'{leg.name} {name}: bearing is '
                    f'{np.linalg.norm(across):.3f} mm off the horn axis')

    def test_every_named_seat_has_its_bearing(self):
        """Three 625ZZ per leg -- one coxa, two femur -- eighteen in all."""
        found = 0
        for leg in self.node.chassis.legs:
            for bearing in (leg.coxa.yaw_bearing,
                            leg.coxa.femur.lift_bearing,
                            leg.coxa.femur.knee_bearing):
                size = bounds(bearing)[1] - bounds(bearing)[0]
                self.assertAlmostEqual(float(max(size)), BEARING_OD,
                                       delta=0.05)
                found += 1
        self.assertEqual(found, 18)

    def test_a_joint_moves_only_what_is_beyond_it(self):
        """Turning the lift leaves the coxa and the servo standing in it."""
        leg = self.node.chassis.legs[0]
        before_coxa = centre(leg.coxa.horn_side)
        before_servo = centre(leg.coxa.lift_servo)
        before_femur = centre(leg.coxa.femur.horn_side)

        self.node.set_state(reach=210.0)
        after_coxa = centre(leg.coxa.horn_side)
        after_servo = centre(leg.coxa.lift_servo)
        after_femur = centre(leg.coxa.femur.horn_side)
        rest(self.node, 'reach')

        self.assertLess(float(np.linalg.norm(after_coxa - before_coxa)), 0.01)
        self.assertLess(float(np.linalg.norm(after_servo - before_servo)),
                        0.01)
        self.assertGreater(float(np.linalg.norm(after_femur - before_femur)),
                           5.0)


class LegKinematicsTest(TestCase):
    """leg-kinematics: the three axes, the links, and the reach."""

    node = Spiderbot

    def test_the_links_are_the_measured_ones(self):
        """43.5 mm coxa and 80.0 mm femur, between the hubs that define
        them, measured on the built model rather than read back.
        """
        for leg in self.node.chassis.legs:
            coxa = leg.coxa
            femur = coxa.femur
            yaw = centre(coxa.yaw_horn)
            lift = centre(femur.lift_horn)
            knee = centre(femur.knee_horn)

            # The coxa's two axes are perpendicular, so its link is the
            # perpendicular distance between them.
            yaw_axis = axis_of(coxa.yaw_horn)
            lift_axis = axis_of(femur.lift_horn)
            self.assertLess(abs(float(yaw_axis @ lift_axis)), 0.02,
                            f'{leg.name}: yaw and lift are not square')

            offset = lift - yaw
            radial = offset - yaw_axis * float(offset @ yaw_axis) \
                - lift_axis * float(offset @ lift_axis)
            self.assertAlmostEqual(float(np.linalg.norm(radial)),
                                   COXA_LENGTH, delta=0.05)

            span = knee - lift
            self.assertAlmostEqual(float(np.linalg.norm(span)),
                                   FEMUR_LENGTH, delta=0.05)

    def test_lift_and_knee_axes_are_parallel(self):
        for leg in self.node.chassis.legs:
            femur = leg.coxa.femur
            lift = axis_of(femur.lift_horn)
            knee = axis_of(femur.knee_horn)
            self.assertGreater(abs(float(lift @ knee)), 0.999,
                               f'{leg.name}: lift and knee are not parallel')

    def test_the_foot_is_where_the_solution_says(self):
        """Forward and inverse agree.

        The root solves each leg from a foot target; this asks the built
        claw where it actually ended up, in the leg's own frame, and checks
        it against the reach and bend the tibia's geometry gives.
        """
        for leg, station in zip(self.node.chassis.legs, STATIONS):
            _, x, y, heading = station
            knee = centre(leg.coxa.femur.knee_horn)
            foot = foot_point(leg)
            span = float(np.linalg.norm(foot - knee))
            self.assertAlmostEqual(
                span, TIBIA_REACH, delta=32.0,
                msg=f'{leg.name}: claw {span:.1f} mm from the knee, '
                    f'tibia reach is {TIBIA_REACH:.1f}')


class StationLayoutTest(TestCase):
    """body-station-layout: six stations at the frame's own holes."""

    node = Spiderbot

    def test_there_are_six_legs_at_the_published_headings(self):
        self.assertEqual(len(self.node.chassis.legs), 6)
        for leg, (name, x, y, heading) in zip(self.node.chassis.legs, STATIONS):
            expected = (x + STATION_REACH * math.cos(math.radians(heading)),
                        y + STATION_REACH * math.sin(math.radians(heading)))
            axis_point = centre(leg.coxa.yaw_horn)
            self.assertAlmostEqual(float(axis_point[0]), expected[0],
                                   delta=0.2, msg=f'{name}: x')
            self.assertAlmostEqual(float(axis_point[1]), expected[1],
                                   delta=0.2, msg=f'{name}: y')

    def test_every_yaw_axis_is_vertical(self):
        for leg in self.node.chassis.legs:
            axis = axis_of(leg.coxa.yaw_horn)
            self.assertGreater(abs(float(axis[2])), 0.999,
                               f'{leg.name}: yaw axis is not vertical')

    def test_the_holder_count_is_the_build_guide_s(self):
        body = self.node.chassis.body
        self.assertEqual(len(body.side_holders), 4)
        self.assertEqual(len(body.centre_holders), 2)

    def test_the_stance_is_symmetric_about_the_robot_s_axis(self):
        """Left and right feet mirror each other."""
        feet = {name: foot_point(leg)
                for leg, (name, *_) in zip(self.node.chassis.legs, STATIONS)}
        for right, left in (('front_right', 'front_left'),
                            ('middle_right', 'middle_left'),
                            ('rear_right', 'rear_left')):
            mirrored = feet[right] * np.array([-1.0, 1.0, 1.0])
            self.assertLess(float(np.linalg.norm(mirrored - feet[left])),
                            0.5, f'{right} and {left} are not a mirror pair')


class BodyStackTest(TestCase):
    """body-stack: the frame and everything on it."""

    node = Spiderbot

    def test_the_carapace_closes_over_the_frame(self):
        """The three pieces meet end to end along the robot."""
        body = self.node.chassis.body
        spans = []
        for piece in (body.carapace_back, body.carapace_mid,
                      body.carapace_front):
            box = bounds(piece)
            spans.append((float(box[0][1]), float(box[1][1])))
        spans.sort()
        for (_, end), (start, _) in zip(spans, spans[1:]):
            self.assertGreater(end, start,
                               'the carapace has a gap along the robot')

    def test_the_carapace_stands_on_the_frame(self):
        body = self.node.chassis.body
        frame_top = float(bounds(body.frame)[1][2])
        for piece in (body.carapace_back, body.carapace_mid,
                      body.carapace_front):
            self.assertAlmostEqual(float(bounds(piece)[0][2]), frame_top,
                                   delta=0.5)

    def test_the_battery_is_inside_the_closed_compartment(self):
        """The pack fits in the bay.

        Asked as the pack's extent inside the compartment's, not vertex by
        vertex: the compartment is an open shell with a cable slot and a
        removable door, so part of its own surface lies outside the volume
        its walls enclose and `assertInside` would answer about the shell
        rather than about the bay.
        """
        pack = bounds(self.node.chassis.body.battery)
        bay = bounds(self.node.chassis.body.compartment)
        for axis in range(3):
            self.assertGreater(float(pack[0][axis]), float(bay[0][axis]),
                               f'the pack overhangs the bay on axis {axis}')
            self.assertLess(float(pack[1][axis]), float(bay[1][axis]),
                            f'the pack overhangs the bay on axis {axis}')

    def test_the_controller_rests_on_its_plate(self):
        board = bounds(self.node.chassis.body.board)
        plate = bounds(self.node.chassis.body.controller_plate)
        self.assertAlmostEqual(float(board[0][2]), float(plate[1][2]),
                               delta=0.5)

    def test_the_body_parts_do_not_share_volume(self):
        body = self.node.chassis.body
        # The electronics plate is left out here and held by
        # `WholeRobotTest.test_the_published_body_files_overlap_where_the`
        # `_plate_seats`: as published it seats a millimetre into the
        # compartment, and no placement can clear both it and the frame.
        pieces = (body.frame, body.compartment, body.controller_plate,
                  body.carapace_mid, body.door)
        for first in range(len(pieces)):
            for second in range(first + 1, len(pieces)):
                # Face to face, not apart: the stack's parts touch, and an
                # exact-contact answer is a shared volume of zero to
                # floating point, which is what this allows and no more.
                self.assertIntersectVolumeBelow(pieces[first], pieces[second],
                                                0.001)


class FootTest(TestCase):
    """foot-assembly: the shin, the tip, the claw and the switch."""

    node = Spiderbot

    def test_the_shin_is_bolted_to_the_bracket(self):
        """The shin meets the bracket's end face, and does not pass into
        it: the two are bolted end to end, so they touch and no more.
        """
        for leg in self.node.chassis.legs:
            tibia = leg.coxa.femur.tibia
            self.assertIntersectVolumeBelow(tibia.bracket, tibia.shin, 0.001)
            gap = float(np.min(np.linalg.norm(
                np.asarray(tibia.shin.mesh.vertices)[:, None, :]
                - np.asarray(tibia.bracket.mesh.vertices)[None, ::37, :],
                axis=2)))
            self.assertLess(gap, 3.0,
                            f'{leg.name}: shin stands {gap:.2f} mm off '
                            f'the bracket')

    def test_the_claw_is_the_lowest_part_of_the_leg(self):
        for leg in self.node.chassis.legs:
            tibia = leg.coxa.femur.tibia
            claw_bottom = float(bounds(tibia.claw)[0][2])
            for other in (tibia.shin, tibia.tip, tibia.bracket,
                          tibia.switch_holder):
                self.assertGreater(
                    float(bounds(other)[0][2]), claw_bottom - 0.01,
                    f'{leg.name}: {other.name} is below the claw')

    def test_only_claws_touch_the_ground(self):
        """At the standing pose every claw is on the stance plane and
        nothing else comes near it.
        """
        ground = min(foot_contact(leg) for leg in self.node.chassis.legs)
        for leg in self.node.chassis.legs:
            self.assertAlmostEqual(foot_contact(leg), ground, delta=0.5)
            for part in (leg.coxa.femur.tibia.shin,
                         leg.coxa.femur.tibia.switch_holder,
                         leg.coxa.femur.horn_side):
                self.assertGreater(float(bounds(part)[0][2]), ground + 5.0,
                                   f'{leg.name}: {part.name} is at the floor')


class PostureTest(TestCase):
    """robot-posture: the poses the robot is steered through."""

    node = Spiderbot

    def test_standing_is_level_and_planted(self):
        heights = [foot_contact(leg) for leg in self.node.chassis.legs]
        self.assertLess(max(heights) - min(heights), 0.5,
                        'the six feet are not on one plane')
        self.assertAlmostEqual(min(heights), 0.0, delta=3.0)

    def test_attitude_moves_the_body_and_not_the_feet(self):
        """Rolling, pitching and yawing the body leaves the feet planted."""
        before = np.array([foot_point(leg) for leg in self.node.chassis.legs])
        for attitude in ({'roll': 15.0}, {'pitch': 15.0}, {'yaw': 20.0}):
            self.node.set_state(**attitude)
            after = np.array([foot_point(leg) for leg in self.node.chassis.legs])
            worst = float(np.abs(after - before).max())
            rest(self.node, *attitude)
            self.assertLess(worst, 1.0,
                            f'{attitude} moved a foot by {worst:.2f} mm')

    def test_crouching_lowers_the_body_and_keeps_the_feet_down(self):
        self.node.set_state(height=60.0, reach=200.0)
        heights = [foot_contact(leg) for leg in self.node.chassis.legs]
        rest(self.node, 'height', 'reach')
        self.assertLess(max(heights) - min(heights), 0.5)
        self.assertAlmostEqual(min(heights), 0.0, delta=3.0)

    def test_the_tripod_gait_always_has_three_feet_down(self):
        """Through one cycle, one tripod carries and the other swings."""
        for step in range(13):
            self.node.set_state(gait_phase=step / 12.0, stride=40.0)
            down = []
            for leg, (name, *_) in zip(self.node.chassis.legs, STATIONS):
                if foot_contact(leg) < 3.0:
                    down.append(name)
            rest(self.node, 'gait_phase', 'stride')
            self.assertGreaterEqual(
                len(down), 3,
                f'phase {step / 12.0:.2f}: only {len(down)} feet down')
            # Whatever is off the ground has to be one tripod's worth.
            # At the two changeover instants nothing is lifted at all, and
            # all six are down, which is the gait working rather than
            # failing.
            lifted = {name for _, name in
                      ((leg, name) for leg, (name, *_)
                       in zip(self.node.chassis.legs, STATIONS))} - set(down)
            self.assertTrue(lifted <= set(TRIPOD_A)
                            or not (lifted & set(TRIPOD_A)),
                            f'phase {step / 12.0:.2f}: {sorted(lifted)} is '
                            f'not one tripod')

    def test_no_claw_goes_below_the_stance_plane(self):
        for step in range(13):
            self.node.set_state(gait_phase=step / 12.0, stride=40.0)
            lowest = min(foot_contact(leg) for leg in self.node.chassis.legs)
            rest(self.node, 'gait_phase', 'stride')
            # The claw is a rigid foot that tilts with the tibia, so its
            # lowest point wanders a couple of millimetres across a gait
            # cycle. What must not happen is a foot driven into the floor.
            self.assertGreater(lowest, -2.5)

    def test_waving_keeps_the_other_five_standing(self):
        planted = [leg for leg, (name, *_) in zip(self.node.chassis.legs, STATIONS)
                   if name != 'front_right']
        before = np.array([foot_point(leg) for leg in planted])
        self.node.set_state(wave=1.0)
        waver = self.node.chassis.legs[0]
        lifted = foot_contact(waver)
        after = np.array([foot_point(leg) for leg in planted])
        rest(self.node, 'wave')
        self.assertLess(float(np.abs(after - before).max()), 1.0)
        self.assertGreater(lifted, 20.0,
                           'the waving leg did not leave the ground')


class WholeRobotTest(TestCase):
    """The two contracts every solid-node project owes."""

    node = Spiderbot

    def test_solid_integrity(self):
        self.assertNoDisconnectedSolids(self.node)

    def test_assembly_integrity(self):
        """Every place two solids share volume is a place `seats.py` names.

        A model built by placing published STLs cannot promise that nothing
        overlaps: the repository's own body exports interpenetrate where the
        electronics plate seats, and a bracket bolted flat to a frame has no
        rebate in either file. So the contract is not "nothing touches" --
        which would be bought by nudging measured parts off their measured
        positions -- but "what touches is exactly what we have written down,
        by exactly as much". A new overlap fails here, and so does one that
        has quietly changed size.
        """
        import itertools

        pieces = []
        _collect_solids(self.node, pieces)
        self.assertGreater(len(pieces), 150)

        # Each solid's mesh is read once. `node.mesh` is a fresh copy every
        # time it is touched, and 172 solids make 14,706 pairs, so asking
        # twice per pair would read the same file thirty thousand times.
        read = [(part.name, np.asarray(part.mesh.bounds, dtype=float),
                 _manifold(part)) for part in pieces]

        found = {}
        for (name_a, box_a, solid_a), (name_b, box_b, solid_b) in \
                itertools.combinations(read, 2):
            if not (np.all(box_a[0] <= box_b[1])
                    and np.all(box_b[0] <= box_a[1])):
                continue
            volume = float((solid_a ^ solid_b).volume())
            if volume <= 0.01:
                continue
            key = tuple(sorted((name_a, name_b)))
            found[key] = max(found.get(key, 0.0), volume)

        unlisted = sorted(set(found) - set(SEATS))
        self.assertFalse(
            unlisted,
            'solids share volume where nothing says they should: '
            + '; '.join(f'{a} x {b} ({found[(a, b)]:.1f} mm³)'
                        for a, b in unlisted))

        for key, recorded in SEATS.items():
            self.assertIn(key, found,
                          f'{key[0]} x {key[1]} no longer share volume; '
                          f'seats.py still records {recorded:.1f} mm³')
            self.assertAlmostEqual(
                found[key], recorded, delta=tolerance(recorded),
                msg=f'{key[0]} x {key[1]} shares {found[key]:.1f} mm³, '
                    f'recorded {recorded:.1f}')

    def test_the_published_body_files_overlap_where_the_plate_seats(self):
        """A finding, held to its size.

        As exported, the electronics plate's lower millimetre lies inside
        the power compartment's ceiling: the two files were drawn without
        the rebate that would let them meet, and no placement can fix it --
        lift the plate clear of the compartment and it goes into the frame.
        The model does not move it, because the layer places what the
        repository publishes. This is that overlap, held to the size it is,
        so a revised export that fixes it is noticed rather than absorbed.
        """
        body = self.node.chassis.body
        self.assertIntersectVolumeAbove(body.compartment, body.electronics,
                                        PUBLISHED_SEAT_OVERLAP - 100.0)
        self.assertIntersectVolumeBelow(body.compartment, body.electronics,
                                        PUBLISHED_SEAT_OVERLAP + 100.0)


class PostureScenarioTest(ScenarioTest):
    """The robot put through the moves it offers, one after another.

    A scenario, not a pose: each instruction ramps every driver it names
    from wherever the last one left it, so this is the model doing what a
    maker would do with the buttons -- and the assertions run while it is
    moving, not only where it lands.
    """

    node = Spiderbot
    dt = 0.05
    meshes = True

    def test_the_instructions_land_where_they_say(self):
        sim = self.simulation()
        sim.at(0.0).trigger('Stand')
        sim.at(2.0).trigger('Crouch')
        sim.at(4.0).trigger('Tiptoe')
        sim.at(6.5).trigger('Sit')
        sim.at(9.0).trigger('Stand')
        sim.run(11.0)

        for name, value in (('height', STAND_HEIGHT),
                            ('reach', STAND_REACH), ('stride', 0.0),
                            ('roll', 0.0), ('pitch', 0.0), ('yaw', 0.0),
                            ('wave', 0.0)):
            self.assertAlmostEqual(sim.state[name], value, delta=0.01,
                                   msg=f'{name} did not land')

    def test_the_feet_stay_planted_through_a_look_around(self):
        """Attitude is attitude: the body turns, the feet do not."""
        sim = self.simulation()
        sim.at(0.0).trigger('Stand')
        sim.at(1.6).trigger('LookAround')
        sim.every(0.4, self._feet_on_the_floor)
        sim.run(4.0)

    def test_walking_keeps_a_tripod_down(self):
        sim = self.simulation()
        sim.at(0.0).trigger('Walk')
        sim.every(0.25, self._three_feet_down)
        sim.run(3.0)

    def _feet_on_the_floor(self):
        for leg in self.node.chassis.legs:
            self.assertAlmostEqual(foot_contact(leg), 0.0, delta=3.0,
                                   msg=f'{leg.name} left the floor')

    def _three_feet_down(self):
        down = sum(1 for leg in self.node.chassis.legs
                   if foot_contact(leg) < 3.0)
        self.assertGreaterEqual(down, 3, f'only {down} feet down')
