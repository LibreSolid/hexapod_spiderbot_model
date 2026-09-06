# Spiderbot simulation -- measuring the printed parts back out of the STLs.
# SPDX-License-Identifier: MIT

"""Read the SpiderBot back out of its meshes.

This repository publishes print files. An STL says what to print and nothing
about where it goes or which way it turns, so every number the simulation
places a part with had to be measured from the mesh itself. This module is
how: it is the tool the constants in `simulation/params.py` came from, and
the tool the contracts hold them to.

Two readings do most of the work.

`cylinders()` finds the turned surfaces. A pivot bore, a bearing seat, a
servo-screw hole and a rounded fork end are all cylinders, and a cylinder is
the one feature a triangle soup gives up easily: its faces meet at shallow
angles, so they fall into one smooth group, and every one of their normals is
perpendicular to the same axis -- which makes the axis the null direction of
the group's normal covariance, and the centre and radius a circle fit in the
plane across it. Whether the surface is a bore or a boss is then just whether
the normals point at the axis or away from it.

`occupancy()` finds the voids: the servo pocket in a holder, the battery bay
in the power compartment, the slot a rib drops into. It asks the mesh, on a
grid, where its material is, and prints the answer as characters, which is
deliberate -- a 20 mm servo body in a 35 mm holder is something a person
recognises at a glance and no fitting routine has to be trusted about.

Run it -- `python -m simulation.tools.probe` -- to reprint the reading that
`docs/measurements.md` records.

The cylinder and occupancy readers are the shop's, carried over from the
InMoov hand simulation unchanged; only the parts they are pointed at are new.
"""

import os

import numpy as np
import trimesh
from scipy import ndimage
from trimesh.graph import connected_components

STL_DIR = os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), 'stl')

#: Decimal places the body ordering compares centroids on -- the same
#: micrometre `StlNode` sorts a multi-body file by.
_CENTROID_PLACES = 3


def mesh(filename):
    """One printed part's mesh, in the file's own coordinates.

    Every file in `stl/` holds exactly one body, so there is no plate to
    index into; this is the whole file.
    """
    return trimesh.load(os.path.join(STL_DIR, filename), force='mesh')


def bodies(filename):
    """The file's connected bodies, in `StlNode`'s own order."""
    whole = mesh(filename)
    parts = whole.split(only_watertight=False, repair=False)
    if len(parts) == 0:
        return [whole]
    return sorted(parts, key=lambda body: tuple(
        round(float(value), _CENTROID_PLACES) for value in body.centroid))


def smooth_groups(mesh, angle=25.0):
    """Faces grouped by the surface they lie on.

    Adjacent faces belong together when the crease between them is shallow,
    which is what makes a tessellated cylinder one group and its end faces
    another.
    """
    adjacency = mesh.face_adjacency[mesh.face_adjacency_angles
                                    < np.radians(angle)]
    return connected_components(adjacency, min_len=1,
                                nodes=np.arange(len(mesh.faces)))


def fit_cylinder(mesh, faces):
    """One smooth group read as a cylinder: axis, centre, radius, extent.

    The axis is the direction every face normal is perpendicular to, i.e.
    the eigenvector of the smallest eigenvalue of the normals' covariance.
    The centre and radius are an algebraic circle fit of the group's
    vertices projected across that axis, and `residual` is how far the worst
    of them misses -- the number that says whether the group was a cylinder
    at all.
    """
    normals = mesh.face_normals[faces]
    _, vectors = np.linalg.eigh(normals.T @ normals)
    axis = vectors[:, 0] / np.linalg.norm(vectors[:, 0])

    seed = ([1.0, 0, 0] if abs(axis[0]) < 0.9 else [0, 1.0, 0])
    across = np.cross(axis, seed)
    across = across / np.linalg.norm(across)
    up = np.cross(axis, across)

    points = mesh.vertices[np.unique(mesh.faces[faces])]
    flat = np.stack([points @ across, points @ up], axis=1)
    solution, *_ = np.linalg.lstsq(
        np.c_[2 * flat[:, 0], 2 * flat[:, 1], np.ones(len(flat))],
        (flat ** 2).sum(axis=1), rcond=None)
    cx, cy, offset = solution
    radius = float(np.sqrt(offset + cx ** 2 + cy ** 2))
    residual = float(np.abs(
        np.linalg.norm(flat - [cx, cy], axis=1) - radius).max())

    along = points @ axis
    centre = cx * across + cy * up + axis * float(along.mean())

    # Hole or boss: a face on the inside of a bore has its normal pointing
    # back at the axis, one on the outside of a boss points away from it.
    radial = mesh.triangles_center[faces] - centre
    radial = radial - np.outer(radial @ axis, axis)
    radial = radial / np.linalg.norm(radial, axis=1)[:, None]
    inward = float((radial * mesh.face_normals[faces]).sum(axis=1).mean())

    return {
        'axis': axis,
        'centre': centre,
        'radius': radius,
        'residual': residual,
        'ends': (centre + axis * (float(along.min()) - float(centre @ axis)),
                 centre + axis * (float(along.max()) - float(centre @ axis))),
        'hole': inward < 0,
        'faces': len(faces),
    }


def cylinders(mesh, min_faces=8, max_residual=0.15, max_radius=60.0):
    """Every smooth group of `mesh` that really is a cylinder."""
    found = []
    for group in smooth_groups(mesh):
        group = np.asarray(group)
        if len(group) < min_faces:
            continue
        fit = fit_cylinder(mesh, group)
        if fit['residual'] > max_residual or fit['radius'] > max_radius:
            continue
        found.append(fit)
    return sorted(found, key=lambda fit: -fit['faces'])


def occupancy(mesh, plane, offset, step=0.5):
    """Where the mesh has material on one slice, as rows of characters.

    `plane` names the two axes of the slice ('xz', 'xy', 'yz') and `offset`
    is the position along the third, in the mesh's own coordinates. Local
    coordinates are printed, i.e. measured from the body's bounding-box
    minimum, because that is the frame a wrapper's `adjust` hook works in.
    """
    axes = {'x': 0, 'y': 1, 'z': 2}
    first, second = (axes[name] for name in plane)
    third = ({0, 1, 2} - {first, second}).pop()

    low, high = mesh.bounds
    rows = np.arange(0.0, high[first] - low[first] + step, step)
    columns = np.arange(0.0, high[second] - low[second] + step, step)

    grid = np.zeros((len(rows), len(columns), 3))
    grid[:, :, first] = rows[:, None]
    grid[:, :, second] = columns[None, :]
    grid[:, :, third] = offset
    inside = mesh.contains(low + grid.reshape(-1, 3)).reshape(len(rows),
                                                              len(columns))
    return rows, columns, inside


def through_windows(mesh, step=1.5, least=40):
    """The openings that go right through a part, front to back.

    A servo's niche in the palm is a window cut through the frame, and that
    is exactly what this asks for: on a grid across the part, the columns
    that meet no material anywhere between its two faces. The columns that
    answer are grouped, and each group is reported as a centre, the heading
    of its long axis, and its size -- which is all a drive unit needs to
    stand in one.

    Groups touching the outside of the part are not windows and are dropped;
    a window has to be surrounded. The grid is coarse on purpose: a niche is
    tens of millimetres across, and asking a forty-thousand-triangle palm
    about every cubic millimetre of itself costs a quarter of an hour.
    """
    low, high = mesh.bounds
    xs = np.arange(low[0], high[0] + step, step)
    ys = np.arange(low[1], high[1] + step, step)
    zs = np.arange(low[2], high[2] + step, step)
    grid = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), -1)
    points = grid.reshape(-1, 3)
    inside = np.zeros(len(points), dtype=bool)
    for start in range(0, len(points), 1500):
        inside[start:start + 1500] = mesh.contains(points[start:start + 1500])
    material = inside.reshape(len(xs), len(ys), len(zs)).any(axis=2)

    labels, count = ndimage.label(~material)
    outside = set(labels[0, :]) | set(labels[-1, :]) \
        | set(labels[:, 0]) | set(labels[:, -1])
    found = []
    for label in range(1, count + 1):
        if label in outside:
            continue
        cells = np.argwhere(labels == label)
        if len(cells) < least:
            continue
        plan = np.stack([xs[cells[:, 0]], ys[cells[:, 1]]], axis=1)
        centre = plan.mean(axis=0)
        _, _, axes = np.linalg.svd(plan - centre, full_matrices=False)
        size = np.ptp((plan - centre) @ axes.T, axis=0)
        heading = float(np.degrees(np.arctan2(axes[0][1], axes[0][0])) % 180)
        found.append({'centre': centre, 'size': size, 'heading': heading,
                      'area': len(cells)})
    return sorted(found, key=lambda window: -window['area'])


def first_clear(mesh, point, axis, radius, depth, start=3.0, stop=16.0,
                step=0.2, pitch=0.6):
    """Where a disc of `radius` and `depth` first sits clear of the material,
    moving out along `axis` from `point`.

    This is what a bolt head or a nut asks the plastic: not where the
    material ends on the pin's own line, which is inside a bore and says
    nothing, but where a face of its own size can bear.
    """
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    seed = ([1.0, 0, 0] if abs(axis[0]) < 0.9 else [0, 1.0, 0])
    across = np.cross(axis, seed)
    across = across / np.linalg.norm(across)
    up = np.cross(axis, across)

    # The sample points stop AT the disc's own radius and depth, never past
    # them: a ring half a millimetre wider than the head answers about the
    # part beside the seat rather than about the seat.
    def steps(limit):
        inner = list(np.arange(0.0, limit, pitch))
        return inner + [limit] if inner[-1] < limit else inner

    disc = []
    for along in steps(depth):
        for ring in steps(radius):
            count = max(1, int(2 * np.pi * ring / pitch))
            for angle in np.linspace(0, 2 * np.pi, count, endpoint=False):
                disc.append(along * axis + ring * (np.cos(angle) * across
                                                   + np.sin(angle) * up))
    disc = np.array(disc)

    for seat in np.arange(start, stop, step):
        points = disc + np.asarray(point, dtype=float) + seat * axis
        hit = np.zeros(len(points), dtype=bool)
        for begin in range(0, len(points), 1500):
            hit[begin:begin + 1500] = mesh.contains(points[begin:begin + 1500])
        if not hit.any():
            return round(float(seat), 2)
    return None




def print_inventory():
    """Every published file: what it is, how big, and whether it closes."""
    print('\n## the published files')
    for name in sorted(os.listdir(STL_DIR)):
        if not name.endswith('.stl'):
            continue
        body = mesh(name)
        size = body.bounds[1] - body.bounds[0]
        print(f'  {name:32s} {np.round(size, 2).tolist()!s:34s} '
              f'volume {body.volume:9.1f}  '
              f'origin {np.round(body.bounds[0], 2).tolist()}  '
              f'watertight {body.is_watertight}  '
              f'bodies {len(bodies(name))}')


def print_cylinders(filename, min_radius=0.0, max_radius=60.0, min_faces=16):
    """One part's turned surfaces, in the coordinates a placement uses.

    Local -- from the bounding-box minimum -- because that is what a reader
    can check against a photograph, and `params.py` carries the offset that
    turns it into the file's own frame.
    """
    body = mesh(filename)
    low = body.bounds[0]
    print(f'\n## {filename}: turned surfaces, local to its own minimum '
          f'{np.round(low, 2).tolist()}')
    for fit in cylinders(body, min_faces=min_faces, max_radius=max_radius):
        if not min_radius <= fit['radius'] <= max_radius:
            continue
        kind = 'hole' if fit['hole'] else 'boss'
        print(f"  r={fit['radius']:6.2f} {kind}  "
              f"axis {np.round(fit['axis'], 3).tolist()}  "
              f"centre {np.round(fit['centre'] - low, 2).tolist()}  "
              f"ends {np.round(fit['ends'][0] - low, 2).tolist()}.."
              f"{np.round(fit['ends'][1] - low, 2).tolist()}  "
              f"faces {fit['faces']}")


def print_occupancy(filename, plane, offset, step=1.0):
    """One slice of a part, as characters.

    A servo channel twenty millimetres across in a bracket thirty-five
    across is something a person recognises at a glance and no fitting
    routine has to be trusted about.
    """
    body = mesh(filename)
    rows, columns, inside = occupancy(body, plane, offset, step)
    print(f'\n## {filename}: slice {plane} at {offset}  '
          f'(rows {plane[0]} 0..{rows[-1]:.0f}, '
          f'columns {plane[1]} 0..{columns[-1]:.0f})')
    for index in range(len(rows) - 1, -1, -1):
        marks = ''.join('#' if value else '.' for value in inside[index])
        print(f'   {rows[index]:5.1f} {marks}')


def main():
    print_inventory()

    # The joints. Every hub, every bearing seat, every eight-hole disc and
    # every fork bolt the model places a part on is in one of these.
    for name in ('CoxaSide1_625ZZ.stl', 'CoxaSide2_8holes.stl',
                 'FemurSide1_625ZZ.stl', 'FemurSide2_8Holes.stl',
                 'TibiaTop.stl', 'TibiaBottomLong.stl', 'LegRib.stl',
                 'ServoJoint.stl', 'Tip.stl', 'Claw.stl'):
        print_cylinders(name)

    # The stations: the frame's own bolt holes, and the two brackets that
    # go on them.
    print_cylinders('Frame.stl', min_radius=1.5, max_radius=3.5)
    print_cylinders('FrameSideServoHolder.stl')
    print_cylinders('FrameCenterServoHolder.stl')

    # What the brackets are hollow for.
    print_occupancy('FrameSideServoHolder.stl', 'zx', 8.0)
    print_occupancy('FrameCenterServoHolder.stl', 'zx', 8.0)
    print_occupancy('Frame.stl', 'zx', 8.0, step=2.0)


if __name__ == '__main__':
    main()
