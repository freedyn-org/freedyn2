# Hardpoints and frames

Every point of a model is given globally at t = 0, where it is used. When several elements
share a point (the outer ball joint of a control arm, the spring's lower mount), name it once
under `points` and use the name:

```yaml
points:
  uca_outer: [0.0, 0.68, 0.52]
  lca_outer: [0.0, 0.72, 0.13]
  spring_bottom: {from: lca_outer, offset: [0.0, -0.10, 0.05]}
joints:
  upper_ball: {type: spherical, body1: carrier, body2: uca, location: uca_outer}
forces:
  spring: {type: spring_damper, body1: lca, point1: spring_bottom, point2: spring_top, stiffness: 60000}
```

A name stands for the numbers in every point field: `location` (joints, bushings), `point`,
`point1`, `point2` (forces, measures, sensors) and `position` (bodies). Move the point and
everything that uses it moves along.

## Points

| form | |
|---|---|
| `[x, y, z]` | global at t = 0; numbers or [formulas](model-file.md#parameters-and-formulas): `[0.15, 0.40, =z_uca]` |
| `{mirror: name}` | the point mirrored at the x-z plane (`y` negated): the other side of a vehicle |
| `{from: name, offset: [dx, dy, dz]}` | another point moved by an offset |
| `{in: body or frame, local: [x, y, z]}` | in the frame of a body or of a named frame (below) |

Points defined by others may come in any order; a loop is an error.

## Points in a body

A part drawn in its own coordinates – a wheel carrier from CAD, a subframe, a seat in the
cabin – is easiest described there. `in:` takes a body or a frame:

```yaml
frames:
  carrier_cad: {origin: [0.0, 0.71, 0.32], z: [0, 0, 1], x: [1, 0, 0]}   # the part's CAD origin
points:
  tie_outer: {in: carrier_cad, local: [-0.12, -0.01, -0.07]}
  caliper: {in: carrier_cad, local: [0.05, 0.02, 0.12]}
  head: {in: chassis, local: [0.4, 0.35, 0.6]}     # from the chassis' center of mass
```

- **In a frame**: the coordinates are along its axes from its origin. Give a part a frame for
  its own origin and axes (as in CAD) and every point of the part in it; move or turn the frame
  and the part's points go along.
- **In a body**: the body frame, i.e. its center of mass and its principal axes of inertia, at
  t = 0. With the default orientation `[1, 0, 0, 0]` these are the global axes, so `local` is
  the offset from the center of mass; with a turned body they turn with it.
- A body's `position` and `orientation` may themselves be a point and a frame. A point in a
  body that places the body is a loop and reported as such.

Like everything else the result is a point at t = 0; the body carries it from there.

## Frames

A frame has an origin and an orientation. Its name stands for the orientation in every
`orientation` field (joints, bushings, torsion springs, sensors, measures, bodies) and for its
origin in a point field:

```yaml
frames:
  lca_axis: {origin: lca_front, z: [lca_rear, lca_front]}    # z from one point to another
  wheel_axis: {origin: wheel_center, z: [0, 1, 0]}           # z along a direction
  sensor_frame: {origin: head, orientation: [1, 0, 0, 0]}    # Euler parameters
joints:
  lca_pivot: {type: revolute, body1: lca, location: lca_axis, orientation: lca_axis}
```

| key | |
|---|---|
| `in` | a body or frame: `origin`, `orientation` and `[x, y, z]` directions are in its frame (point names stay global) |
| `origin` | a point name or `[x, y, z]`. Default `[0, 0, 0]` |
| `z` | the direction of the z axis (the axis of a revolute or cylindrical joint, a prismatic joint's direction): `[x, y, z]` or two point names `[from, to]` |
| `x` | where the x axis points, made normal to z: `[x, y, z]` or two point names. Default the global x axis (y if z is along x) |
| `orientation` | Euler parameters instead of `z` and `x` |

## Tuning hardpoints

A hardpoint is a number like any other: in the viewer, tick **hardpoints** under *View*, click
one, and tune its coordinates; the joints, springs and frames that use it follow. A formula
in a point (`[0.15, 0.40, =z_uca]`) makes it a parameter with a slider, a fan (≋) and a root
locus (⌁) – a design study of the geometry.

The inspector of a hardpoint lists what uses it. Points and frames nothing uses are reported
by the checks (*points and frames not used*).

## Example

`freedyn2 examples double_wishbone`: a double-wishbone front suspension on a rig, built from
hardpoints, with a tire spring on a road with a bump.

## All keys

<!-- keys: frame_definition -->
