# Forces

Every force acts on `body1`; forces between two bodies act on `body2` (default: the ground)
with the opposite sign. Points and frames are given globally at t = 0.

## Spring-damper

Along the line between `point1` on body1 and `point2` on body2: s = k (l − l0) + d l̇ + f0,
**tension positive**.

```yaml
forces:
  suspension:
    type: spring_damper
    body1: wheel
    body2: body
    point1: [0, 0, 0.3]
    point2: [0, 0, 0.65]
    stiffness: 25000      # k [N/m]
    damping: 1800         # d [N s/m]
    length: 0.35          # l0; default: the initial distance (no force at t = 0)
    preload: -3924        # f0 [N]; negative: compression
```

Channels: `suspension.length`, `.velocity`, `.force`.

## Torsion spring

About the z axis of a frame (`orientation`) fixed on body2 or the ground:
τ = −k (φ − φ0) − d φ̇ + τ0 on body1.

```yaml
  coil:
    type: torsion_spring
    body1: disk
    orientation: [1, 0, 0, 0]     # axis z
    stiffness: 50                 # [N m/rad]
    damping: 0.2
    angle: 0                      # phi0
```

Channels: `.angle`, `.rate`, `.torque`.

## Bushing

Six springs and dampers between body1 and body2 in a frame (`location`, `orientation`): three
translations and three rotations, each with its own stiffness and damping (a number for all
three axes, or a list of three).

```yaml
  mount:
    type: bushing
    body1: engine
    body2: chassis
    location: [0.2, 0, 0.4]
    stiffness: [2.0e5, 2.0e5, 5.0e5]     # [N/m] along the frame axes
    damping: 300
    rot_stiffness: [3000, 3000, 1000]    # [N m/rad]
    rot_damping: 5
```

Channels: `.displacement_x/y/z`, `.rotation_x/y/z`, `.force_x/y/z`, `.torque_x/y/z` (in the
bushing frame on body2).

## Constant force and torque

```yaml
  push: {type: force, body1: block, point: [0, 0, 0.1], vector: [10, 0, 0], frame: global}
  twist: {type: torque, body1: block, vector: [0, 0, 2], frame: body}   # turns with the body
```

Channels: `.force_x/y/z` or `.torque_x/y/z`, global.

## Expression force and torque

A magnitude from a law you write – an [expression](measures.md#expressions) of time, the
parameters, [measures](measures.md) and splines – along a direction (or about an axis) on body1,
and opposite on body2. These are compiled into a model plugin (a C compiler is needed); the
derivatives are exact.

```yaml
measures:
  z: {type: displacement, body1: mass, component: z, offset: initial}
  vz: {type: velocity, body1: mass, component: z}
forces:
  actuator:
    type: expression_force
    body1: mass
    direction: [0, 0, 1]            # or line_of_sight (point1 to point2, the default)
    force: -k * z - d * vz + F0 * sin(w * t)
  motor:
    type: expression_torque
    body1: rotor
    body2: stator
    axis: [0, 0, 1]
    frame: body                     # the axis turns with body1
    torque: T_max * step(t, 0, 0, 0.5, 1) - c * w_rel
```

Channels: `.value` (the law), `.force_x/y/z` or `.torque_x/y/z`.

### Direction: fixed in space or in body1

| | the direction `e` |
|---|---|
| no `direction` (`line_of_sight`) | from `point1` on body1 to `point2` on body2 (or the ground), at every instant |
| `direction: [x, y, z]` | given globally at t = 0, **fixed in space** (the default, `frame: global`) |
| `direction: [x, y, z]`, `frame: body` | given globally at t = 0, then **fixed in body1**: it turns with body1 |

A direction is never carried by body2. The same holds for the `axis` of an `expression_torque`
and for the `vector` of a constant `force` or `torque` (with `frame: body` its components are
taken in the body1 frame).

The force acts as `+value · e` on body1 at `point1` and `−value · e` on body2 at `point2`. With
a fixed direction and points that do not lie on one line along `e`, the pair of forces also
makes a moment: action and reaction are equal and opposite, but not collinear.

## Tire

See [Tires and roads](tires-roads.md).

## All keys

<!-- keys: force -->
