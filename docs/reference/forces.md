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

**Characteristic curves.** `stiffness` and `damping` can each be the name of a
[spline](measures.md#splines): s = k(l − l0) + d(l̇) + f0, k the force over the deflection,
d the force over the rate. A number stays a straight line. This is a library type like the
linear spring: no model plugin, no compiler, any number of points.

```yaml
splines:
  progressive: {x: [-0.05, 0, 0.03, 0.06], y: [-1500, 0, 900, 3200]}   # [m] -> [N]
  damper: {x: [-1, -0.1, 0, 0.1, 1], y: [-2500, -400, 0, 900, 4000]}    # [m/s] -> [N]
forces:
  suspension: {type: spring_damper, body1: wheel, body2: body, point1: [0, 0, 0.3],
               point2: [0, 0, 0.65], stiffness: progressive, damping: damper}
```

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

As for the spring-damper, `stiffness` and `damping` can be splines: τ = −k(φ − φ0) − d(φ̇) + τ0,
k the torque over the angle, d over the rate (library type, no plugin).

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

## Curve force and torque

A force or torque that is a curve ([spline](measures.md#splines)) of a
[measure](measures.md): s = a · curve(b · m + c). `curve_force` acts along a direction like an
expression force (`direction`, `frame`, `point1`, `point2`; default the line of sight),
`curve_torque` about an `axis` (`frame` global or body) like an expression torque. Library
types: no model plugin, no compiler, any number of points.

```yaml
splines:
  bump_stop: {x: [-0.05, -0.02, 0, 0.01], y: [-8000, -900, 0, 0]}   # [m] -> [N]
measures:
  travel: {type: displacement, body1: wheel, body2: body, component: z, frame: body, offset: initial}
forces:
  stop:
    type: curve_force
    body1: wheel
    body2: body
    measure: travel
    curve: bump_stop
    direction: [0, 0, 1]
    frame: body
    scale: 1.0             # a; default 1
    argument_scale: 1.0    # b; default 1
    argument_offset: 0.0   # c; default 0
  twist: {type: curve_torque, body1: arm, body2: body, measure: phi, curve: torsion_curve,
          axis: [0, 1, 0], frame: body}
```

The measure must act between the force's two bodies (either order; for a force on the ground,
of body1 against the ground): displacement, velocity, distance, distance rate, angle
(continuous), angular rate, angular velocity, in any frame of those bodies. Otherwise (a
measure of other bodies, another angle range) the same law is built as an expression force or
torque in the model plugin. Channels: `.value` (s), `.force` or `.torque`, `.measure` (m).

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

## Contact

A sphere on body1 against a plane or another sphere on body2 or the ground: the bodies touch
when the sphere penetrates, `δ > 0`, and then push each other apart (a penalty force) and rub
(friction). For a ball, a foot, the end of a stop, a pin in a slot, the corners of a box (one
sphere each).

```yaml
forces:
  floor:
    type: contact
    body1: ball
    radius: 0.05            # the sphere on body1, centered at point1 (default: its center of mass)
    normal: [0, 0, 1]       # sphere_plane (default): the plane through point2, towards body1
    stiffness: 1.0e+7       # k
    exponent: 1.5           # e (Hertz, the default)
    damping: 2.0e+4         # d: energy lost per impact
    friction: 0.4           # mu (sliding)
    static_friction: 0.5    # mu_s at slip 0 (default: friction)
```

`geometry: sphere_sphere` with `point2` (the center on body2 / the ground) and `radius2` puts a
second sphere in place of the plane.

- **Normal force** (Hunt and Crossley): `Fn = max(0, k δ^e + d δ^e δ̇)`, acting along the normal
  at the middle of the overlap. It starts from 0 at the touch (no jump), and the damping only
  acts during the contact. For small damping the coefficient of restitution is about
  `1 − (2/3) (d/k) v` for an impact at speed `v`: choose `d` from the energy an impact should lose.
  `k` for steel spheres follows from Hertz' law: `(4/3) E* sqrt(R*)`.
- **Friction** against the slip velocity `v_t` of the touching points: `F_t = −μ(|v_t|) Fn
  v_t / sqrt(|v_t|² + v_ε²)`, Coulomb friction made smooth below `slip_velocity` `v_ε` (default
  1 mm/s) so that the solver can take it; `μ(v) = μ + (μ_s − μ) exp(−(v/v_s)²)` with
  `stribeck_velocity` `v_s` gives a static peak. A sphere set sliding starts to roll as it
  should (at 5/7 of its speed, test `test_contact.py`).
- **Events**: the runtime locates the moment each contact begins and ends (to `event_tol`,
  default 1e-7 s) and ends a step there, so an impact starts with no penetration however large
  the step. `freedyn2_run --event-log FILE` lists them; the run reports how many.
- **Outputs**: `penetration`, `penetration_rate`, `normal_force`, `friction_force`,
  `slip_velocity`, `point` (the contact point), `force` (on body1).
- **In the viewer** the contact geometry is drawn on its bodies: the sphere as three great
  circles (they turn with the body) and a faint shell, the plane as a square under the sphere
  with its normal, or the second sphere. While the bodies touch it is drawn stronger, with the
  contact point; with *force arrows* on, the contact force is drawn there (on both bodies). It
  shows a sphere placed off the body or a normal pointing the wrong way at a glance.
- **The step**: during a contact the step must resolve it. A Hertz contact (`e = 1.5`) lasts about
  `3.22 (m/k)^(2/5) v^(-1/5)` (m the mass, v the impact speed; the ball of the tests: 2.9 ms); 20 to 30 steps across it are enough, or `adaptive: true`. With a fixed
  step an elastic impact keeps its energy very well (the scheme is time-reversible); with variable
  steps the energy error is that of the tolerance `eps`.

## Tire

See [Tires and roads](tires-roads.md).

## All keys

<!-- keys: force -->
