# Measures, expressions, splines, drivers

## Measures

A measure is a named scalar of the state – a distance, a displacement component, an angle, a
velocity – used in [expression forces](forces.md#expression-force-and-torque) and drivers by its
name. They are not result channels themselves; to plot one, write it as a
[formula channel](../viewer.md#plots) in the viewer, look at the force that uses it, or make it
an [output](#outputs).

```yaml
measures:
  z_wheel: {type: displacement, body1: wheel, component: z, offset: initial}  # travel from the start
  vz_wheel: {type: velocity, body1: wheel, component: z}
  gap: {type: distance, body1: a, point1: [0, 0, 1], body2: b, point2: [0, 0, 2]}
  steer: {type: angle, body1: knuckle, body2: carrier, orientation: [1, 0, 0, 0]}  # about z of the frame
  lean: {type: direction, body1: frame, vector: [0, 0, 1], component: y}         # vertical in the body frame
```

| type | value |
|---|---|
| `displacement` / `velocity` | a component of P1 − P2 (V1 − V2), in `frame` (global or a body) |
| `distance` / `distance_rate` | \|P1 − P2\| and its rate |
| `angle` / `angular_rate` | rotation of body1 about the z axis of `orientation` fixed on body2 (continuous: counts turns) |
| `direction` | a component of a global vector in a frame (default body1), e.g. the lean of a vehicle |
| `angular_velocity` | a component of ω1 − ω2 in `frame` |

`offset: initial` subtracts the initial value; a number subtracts that number.

<!-- keys: measure -->

## Expressions

Force laws, driver motions and formulas share one small language (it is parsed, never
executed):

- numbers, `+ - * / ^` (or `**`), unary minus, parentheses;
- names: `t` (time), `pi`, the parameters, the [runtime inputs](model-file.md#runtime-inputs)
  and the measures (both not in formulas), the splines as
  functions: `damper_curve(v)`;
- functions: `sin cos tan asin acos atan atan2 sinh cosh tanh exp log sqrt abs sign min max`;
- `step(x, x0, y0, x1, y1)`: smooth step from y0 (x ≤ x0) to y1 (x ≥ x1) – a 9th order polynomial;
  `step(x, x0, y0, x1, y1, "cosine")` or `"hard"` (the legacy FreeDyn STEP);
- `if_else(condition, a, b)` with `< <= > >= == !=`; conditions multiply as *and*:
  `if_else((t > 1) * (t < 2), 1, 0)`.

Everything is differentiated exactly (CasADi); there are no numerical derivatives.

## Splines

Named curves y(x), called like functions in expressions: a damper characteristic, a measured
road, a motor map.

```yaml
splines:
  damper_curve: {x: [-1, -0.2, 0, 0.2, 1], y: [-3000, -900, 0, 600, 1500]}   # [m/s] -> [N]
  road: {file: data/road.txt, columns: [0, 1], header_lines: 1}             # from a table
forces:
  damper: {type: expression_force, body1: wheel, body2: body, point1: [0, 0, 0.3], point2: [0, 0, 0.65],
           force: damper_curve(v_rel)}
```

`type: akima` (default: no overshoot, linear beyond the ends) or `zcc` (the legacy FreeDyn
spline).

<!-- keys: splines -->

## Outputs

An output is a named expression of measures, parameters and [runtime
inputs](model-file.md#runtime-inputs), written as the result channel `<name>.value`. It acts on
nothing. A program that runs the model and reads its channels (`fd::System::outputs`) gets
derived quantities, a lean angle, an engine speed, a suspension travel, without knowing the
model's geometry.

```yaml
measures:
  lean: {type: direction, body1: frame, vector: [0, 0, 1], component: y}   # sin(lean)
  w_crank: {type: angular_rate, body1: crank, body2: frame, orientation: [0.7071068, -0.7071068, 0, 0]}
outputs:
  lean_deg: asin(lean) * 180 / pi
  rpm: w_crank * 30 / pi
```

Each output must use a measure; its name must differ from the bodies, joints, forces, sensors
and drivers.

## Drivers

A driver prescribes a measure – a displacement, distance or angle – as a function of time: a
constraint m(q) − f(t) = 0. With a [runtime input](model-file.md#runtime-inputs) in f the motion
is set from outside while the model runs (`motion: x_set`). The reaction is a result channel (`<driver>.lambda`).

```yaml
measures:
  phi: {type: angle, body1: rotor, orientation: [1, 0, 0, 0]}
drivers:
  spin: {measure: phi, motion: w * t}                      # a constant speed
  lift: {measure: z_table, motion: 0.1 * sin(2 * pi * t), ramp: 0.5}   # from rest, smoothly
```

Without `ramp` the motion must start where the measure is at t = 0. With `ramp: Ta` a smooth
polynomial joins the initial value at rest to the motion at Ta.

<!-- keys: drivers -->
