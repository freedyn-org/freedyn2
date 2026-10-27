# Sensors and accelerations

FreeDyn writes the accelerations it computes. HHT solves for the accelerations of every body
in every step, so the result channels `Rddot_*` (center of mass) and `omegadot_*` (angular) of
the bodies are exact values of the solution, not difference quotients of the velocities: they
do not depend on `output_step` and keep their peaks. (The viewer's `der()` differentiates the
output samples; with `output_step: 0.005` it smooths a bump's acceleration peaks by a few
percent.)

## Sensors

A sensor is a point on a body with a frame fixed in it: a seat, the driver's head, an
inertial measurement unit, the reference point of a motion platform. It has no effect on the
simulation.

```yaml
sensors:
  head:
    body1: chassis
    point: [1.2, 0.35, 1.1]          # global, at t = 0
    # orientation: [1, 0, 0, 0]      # the sensor axes: global at t = 0, then fixed in body1
```

Channels, in the sensor frame (except the position):

| channel | |
|---|---|
| `position_x/y/z` | global position of the point [m] |
| `velocity_x/y/z` | velocity of the point [m/s] |
| `acceleration_x/y/z` | acceleration of the point [m/s²] |
| `specific_force_x/y/z` | acceleration minus gravity, `a − g` [m/s²]: what an accelerometer measures, +9.81 upwards at rest |
| `angular_velocity_x/y/z` | angular velocity of the body [rad/s] |
| `angular_acceleration_x/y/z` | angular acceleration of the body [rad/s²] |

The acceleration of the point is `a = R̈ + ω̇ × r + ω × (ω × r)`, with `r` from the center of
mass to the point; all quantities are taken from the solved accelerations.

**The sensor axes** are the global axes at t = 0 by default, then they turn with the body: in a
vehicle model built with x forward and z up, `specific_force_x` is the longitudinal,
`specific_force_y` the lateral acceleration felt in the car. `orientation` (Euler parameters,
global at t = 0) turns the sensor frame, e.g. to the axes of a real sensor or a motion
platform. The axes of the body itself are its principal axes of inertia, usually not those
you want.

### Motion cueing

The inputs of a motion cueing algorithm are the specific force and the angular velocity (and
acceleration) at the reference point of the driver, in the vehicle frame: one sensor gives
all of them, at every output step (`output_step` equal to `step` gives every step).

## Accuracy

The accelerations satisfy the equations of motion at the end of each step. The constraints of
the joints are kept on the positions; on the accelerations they hold to the integration error:
a sensor on the axis of a pendulum's revolute joint measures a few µm/s² instead of zero, at
accelerations of 20 m/s² in the bob. In the `legacy` profile (`.fds` models, the
original HHT form) the accelerations satisfy the equations of motion in the α-weighted form of
that method.

## All keys

<!-- keys: sensor -->
