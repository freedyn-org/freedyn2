# Result channels

The result is a CSV file, one row per output time, one column per channel. Channel names are
`<element>.<output>`.

## Bodies

| channel | |
|---|---|
| `R_x`, `R_y`, `R_z` | position of the center of mass [m] |
| `theta0` … `theta3` | orientation (Euler parameters) |
| `Rdot_x`, `Rdot_y`, `Rdot_z` | velocity of the center of mass [m/s] |
| `omega_x`, `omega_y`, `omega_z` | angular velocity, global components [rad/s] |
| `omega_body_x`, … | angular velocity, body components |
| `T` | kinetic energy [J] |
| `H_x`, `H_y`, `H_z` | angular momentum about the center of mass |
| `theta_norm` | norm of the Euler parameters (1; a check of the integration) |
| `Rddot_x`, `Rddot_y`, `Rddot_z` | acceleration of the center of mass [m/s²] |
| `omegadot_x`, … / `omegadot_body_x`, … | angular acceleration, global / body components [rad/s²] |

The accelerations are those the solver computes in every step (HHT solves for them), not
difference quotients of the velocities: they do not depend on `output_step` and keep their
peaks. For the motion felt at a point (a seat, a sensor, the driver's head) use a
[sensor](sensors.md).

## Joints and drivers

One constraint force per fixed direction of the joint frame: `hinge.dx.lambda`,
`hinge.dy.lambda`, `hinge.dz.lambda` (forces [N]), `hinge.ax.lambda`, … (torques [N m]).
Drivers: `<driver>.lambda`, the force or torque that prescribes the motion.

## Forces

| type | channels |
|---|---|
| `spring_damper` | `length`, `velocity`, `force` (tension positive) |
| `torsion_spring` | `angle`, `rate`, `torque` |
| `bushing` | `displacement_x/y/z`, `rotation_x/y/z`, `force_x/y/z`, `torque_x/y/z` (bushing frame) |
| `force` / `torque` | `force_x/y/z` / `torque_x/y/z` (global) |
| `expression_force` / `expression_torque` | `value` (the law), `force_x/y/z` / `torque_x/y/z` |
| `tire` | `force_x/y/z` (contact frame), `slip0`, `slip1`, `friction`, `penetration`, `contact_x/y/z`, `road_height` |

Measures are not channels; they enter the forces that use them.

## Sensors

`position_x/y/z` (global), `velocity_*`, `acceleration_*`, `specific_force_*`,
`angular_velocity_*`, `angular_acceleration_*` in the sensor frame. See
[Sensors](sensors.md).

## Reading the results

- In the [viewer](../viewer.md): click a channel to plot it; formulas for derived channels.
- With the assistant: `channels` lists them, `results` gives statistics and a table.
- In Python: `pandas.read_csv("model.csv")`, or `fdgen.view.scene.read_results(path)` for a
  dictionary of numpy arrays.
