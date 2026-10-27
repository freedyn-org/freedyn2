# Numerical findings

Observations made while verifying FreeDyn 2.0 against legacy FreeDyn and against independent
references. The comparison models in `examples/compare` and the tests reproduce them.

- **Fast rotors**: with Euler parameters as coordinates, the parameters oscillate with omega/2
  and HHT damps that oscillation, so a freely spinning rotor loses speed (h = 1e-3,
  alpha = -0.1, 1 s): Negrut form -18 % at 100 rad/s, -57 % at 300 rad/s; Arnold–Bruls with
  Euler parameters -0.02 % and -1 %, both -100 % at 1000 rad/s. Under a driving torque the
  rotor saturates. The legacy `ModifyNewmark` (Newmark on the angular velocity) removes the loss
  up to omega h ~ 1 and is reproduced in the `legacy` profile. The Lie group body has no loss
  at all (0 up to 3000 rad/s at h = 1e-3, orientation error 2e-13 rad).
- With Euler parameters, `alpha = 0` (trapezoidal rule) is weakly unstable: the hidden
  constraint `theta . thetadot` carries an undamped `(-1)^n` mode that grows exponentially (factor
  ~10 per 2 s in the example, faster for smaller h). The Lie group body has no normalization
  constraint and no such mode (energy drift 3e-7 after 10 s at alpha = 0).
- With `alpha < 0` the force-weighted (Negrut) form, which the legacy kernel uses too, is only
  **first order** on rotating bodies (hence `arnold_bruls` is the default):
  the gyroscopic part of `h(q, qd)` is alpha-weighted while `M(q) qdd` is not. Energy loss and the
  precession error halve with h. The Arnold–Brüls form of HHT (equations at `t_{n+1}`,
  alpha only in the Newmark pseudo-acceleration) is second order: at alpha = -0.1 the energy
  loss over 10 s drops from 1.8e-3 to 8e-8 and the precession error from 1.7e-2 to 1.5e-4.
- **`alpha = 0` with joints is unstable** for both bodies, as known for the trapezoidal rule with
  index-3 constraints (Cardona and Geradin 1989): a `(-1)^n` mode of the multipliers and the
  accelerations grows until the run breaks down (pendulum: after ~8 s in FreeDyn 2.0; legacy
  FreeDyn crashes on the same model after 2.75 s). A little numerical damping removes it:
  `alpha = -0.01` keeps the pendulum's energy to 9e-6 and its period to 3.5e-6 of the elliptic
  integral solution. The generator warns about `alpha = 0` in models with joints.
- **Legacy line-of-sight convention**: `In-line-of-sight` points from body 1 to body 2 between two
  bodies, but from the ground point to the body for body-ground forces. FreeDyn 2.0 always uses
  "from point 1 to point 2" (positive = tension); the `.fds` importer negates body-ground
  expressions.
- **Legacy `AKISPL` is an alias of `SPL`**: it uses the interpolation type of the spline data
  object, not necessarily Akima (`MBS_FunctionExpression`); the importer follows that.
- **Legacy angle measures count revolutions** with state updated at every evaluation;
  FreeDyn 2.0 keeps the last accepted value as element state (`post`), so Newton iterations and
  rejected steps cannot change it. Both agree as long as an angle changes by less than pi per step.
- **Legacy differentiates driver motions numerically** for its activation polynomial; FreeDyn
  2.0 uses exact derivatives (difference ~1e-10 in the comparison models). Legacy FreeDyn also
  crashed (Pardiso, forward/backward substitution) on a TimeMotion model with a body hung on a
  body-body force only; the comparison model keeps that body on a prismatic joint.
- **Newton criterion on the velocities only** (FreeDyn 2.0, fixed): the `velocity` criterion
  took the maximum over the whole correction, multipliers included, and so compared the
  round-off of the multiplier corrections (eps |lambda| times the condition number) with a
  velocity tolerance. On the car model of `examples/scaling` this stopped Newton from
  converging once the car stood more than about 9 m from the origin. It now uses the
  acceleration part only, as documented.
- **Legacy `.fds` input pitfalls** found with the car model: `STEP[...]` takes numbers only (no
  parameter expressions); a `DataObject Parameter` must come before the expressions that use it
  (expressions are evaluated while reading, an undefined parameter is 0 and `x/0` aborts with
  "QNAN"); with a relative model path legacy FreeDyn crashes (segmentation fault) when it
  writes its result files, the comparison scripts pass absolute paths. Inside `freedyn2 compare`
  the refined reference run of the car model ends at once with `SIM_ERROR` (exit code 4) in
  output mode `no`; the same input runs through in mode `screen`, and in mode `no` when started
  alone. `legacy/benchmark.py` repeats a failed refined run in mode `screen`.
- **Linearize about an equilibrium, not about the initial state** (stability of a two-wheeler):
  the initial state (preloads and tire deflections from the static wheel loads)
  is consistent but not yet settled; its constraint forces are up to twice the settled ones
  and enter the stiffness through B^T lambda. Linearized there, a spurious lateral mode of
  +5.7 1/s appears, which the hands-off simulation does not show; after 1 s of straight
  running it is gone and the capsize eigenvalue (+0.66 1/s at 15 m/s) matches the simulated
  lean growth. Two further pitfalls: spinning wheels must be described in a non-spinning frame
  (otherwise the equations depend on the wheel angle), and the lateral position must be left
  out as a cyclic coordinate (with the heading it forms a defective double eigenvalue 0 that
  round-off splits into +-sqrt(eps)).
