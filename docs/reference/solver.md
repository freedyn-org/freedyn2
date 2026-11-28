# Solver and real time

```yaml
solver: {type: HHT, alpha: -0.1, end_time: 10.0, step: 0.001, output_step: 0.01}
```

freedyn2 integrates with the **HHT-α method** (Hilber–Hughes–Taylor) on the Lie group of
the bodies, at a fixed step, solving the nonlinear equations of each step with Newton's
method and a sparse LU (KLU). The equations and their Jacobians are exact (generated).

<!-- keys: solver -->

## Choosing the values

- **`step`**: small enough for the fastest motion you care about – about a tenth of its period
  or less (1 ms for a vehicle with wheel hop at 12 Hz is plenty; stiff bushings may want less).
  The run reports the Newton iterations per step; more than 3 or 4 on average means the step
  is large for the model.
- **`alpha`** between −1/3 and 0 damps high-frequency numerical noise; −0.1 (the default)
  damps what the step cannot resolve and leaves the physics alone. Models with joints need
  α < 0.
- **`output_step`**: how often a row is written; a multiple of `step`. It does not change the
  accuracy, only the size of the result file.

Any of them can be overridden on the command line without editing the model:
`freedyn2 run model.yaml -- --end-time 2 --step 1e-4`.

## Numerical damping: HHT or generalized-alpha

`alpha` (HHT, the default −0.1) and `rho_inf` (generalized-alpha) both set how strongly the
integrator damps what the step cannot resolve: `rho_inf` is the factor by which such a motion
shrinks per step (0: at once, 1: not at all; HHT's is (1 + α)/(1 − α), 0.82 at −0.1).
Generalized-alpha reaches that damping with less damping of the motion the step does resolve
(on the pendulum a third of HHT's at the same `rho_inf`), so it is the better choice for long
runs and lightly damped structures:

```yaml
solver: {type: HHT, rho_inf: 0.8, end_time: 10, step: 0.001}
```

Give one of the two, not both. With joints keep `rho_inf` below 1, as α below 0.
On the command line: `-- --rho-inf 0.8`.

## Step size control

```yaml
solver: {type: HHT, end_time: 10, step: 0.01, adaptive: true, eps: 1.0e-6}
```

With `adaptive: true` the step follows the accuracy `eps` (absolute and relative to the size of
each coordinate): small where the motion is fast, up to `step` where it is slow, never below
`h_min`. Steps end exactly at the output times, so `output_step` need not be a multiple of `step`.
The error of the results goes with `eps^(2/3)`: a hundred times smaller `eps` gives about 20
times smaller errors (and about 4 times more steps). The controller is Gustafsson's PI
controller (`controller: pi`); models converted from FreeDyn keep FreeDyn's own
(`controller: legacy`). On the command line: `-- --adaptive pi --tolerance 1e-6`.

## The initial state

Before the first step the runtime makes the initial state consistent and says on its output
what that took:

- **Redundant constraints** (an overconstrained mechanism, e.g. a planar loop of revolute
  joints, which is overconstrained in space): constraints that repeat others are found in
  model order and deactivated. They carry no force (their `lambda` is 0), the others carry
  the load, and the motion is the same. The run lists them (`redundant constraints,
  deactivated: pin_b.ax ...`). If one of them is violated while the others hold, the
  mechanism does not fit together (hinge axes of a loop not parallel) and the run stops with
  that constraint named. Example: [`parallelogram.yaml`](../examples.md).
- **Assembly**: positions that violate the joints or drivers are corrected by Newton's method,
  the smallest change in the mass metric (light bodies move more than heavy ones), and the run
  reports the largest change. Positions that cannot be assembled stop the run.
- **Static equilibrium** with `statics: true`:

  ```yaml
  solver: {type: HHT, end_time: 5, step: 0.001, statics: true}
  ```

  the positions move until gravity, springs and preloads balance, the bodies at rest
  (initial velocities are not used, drivers hold their values at t = 0), and the motion
  starts from there: a vehicle settled on its springs, a mechanism hanging in its stable
  position. On the command line for one run: `freedyn2 run model.yaml -- --statics`. A body
  free to fall has no equilibrium and stops the run.

## Real-time mode

```yaml
solver: {type: HHT, end_time: 10, step: 0.001, newton: fixed, newton_iterations: 2}
```

With `newton: fixed` every step does exactly `newton_iterations` corrections, with a fixed
pivot order: every step costs the same, for hardware-in-the-loop and driving simulators. At
1 ms the Newton error is far below the discretization error (10⁷ times at 2 iterations).
Median cost per step at two iterations: a car with simple tires 164 µs (one core).

## Models converted from FreeDyn

A model converted from a legacy `.fds` ([`freedyn2 convert`](../cli.md#convert)) keeps the step
size control of legacy FreeDyn: `adaptive: true`, the error tolerance `eps`
(`GlobalErrorTolerance`), `step` as the largest step (`SolverMaxTimeStep`; `output_step` need
not be a multiple of it), `h_min`. Converted with `--profile legacy` it also has `form: negrut`,
`newton: legacy`, `error_norm: legacy`, `vproj_tol` and Euler-parameter bodies
(`element: rigid_body_euler`), and runs as legacy FreeDyn does. For a fixed step, remove
`adaptive` and `eps` and choose `step`; `--fixed-step` on the command line does the same for
one run.

## Linear solvers and the command line

`freedyn2_run` takes `--linear-solver klu|sparselu|dense`, `--step-log FILE` and
`--timing-log FILE` for diagnosis; see [Command line](../cli.md).
