# Solver and real time

```yaml
solver: {type: HHT, alpha: -0.1, end_time: 10.0, step: 0.001, output_step: 0.01}
```

FreeDyn 2.0 integrates with the **HHT-α method** (Hilber–Hughes–Taylor) on the Lie group of
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

## Real-time mode

```yaml
solver: {type: HHT, end_time: 10, step: 0.001, newton: fixed, newton_iterations: 2}
```

With `newton: fixed` every step does exactly `newton_iterations` corrections, with a fixed
pivot order: every step costs the same, for hardware-in-the-loop and driving simulators. At
1 ms the Newton error is far below the discretization error (10⁷ times at 2 iterations).
Median cost per step at two iterations: a car with simple tires 164 µs (one core).

## Linear solvers and the command line

`freedyn2_run` takes `--linear-solver klu|sparselu|dense`, `--step-log FILE` and
`--timing-log FILE` for diagnosis; see [Command line](../cli.md).
