# Coming from FreeDyn

FreeDyn 2.0 is the successor of the FreeDyn freeware. If you have worked with the legacy
FreeDyn binaries, unpacked into a folder, this page says what changes, what stays, and how to
run your existing `.fds` models in FreeDyn 2.0.

## What changes

| legacy FreeDyn | FreeDyn 2.0 |
|---|---|
| binaries downloaded and unpacked into a folder | one installer (`FreeDyn2-setup.exe`), or `pip` / `uv` ([Getting started](getting-started.md#install)) |
| model files in `.fds` | YAML model files ([the model file](reference/model-file.md)); `.fds` files keep working |
| result files next to the model | one CSV file per run, one column per [channel](reference/results.md) |
| | the app window and the [viewer](viewer.md): 3D view, plots, checks, tuning, runs side by side |
| | the [command line](cli.md): `freedyn2 run`, `view`, `check` |
| | an [assistant](assistants.md) can build, run and examine models |
| Euler-parameter bodies, the Negrut form of HHT | Lie group bodies, the Arnold–Brüls form of HHT; the legacy algorithms on request (`--profile legacy`) |

**Your `.fds` files keep working.** `freedyn2 run`, `freedyn2 view` and `freedyn2 check` read
them as they are. By default FreeDyn 2.0 solves them with its own methods, with the step size
control keeping the model's `GlobalErrorTolerance` and `SolverMaxTimeStep`. With
`--profile legacy` it uses the legacy algorithms instead (Euler-parameter bodies, Negrut HHT,
the legacy Newton rule and step size control): then both programs take the same steps and
agree to round-off.

The importer reads the blocks FreeDyn 2.0 supports: `Body Rigid_EulerParameter_PAI`,
`Constraint Generic`, `Constraint TimeMotion`, `Force Gravity`, `Force GenericForce` and
`GenericTorque`, `Measure Translational` and `Rotational`, `DataObject Parameter` and
`Spline_1D`, `Simulation Dynamic`, `Solver HHT` and `HHT_Negrut`. Any other block is rejected
with its name, so a model is never simulated with parts missing.

## Side by side

FreeDyn 2.0 and legacy FreeDyn can stay on the same machine. The installer installs for the
current user into its own folder, with its own Python, and does not touch the folder of the
legacy FreeDyn. The component *associate `.fds`* is off by default, so what opens `.fds` files
now keeps opening them; switch it on if a double click should open FreeDyn 2.0.

## A first session with an existing model

1. Install FreeDyn 2.0 ([Getting started](getting-started.md#install)) and open
   **FreeDyn 2.0 Command Prompt** from the start menu (or any prompt, if `freedyn2` is on the
   PATH).
2. Go to the folder of your model. FreeDyn 2.0 does not change the `.fds` file; `run` writes its
   results as a CSV next to it.

    ```sh
    cd C:\models\crank
    ```

3. Check it. This reads the model, reports blocks the importer does not support, and lists
   what it finds: degrees of freedom, total mass, likely slips.

    ```sh
    freedyn2 check crank.fds
    ```

4. Run it with the legacy algorithms; the results should be those of legacy FreeDyn.

    ```sh
    freedyn2 run crank.fds --profile legacy -o crank_legacy.csv
    ```

5. Run it with FreeDyn 2.0's methods (the default) and look at it:

    ```sh
    freedyn2 run crank.fds -o crank_v2.csv
    freedyn2 view crank.fds
    ```

    The viewer (or `freedyn2 gui crank.fds` for the app window) shows the bodies as boxes of
    their mass and inertia, as an `.fds` has no geometry, plays the results and plots the
    channels. `freedyn2 view crank.fds crank_legacy.csv` shows the results of the legacy run.

6. Compare the two runs. They differ by the accuracy of the methods, usually little; where
   they differ much, read the [findings](findings.md) (fast rotors, `alpha = 0`, angle
   measures).

Bodies and joints keep the names of the `.fds` (`BodyName`, `Name` of a constraint), so the
channels are `crank.R_x`, `crank.omega_z` and so on.

## Moving a model to YAML

There is no converter from `.fds` to YAML yet. A model is moved by writing its YAML file:
by hand, following the [model file reference](reference/model-file.md), or with an
[assistant](assistants.md), which can read the `.fds` text, write the YAML and run both to
compare. The concepts map directly:

| `.fds` | YAML |
|---|---|
| `Body Rigid_EulerParameter_PAI`: `Mass`, `Inertia`, `LocationCenterMass`, `OrientationPAI` | [`bodies`](reference/bodies.md): `mass`, `inertia`, `position`, `orientation` (a quaternion instead of the rotation matrix) |
| `InitialVelocityVector`, `InitialOmegaVector` (global) | `velocity`, `angular_velocity` of the body |
| `Constraint Generic` with `dxFixed` … `azFixed` | [`joints`](reference/joints.md): `revolute`, `prismatic`, `spherical`, … or `generic` with the directions to fix |
| `Constraint TimeMotion` | [`drivers`](reference/measures.md#drivers) |
| `Measure Translational` / `Rotational` | [`measures`](reference/measures.md) |
| `Force GenericForce` / `GenericTorque` with `ForceExpression` | [`expression_force`](reference/forces.md#expression-force-and-torque), `expression_torque` |
| `DataObject Parameter` | [`parameters`](reference/model-file.md#parameters-and-formulas), usable as `=formulas` in any field |
| `DataObject Spline_1D` | [`splines`](reference/measures.md#splines) |
| `Simulation Dynamic`, `Solver HHT` | [`solver`](reference/solver.md) (fixed step) |

What YAML adds: library forces (spring-damper, torsion spring, bushing) that need no C
compiler, tires and roads, shapes for the viewer, named points and frames, references in a
`checks` section, sensors and runtime inputs.

Run the YAML model and the `.fds` and compare the channels of the two CSV files (in a script
or a spreadsheet), or let the assistant compare them. A `checks` section in the YAML with
values from the `.fds` run keeps the comparison with every later change.

## Differences in results

FreeDyn 2.0 solves the same physics with other methods, so its results differ from legacy
FreeDyn by the accuracy of both. On the 14 comparison models FreeDyn 2.0 is as accurate or
more accurate, and its simulation 2.5 to 19 times faster. The largest differences:

- **Fast rotors**: with Euler parameters, HHT damping slows a freely spinning body down (at
  300 rad/s and h = 1 ms by more than half in one second); the legacy `ModifyNewmark` option
  removes most of that. FreeDyn 2.0's Lie group body has no such loss.
- **`alpha = 0` with joints** is unstable in both programs; a little numerical damping
  (`alpha = -0.01`) removes it.
- **Line of sight forces** between a body and the ground point the other way in legacy
  FreeDyn; the importer corrects the sign, so the forces are the same.
- **Angle measures** count revolutions in both; they agree as long as an angle changes by
  less than π per step.

All of them, with numbers: [Numerical findings](findings.md). How the two programs were
compared (two levels: equal to round-off with `--profile legacy`, against independent
references with the default): the sections *Comparison with legacy FreeDyn* and *Legacy
FreeDyn against FreeDyn 2.0* of the
[README](https://github.com/freedyn-org/freedyn2#comparison-with-legacy-freedyn).

### `freedyn2 compare`

```sh
freedyn2 compare model.fds --legacy <path of the legacy FreeDyn solver> [--markdown report.md]
```

runs both programs on the same `.fds` and reports accuracy (against a converged solution both
compute themselves) and times. It is a tool for verification: it needs a checkout of the
freedyn2 repository (its `legacy/` folder) and the command-line solver of legacy FreeDyn (on
Linux built with `legacy/build_legacy.py`, see `legacy/README.md`). For running your models
you do not need it.
