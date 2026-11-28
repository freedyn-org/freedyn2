# Co-simulation (FMU)

A freedyn2 model can be exported as a **Functional Mock-up Unit** for co-simulation, FMI 2.0 or
3.0, and run in any tool that imports FMUs: Simulink, Dymola, OpenModelica, Simcenter Amesim,
Python with FMPy, an HIL or system simulation framework.

```sh
freedyn2 fmu model.yaml                 # FMI 2.0: model.fmu next to the model file
freedyn2 fmu model.yaml --fmi 3 -o car.fmu
```

## What the FMU has

- **Inputs**: the [runtime inputs](reference/measures.md) of the model (`inputs:` with their
  defaults as start values), real numbers. The master sets them before each communication step;
  they stay constant during it.
- **Outputs**: every result channel of a run, named as the columns of its CSV result
  (`mass.R_z`, `floor.normal_force`, `z_mm.value`, ...; see [result channels](reference/results.md)).
- **Time**: the model's time starts at the start time of the co-simulation. A communication step
  is integrated with the model's own step (`solver: step`), its last step ending at the
  communication point, so any communication step size works (FMI 3.0 states the step as
  `fixedInternalStepSize`). Contacts are located within a step as in a run.
- **Default experiment**: `end_time` and `output_step` of the model.

With constant inputs and a communication step that is a multiple of the model's step, the
outputs are those of `freedyn2 run` at the same times (the tests compare them to round-off). With
contacts the steps after an impact differ (the FMU goes on with full steps from the impact, a run
with the rest of its step); both lie within the discretization error.

## What it does not have

- **Co-simulation only**: no model exchange (the solver is part of the model: HHT on the
  index-3 equations with Lie group bodies, which a general ODE solver cannot replace).
- No saving and restoring of the state (`canGetAndSetFMUstate = false`), no derivatives, no
  event mode, no rollback: each step continues from the last one.
- **One platform per FMU**: the binary and the model plugin are compiled for the platform the
  FMU is made on (`linux64` / `x86_64-linux`, `win64` / `x86_64-windows`). Make the FMU on the
  platform it will run on.
- The linear solver of the FMU is Eigen's sparse LU, not KLU (the FMU carries no other shared
  libraries): results are the same, large models are somewhat slower than in `freedyn2 run`.
- Parameters of the model are fixed in the FMU; to vary a value in the co-simulation, make it an
  input.

## Inside

The FMU holds `binaries/<platform>/<model>.so|dll` (one generic library of the runtime with the
FMI 2.0 and 3.0 functions, the same for every model), `resources/model.fdm` (the runtime model
file) with the model plugin and data files (OpenCRG roads), `modelDescription.xml`, and freedyn2's
license in `documentation/`.
