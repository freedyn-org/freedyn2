# Numerical findings

Observations made while verifying freedyn2 against legacy FreeDyn and against independent
references. The comparison models in `examples/compare` and the tests reproduce them.

- **Fast rotors**: with Euler parameters as coordinates, the parameters oscillate with omega/2
  and HHT damps that oscillation, so a freely spinning rotor loses speed (h = 1e-3,
  alpha = -0.1, 1 s): Negrut form -18 % at 100 rad/s, -57 % at 300 rad/s; Arnold–Bruls with
  Euler parameters -0.02 % and -1 %, both -100 % at 1000 rad/s. Under a driving torque the
  rotor saturates. The legacy `ModifyNewmark` (Newmark on the angular velocity; Sherif et al.,
  Multibody System Dynamics 46, 2019) removes the loss
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
  accelerations grows until the run breaks down (pendulum: after ~8 s in freedyn2; legacy
  FreeDyn crashes on the same model after 2.75 s). A little numerical damping removes it:
  `alpha = -0.01` keeps the pendulum's energy to 9e-6 and its period to 3.5e-6 of the elliptic
  integral solution. The generator warns about `alpha = 0` in models with joints.
- **Legacy line-of-sight convention**: `In-line-of-sight` points from body 1 to body 2 between two
  bodies, but from the ground point to the body for body-ground forces. freedyn2 always uses
  "from point 1 to point 2" (positive = tension); the `.fds` importer negates body-ground
  expressions.
- **Legacy `AKISPL` is an alias of `SPL`**: it uses the interpolation type of the spline data
  object, not necessarily Akima (`MBS_FunctionExpression`); the importer follows that.
- **Legacy angle measures count revolutions** with state updated at every evaluation;
  freedyn2 keeps the last accepted value as element state (`post`), so Newton iterations and
  rejected steps cannot change it. Both agree as long as an angle changes by less than pi per step.
- **Legacy differentiates driver motions numerically** for its activation polynomial; freedyn2
  uses exact derivatives (difference ~1e-10 in the comparison models). Legacy FreeDyn also
  crashed (Pardiso, forward/backward substitution) on a TimeMotion model with a body hung on a
  body-body force only; the comparison model keeps that body on a prismatic joint.
- **Newton criterion on the velocities only** (freedyn2, fixed): the `velocity` criterion
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

## Flexible bodies against legacy FreeDyn

Checked once, when the flexible body was added (November 2026), with the legacy check-in tests
of `me_testing` (Linux build of the legacy kernel, which reproduces the Windows reference
results to 2e-8). The models are kept as golden results of freedyn2
(`tests/data/flex`, `tests/python/test_flex_golden.py`); the legacy comparison is not repeated.

- **03_FlyingFleBo** (free beam, 26 modes, full coupling, spinning at 5000 rad/s, 1e-4 s):
  at the legacy step 1e-6 the modal coordinates differ by 2.5e-3 relative (mostly the dominant
  mode at 13 kHz, omega h = 0.08), the orientation by 1e-5. With 1e-7 in both programs:
  2.5e-5 and 1e-6, the differences shrink with the step size (discretization of the two HHT
  forms, not a difference in the model).
- **04_FlexRotUHackerl** (U-shaped part on a massless body with a revolute joint, driven by a
  torque past its first natural frequency, 20 modes, full coupling, damping 5 %, 1 s): modal
  coordinates 2e-5 relative, orientation 3e-4 rad after 66 rad of rotation. freedyn2 is the
  same at steps 1e-3 and 1e-4 and closes the energy balance (work of the torque = kinetic +
  elastic energy + modal damping) to 1e-6. **Legacy FreeDyn is not**: with the step 1e-4 and
  the tolerance 1e-8 it moves away from its own result at 1e-3 (3e-2 rad after 1 s, growing in
  time), so the converged legacy result is the one at the larger step.
- **106_V8_FlexConRod** (8 flexible connecting rods on 16 massless bodies, mm-t-s) broke down at
  5 ms, with rigid rods too: the constraints were fine (full rank, one degree of freedom,
  condition number 2.5e5 from the mm lever arms), but Newton's velocity criterion (1e-10
  relative) lay below the round-off of the massless dofs, so no step size converged. With a
  Newton correction that stagnates below 1e3 times the tolerance accepted as converged (round-off
  floor, `hht.cpp`) it runs with 2.0 Newton iterations per step; its pistons agree with legacy
  FreeDyn to 0.6 % of the stroke, freedyn2 at 1e-4 against 5e-4 to 0.01 % (the legacy result at
  its step is the less accurate one). Kept as a golden (0.05 s).
- **101_V8Motion** (rigid, driven along an Akima spline of 1000 points) took 20 minutes: not
  the simulation (4 s for 10 s) but compiling the driver, 1.9 MB of C (the spline as a chain of
  if_else with its time derivatives): gcc -O2 20 min, -O1 10 min, -O0 11 s. Generated sources
  above 256 kB are now compiled with -O0 (`plugin.LARGE_SOURCE`); the run agrees with legacy
  FreeDyn to 1e-7 mm. Since then splines are data (the runtime looks up the segment, the
  code holds one cubic) and the driver is a library type: no plugin, built in 0.2 s, bitwise
  the same results.
- 05_FlexBodyJointForce and 105_Waegemaschine need contact elements freedyn2 does not have.
- The units in the title line of an MTX file are not reliable (`beam.mtx` says METER and is in
  millimeters); like the legacy kernel, freedyn2 uses the values in the units of the model.
- The legacy kernel builds with MKL up to 2024.2 only (`pip install "mkl-devel==2024.2.*"`):
  later versions dropped the sparse routines it calls (`mkl_dcsrgemv`, `mkl_dcsrcoo`, ...).

## Own FE reduction against Nastran / Adams

Checked once with the FE models of the legacy check-in tests (`me_testing`), against their MTX
files (made with MSC Nastran / Adams), November 2026:

- **struct1** (04_FlexRotUHackerl; 6420 HEX8, one RBE3 with 600 nodes, 30000 dofs, Craig-Bampton
  with 20 modes): mass 3e-8, inertia tensor 6e-5, frequencies within 0.9 % (the first five
  within 0.04 %), the norms of the invariants I5, I8, I9 within 0.3 %, the mode shapes at the
  interface node 0.3 %. The rotating hook of 04 with these modal data: largest modal amplitude
  within 0.07 %, frame position within 0.1 % of its motion and orientation within 4e-4 rad
  after 0.2 s. Reduction 9 s (Eigen's simplicial LDL^T, 3 s; the dense coupling of the large RBE3
  dominates the factor; Lanczos for the modes).
- **DaBsp** (05_FlexBodyJointForce; two plates of 931 TET10, joined by two bolts of 40 CBAR
  each (ROD, r = 1 mm, 4 mm long) with RBE2 spiders): with Euler-Bernoulli bars the frequencies
  were up to 2 % high, with shear deformation in the bars (Timoshenko, as Nastran's CBAR) the
  free-free frequencies are within 0.6 % of the first nine MTX frequencies. The MTX itself was
  made with a different reduction (more retained dofs for the contact between the plates), so
  only its frequencies are comparable.
- HEX8 needs the incompatible modes: with full integration only, bending locks.
- The shift for free-free modes must be scaled with the model (mm-t-s: M is 1e-9 of K); a
  shift of -1 left K - shift M singular in round-off.

## Reduction time

The modes of the reduction come from Lanczos with shift-invert in the M inner product and full
reorthogonalization (`runtime/app/modal.cpp`); a subspace iteration before needed about 2 m
solves per iteration for m modes and 8 iterations. Box beam meshed with TET10, 70 000 dofs, 2
interfaces, 20 fixed-interface modes: elimination 3 s, LDL^T 12 s (28 million entries in the
factor), constraint modes 0.2 s (in parallel), modes 10 s with 60 solves (before: 46 s, 320
solves), the same eigenvalues to 2e-11. The factorization is next: CHOLMOD (supernodal) would
need BLAS / LAPACK in the Windows build.

**Supernodal factorization** (`runtime/app/supernodal.hpp`, since then): a multifrontal LDL^T
with Eigen's dense kernels (the columns with the same structure as dense blocks, the update as a
matrix product; large updates and assemblies on all cores, a small thread pool) instead of
Eigen's SimplicialLDLT, which works column by column. The same factor (AMD ordering, 37 million
entries for 64 000 dofs of TET10), the same results to 1e-12, the same inertia. Factorization
of that matrix on 4 cores:

| | factorize |
|---|---|
| Eigen SimplicialLDLT (before) | 18.2 s |
| Eigen SimplicialLDLT, METIS ordering | 12.2 s |
| supernodal, generic x86-64 (SSE2) | 3.4 s |
| supernodal, AVX2 and FMA | 1.5 s |
| CHOLMOD supernodal with OpenBLAS (reference, not used: its supernodal module is GPL) | 1.1 s |

The wheels are built for any x86-64 CPU: `freedyn2_modal_avx2` is built beside `freedyn2_modal`
and taken where the CPU has AVX2 and FMA (`freedyn2_modal --cpu`). Lanczos keeps M Q along (one
product with M per step instead of four). The inertia invariants come from two matrix products
of the weighted moments of the quadrature points (the identities of the skew matrices) instead
of einsum over all points and mode pairs: 12.7 s to 0.3 s, the same to 1e-15. The whole
reduction of the box beam (64 000 dofs, 2 interfaces, 20 modes, Sturm count, invariants and
stresses): 79 s before, 14 s now (freedyn2_modal 11.6 s: elimination 2.6 s, analysis 1.2 s,
factorization 1.5 s, Lanczos 4.4 s, Sturm count 1.4 s).

A Sturm count checks the modes since then: the negative pivots of K_ii - sigma M_ii just above
the highest eigenvalue must be the number of modes (Lanczos with one start vector can miss one
of a multiple eigenvalue; on the test beams it did not, against SciPy to 1e-10 with the double
bending eigenvalues and the six rigid body modes found). It costs one more numeric
factorization (the pattern analysis is reused): box beam with TET10, 64 000 dofs, factor 35
million entries: elimination 3 s, analysis 1 s, LDL^T 22 s, modes 11 s, Sturm count 22 s,
the reduction 79 s with the invariants and stresses. The reduction is kept (cms) and made again
only when its inputs change.

The legacy flexible body tests spin fast against their first elastic frequency: 03_FlyingFleBo
at 63 %, 04_FlexRotUHackerl at 38 % (`freedyn2 check`: `spin_stiffening`). Without geometric
stiffening (neither in legacy FreeDyn nor here) their deformation is too soft by a share of the
order of 40 % and 14 %: the agreement with legacy is that of the same model, not with the real
parts.

## Forces at interface nodes: the massless interface bodies

A force at an interface node acts on a massless body `<body>_if<k>` fixed to the node (6 dofs
and 6 constraints per interface). A force element at the node itself would see the node through
all modes (its size would depend on the number of modes, against the element types of fixed
size). Measured on the flexible arm with a spring at its tip (1000 steps, 8 modes): massless
2168 Newton iterations, with 1e-4 kg 2167, with 1e-2 kg 2162, no step rejected; the massless
body costs nothing in convergence. (The round-off floor of 106_V8_FlexConRod came from the
massless dummy bodies of the legacy model, in mm-t-s.) So it stays.
