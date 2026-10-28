# How the solver works

This page explains the solver in plain words, with a pointer to the code for each idea. It is
meant to be read before [Theory and algorithms](theory.md), which states the equations, and
[Architecture](architecture.md), which describes the modules and files. Section references
like (theory 4.2) point into the theory page.

## The program in five sentences

1. A model is a list of **elements**: bodies, joints, forces, drivers.
2. Every body obeys Newton's law: mass times acceleration equals the sum of the forces.
3. A joint is a condition on the positions ("this point stays here"); to keep it, the solver
   computes the joint force together with the motion.
4. Time advances in steps. Each step predicts the new state, corrects it with Newton
   iterations and checks whether the step was accurate enough.
5. Every derivative Newton needs is derived symbolically by CasADi and generated as C code;
   none is written by hand.

## One equation

All elements together give

$$
M \dot v + h + B^T \lambda = Q, \qquad \Phi = 0 .
$$

\(M\) is mass and inertia, \(\dot v\) the accelerations, \(h\) the gyroscopic terms, \(Q\) the
applied forces (gravity, springs, tires). \(\Phi = 0\) are the joint conditions and
\(B^T\lambda\) the joint forces that keep them: \(\lambda\) is how hard each joint has to push
or pull. The accelerations and the multipliers are the unknowns of every step,
\(x = [\dot v; \lambda]\) (theory 3).

The joint conditions are stated on the positions, not on velocities or accelerations. This
makes the time integration sensitive, which is why the integrator damps high frequencies
(see [One physics, two integrators](#one-physics-two-integrators)).

## Elements and roles

Each element only knows itself. An element type is given by a few expressions (mass matrix
and gyroscopic terms, or kinetic and potential energy; applied forces; constraints), and the
generator derives a fixed set of functions from them, the **roles** (theory 2):
`res` (residual), `iter` (iteration matrix), `ic` (initial conditions), `init`, `update`,
`scale`, `out`, `post`. All of them share one input signature (`STD_INPUTS` in
`generator/fdgen/lagrange.py`).

The runtime adds the contributions of all elements into one residual and one matrix through
a local-to-global index map. It does not know what a spring or a hinge is; it only adds
numbers at precomputed positions.

There are two builders:

- `lagrange_element`: elements whose coordinates are plain vectors, \(v = \dot q\). Used by
  the legacy Euler-parameter body.
- `group_element`: elements on a Lie group, where the configuration is updated by
  \(q \circ \exp(\Delta)\). Used by the default rigid body and by everything connected to it.

## Rotations: four numbers, six directions

The position of a body is three numbers. Its orientation is stored as a unit quaternion
\(\theta\): four numbers for three rotational degrees of freedom, with no singular orientation,
but they must keep length 1 (theory 3.2, 3.3).

- **Legacy body (Euler parameters):** the four numbers are ordinary unknowns, with velocities
  \(\dot\theta\). A normalization constraint \(\theta\cdot\theta = 1\) with its own multiplier keeps
  them on the unit sphere. Per body: 7 accelerations and 1 multiplier.
- **Default body (Lie group \(\mathbb{R}^3 \times SO(3)\)):** the quaternion is only *stored*.
  It is never changed directly, only rotated further: \(\theta_{n+1} = \theta_n \circ
  \exp(\Delta_\phi)\). A rotation followed by a rotation is a rotation, so the length stays 1
  without a constraint. The velocities are \(v = [\dot R; \bar\omega]\) with the angular velocity
  in body axes, so the inertia matrix is a constant diagonal and the gyroscopic terms are
  Euler's \(\bar\omega \times \bar J \bar\omega\). Per body: 7 stored numbers, 6 accelerations.

## Derivatives on the Lie group

This is the part of the implementation that differs most from a textbook multibody code
(theory 4.2, 7.1, 7.3). At its core it is calculus of variations: \(\delta\) below is a
variation (a virtual displacement), and \(\partial F/\partial\delta\) is the first variation of
\(F\), with the variation taken on the group, \(q \circ \exp(\varepsilon\delta)\), instead of
\(q + \varepsilon\,\delta q\). The subsections show how this is implemented;
[One variation for everything](#one-variation-for-everything) shows where the same \(\delta\)
is used.

### The problem

Newton needs the derivative of the residual with respect to the configuration. Taking the
derivative with respect to \(\theta_1\) asks what happens if \(\theta_1\) alone grows a little.
Then \(|\theta| \ne 1\), and the four numbers no longer describe a rotation: the derivative
points in a direction no body can move in. The legacy body handles this with the
normalization constraint, whose multiplier absorbs that direction.

### Perturb the body, not the numbers

The derivative is taken along a small *motion* of the body,
\(\delta = [\delta_R; \delta_\phi]\), six numbers per body: \(\delta_R\) shifts the center of mass
(global components), \(\delta_\phi\) turns the body by a small angle about its own axes. Every
direction is a motion the body can make. The perturbed configuration is
\(q \circ \exp(\delta)\), and the derivative is an ordinary one at \(\delta = 0\):

$$
\frac{\partial F}{\partial \delta} = \left.\frac{\partial}{\partial \delta} F\big(q \circ \exp(\delta)\big)\right|_{\delta = 0},
\qquad R \mapsto R + \delta_R, \quad \theta \mapsto \theta \circ \big[1;\ \tfrac12 \delta_\phi\big].
$$

First order in \(\delta\) is enough, since only the derivative at \(\delta = 0\) is used.

In code: `perturb` in `fdgen/rigid_body.py` builds the perturbed configuration;
`partial_delta` in `group_element` (`fdgen/lagrange.py`) substitutes it into an expression,
lets CasADi take the Jacobian with respect to \(\delta\) and sets \(\delta = 0\). It is a recipe
applied to every expression, not a formula written per element.

### Why body axes

The velocities \(v = [\dot R; \bar\omega]\) have exactly these six directions. So for every
quantity \(F\) that depends on the configuration,

$$
\dot F = \frac{\partial F}{\partial \delta}\, v .
$$

One derivative serves twice: in the iteration matrix and as a time derivative (point
velocities, the rate of a measure, the terms of the consistent initial accelerations).
`Connector.rate` in `fdgen/kinematics.py` is exactly this product. The constraint Jacobian
\(B = \partial\Phi/\partial\delta\) has six columns per body, as many as the mass matrix, so
everything is assembled in the same space.

### The rotation matrix is a symbol

A small rotation about the body axes multiplies \(A\) by \(I + \tilde\delta_\phi\), so the
derivative of the rotation matrix is known in advance:

$$
\frac{\partial A}{\partial \delta_{\phi,i}} = A\, \tilde e_i, \qquad i = 1, 2, 3 .
$$

`Connector` (`fdgen/kinematics.py`) therefore makes the rotation matrix of each connected body
a symbol \(A_k\) with this known derivative. Joints and forces are written in \(A_k\), the
derivatives go through the chain rule (`Chain` in `fdgen/lagrange.py`), and only at the end is
\(A_k\) replaced by the quaternion formula. No derivative is ever taken through
\(A(\theta)\).

Measures and the values of force laws in expression forces are intermediates of the same kind:
each is differentiated once and memoized, instead of once per row of the residual. This does
not change any value, only the size of the generated code (gear coupling of 5 bodies in
`examples/scaling`: 40,800 to 6,000 instructions, see [Architecture](architecture.md)).

### Example: a point on the body

A body-fixed point \(u\) is at \(P = R + A u\). With the perturbation,

$$
P(\delta) = R + \delta_R + A (I + \tilde\delta_\phi) u = R + A u + \delta_R - A \tilde u\, \delta_\phi ,
$$

so \(\partial P/\partial\delta_R = I\), \(\partial P/\partial\delta_\phi = -A\tilde u\), and

$$
\dot P = \frac{\partial P}{\partial \delta}\, v = \dot R - A \tilde u\, \bar\omega = \dot R + A(\bar\omega \times u),
$$

the familiar velocity of a point on a rigid body, from the same derivative that enters the
iteration matrix.

### The tangent operator

Within a step the configuration is \(q = q_n \circ \exp(\Delta)\), and the increment depends on
the unknown accelerations through Newmark's formula, \(\Delta = \Delta_\text{pred} + \beta' h^2
\dot v\). When Newton changes \(\dot v\), \(\Delta\) changes by \(d\Delta\).

For translations, "\(\Delta + d\Delta\)" and "first \(\Delta\), then \(d\Delta\)" are the same
motion. For rotations they are not: rotations do not commute. The difference is the tangent
operator \(T\),

$$
\exp(\Delta + d\Delta) = \exp(\Delta) \circ \exp\big(T(\Delta)\, d\Delta\big) \quad \text{to first order},
$$

so the actual motion is \(\delta = T(\Delta)\, d\Delta\). \(T(0) = I\), and since a step is a
small rotation, \(T\) stays close to the identity. Its formula contains terms like
\((1 - \cos\varphi)/\varphi^2\) that are 0/0 at \(\varphi = 0\); below \(\varphi = 10^{-4}\)
`tangent_so3` and `quat_exp` (`fdgen/rigid_body.py`) use Taylor series, exact to round-off
there.

### The iteration matrix

The unknown accelerations act on the residual \(r\) in three ways: directly, through the
velocities (\(v = v_\text{pred} + \gamma' h \dot v\)) and through the configuration (via
\(\Delta\) and \(T\)):

$$
J = \left[\frac{\partial r}{\partial \dot v} + \gamma' h \frac{\partial r}{\partial v} + \beta' h^2 \frac{\partial r}{\partial \delta}\, T(\Delta),\quad \frac{\partial r}{\partial \lambda}\right].
$$

For the legacy body the configuration changes additively (\(q = q_\text{pred} + \beta' h^2
\ddot q\)), so the last term is \(\beta' h^2\, \partial r / \partial q\) without \(T\).

| | Euler parameters (legacy) | Lie group (default) |
|---|---|---|
| velocities | \(\dot R, \dot\theta\) (7) | \(\dot R, \bar\omega\) (6) |
| derivatives with respect to | the stored numbers \(q\) | the motion \(\delta\) |
| \(\lvert\theta\rvert = 1\) kept by | constraint with multiplier | update by \(\exp\) |
| configuration update | \(q + \Delta\) | \(q \circ \exp(\Delta)\) |
| unknowns per body | 7 + 1 | 6 |

### One variation for everything

The same \(\delta\) carries four things that otherwise look unrelated:

| where | variational reading |
|---|---|
| applied forces | \(Q\) from the virtual work \(Q \cdot \delta\) (written as virtual power \(Q \cdot v\) in the code, see [Forces by virtual power](#forces-by-virtual-power)) |
| joint forces | \(\lambda^T \delta\Phi = \lambda^T B \delta\) gives \(B^T\lambda\), hence \(B = \partial\Phi/\partial\delta\) |
| iteration matrix | the first variation of the discrete residual; \(T\) is the variation of the exponential map |
| time derivatives | the actual motion is the variation \(\delta = v\,dt\), hence \(\dot F = (\partial F/\partial\delta)\, v\) |

The difference to the legacy body lies in the **admissible variations**. With Euler parameters
the four numbers are varied freely; this includes variations that change the length of the
quaternion, and the normalization constraint with its multiplier filters them out again. On
the Lie group these variations do not exist.

The equations of motion of the Lie group body are not derived variationally in the code:
\(M\) and \(h\) are written directly as the Newton-Euler equations (`fdgen/rigid_body.py`).
Hamilton's principle with these variations gives the same Euler equations. For vector-space
elements the generator does derive the Lagrange equations from \(T\) and \(V\).

## Joints from primitives

A joint is a list of scalar primitives of two kinds, each to ground or between two bodies
(`fdgen/joints.py`, theory 5.1): `transfix` (a point does not move along an axis) and `rotfix`
(an axis of one body stays perpendicular to an axis of the other, which locks the rotation
about the third axis). A revolute joint is three `transfix` and two `rotfix`; a spherical joint
is the three `transfix`. Each primitive subtracts its own initial value, so it is exactly zero
in the initial configuration.

A driver is a joint with a schedule: a measure equals \(f(t)\). With an activation time, a
degree-7 polynomial blends from rest into \(f\) with continuous value, rate and acceleration
(theory 5.2).

## Forces by virtual power

A force law returns loads: a force at a body point, a torque, or a scalar conjugate to a
measure. `Connector.generalized_force` turns them into generalized forces by virtual power:
the power of the load for velocities \(v\) is linear in \(v\), and its gradient is \(Q\)
(theory 5.3). The lever arm bookkeeping is automatic.

Force laws must be **smooth**: Newton uses their exact derivative, and a kink has none. Kinks
are replaced by smooth blends: \(|x| \to \sqrt{x^2 + \varepsilon}\), softplus for contact,
tanh for a sign.

## One physics, two integrators

The time integration is HHT: Newmark's formulas for the new positions and velocities in terms
of the new accelerations, with a parameter \(\alpha \in [-\tfrac13, 0]\) that damps only the
highest frequencies. \(\beta\) and \(\gamma\) follow from \(\alpha\). With \(\alpha = 0\) the
solution of the position-level constraints slowly grows unstable (theory 7).

Two forms are implemented:

- **Negrut form** (legacy profile): forces averaged between the old and the new time, positions
  added. With the legacy profile, FreeDyn 2.0 and the legacy FreeDyn kernel take the same steps
  with the same Newton iterations and agree to round-off.
- **Arnold-Brüls form** (default): the equations hold at the new time, \(\alpha\) enters through
  a pseudo-acceleration, the configuration is updated by \(\exp\) on the Lie group.

The generated functions do not know which form runs. The runtime passes **effective
coefficients** \(c = (\alpha', \beta', \gamma', h)\): \((\alpha, \beta, \gamma, h)\) for Negrut,
\((0, (1+\alpha)\beta, (1+\alpha)\gamma, h)\) for Arnold-Brüls. One generated residual and one
iteration matrix serve both (theory 7.3).

Two details apply to both forms:

- The constraint rows are divided by \(\beta' h^2\), so the iteration matrix stays well
  conditioned for small steps (theory 7.1).
- The iteration matrix is the **exact** derivative of the discrete residual, including the
  derivatives of the gyroscopic terms and of the joint forces, so Newton converges
  quadratically. The legacy FreeDyn kernel derived its matrix by hand, and per body the flag
  `RegardVelocityInertiaForcesInsideJacobian` decided whether the derivatives of \(h\) were
  included; without the flag they were left out. The `.fds` importer carries this over as the
  body setting `jacobian: no_qvv`, which selects the role `iter_noqv`. Without those
  derivatives Newton still converges, but only linearly, so a body that spins fast needs more
  iterations. Now the full matrix is the default and is generated for every element type,
  including expression forces and extensions.

## A step

Predict with Newmark's formulas, correct with Newton until the velocity change of the last
correction is below the tolerance, estimate the local error from the change of the
accelerations, then accept (and grow the step) or halve the step and try again. Output times
are hit exactly. The pseudocode is in theory 7.4, the controller in theory 8.

## Assembly and the linear solver

Every generated function has a fixed sparsity pattern. At model load the runtime maps every
nonzero of every element matrix to its position in the global sparse matrix (scatter map,
`runtime/src/scatter.cpp`); per evaluation it only adds numbers. KLU (SuiteSparse) analyses the
pattern once and afterwards refactorizes numerically with the kept pivot order; a full
factorization follows only when the pivots degrade (theory 10).

## Real time

With `newton: fixed` and a fixed step size, every step does exactly \(N\) Newton corrections,
no error estimate and no branching, and the linear solver keeps its pivot order. Every step
executes the same instructions, so the time per step is predictable (theory 9,
[Architecture](architecture.md#real-time-mode)).

## Linearization from the iteration matrix

With \(\alpha' = 0\), \(h = 1\) and \(\Delta = 0\) the iteration matrix is
\(M + \gamma' C + \beta' K\) plus the constraint blocks. Three evaluations with different
\(\beta', \gamma'\) separate the mass, damping and stiffness matrices. The constraints are
eliminated with a null-space basis of \(B\), and the eigenvalues of the reduced system give the
modes (`fdgen/linearize.py`, theory 12). Wheels are described in a frame that does not spin
with them, so the matrices do not depend on the wheel angle.

## Build time and run time

| | where | when |
|---|---|---|
| physics as expressions, all derivatives | Python, CasADi | build time, per element type |
| C code | CasADi code generator | build time |
| assembly, Newton, HHT, step control, linear solver, roads | C++ runtime | run time |

CasADi builds expression graphs and differentiates them with algorithmic differentiation, so
the derivatives are exact up to round-off and their sparsity is structural. The generated C is
self-contained: the runtime does not link CasADi. Library element types are compiled into the
runtime. What belongs to one model (expression forces, drivers, extension elements) is
generated the same way into a model plugin, a shared library next to the model; numbers are
parameters, so changing values does not recompile it (theory 13).

## How it is checked

- every derivative (Lie group derivatives, tangent operator, iteration matrices) against finite
  differences, every force law against independent NumPy code (`tests/python`);
- the runtime against analytic solutions: free body, pendulum period, forces in closed form
  (`tests/cpp`);
- the legacy profile against the legacy FreeDyn kernel, equal to round-off
  (`legacy/compare.py`);
- both programs against a converged solution each computes itself (`freedyn2 compare`).
