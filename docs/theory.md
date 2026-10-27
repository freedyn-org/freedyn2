# Theory and algorithms

This page states the mathematics of FreeDyn 2.0 as it is implemented: the equations of motion,
the constraints and forces, how the derivatives are obtained, the time integration and the
linear algebra. It is written for a review of the algorithms; every section names the source
file it describes, and the formulas use the symbols of the code. The software architecture
(modules, file formats, viewer, assistant tools) is described in [Architecture](architecture.md)
and is not repeated here.

## 1. Notation

| symbol | meaning |
|---|---|
| \(q\) | configuration (stored coordinates), \(n_q\) numbers |
| \(v\) | velocities, \(n_v\) numbers; \(v = \dot q\) for vector-space elements, a Lie algebra element for Lie group bodies |
| \(\dot v\) | accelerations (in the code `qdd`) |
| \(\lambda\) | Lagrange multipliers, \(n_c\) numbers |
| \(x = [\dot v; \lambda]\) | unknowns of a time step, \(n_x = n_v + n_c\) |
| \(M\), \(h\), \(Q\) | mass matrix, quadratic velocity (gyroscopic) vector, generalized applied forces |
| \(\Phi(q, t)\) | holonomic constraints; \(B = \partial \Phi / \partial \delta\) their Jacobian (see 4.2) |
| \(p\) | element parameters (numbers given by the model) |
| \(\alpha, \beta, \gamma, h\) | HHT parameter, Newmark parameters, step size |
| \([a\times]\) | skew-symmetric matrix of \(a\), \([a\times] b = a \times b\) |
| \(A\) | rotation matrix of a body (body to global) |
| \(\bar\omega\), \(\omega = A \bar\omega\) | angular velocity in body and in global components |

Vectors without a bar are global components, \(\bar{(\cdot)}\) body components.

## 2. Element formulation

A model is a list of **elements** (rigid bodies, joint primitives, forces, drivers). Each
element type is a set of functions of its local quantities with one fixed input signature
(`fdgen/lagrange.py`, `STD_INPUTS`):

$$
(q,\; v,\; \dot v,\; \lambda,\; p,\; c,\; \Delta,\; t), \qquad c = (\alpha, \beta, \gamma, h),
$$

where \(\Delta\) is the configuration increment of the step (section 7). A body owns
coordinates and velocity dofs; a joint or force owns none and acts on those of the bodies it
connects; a joint owns its multipliers. The functions per element type (roles) are:

| role | output |
|---|---|
| `res` | local residual \(r = [r_\text{dyn}; r_\text{con}]\) and \(f\) (section 7) |
| `iter` | local Newton iteration matrix \(\partial r / \partial x\) |
| `iter_noqv` | the same without the derivatives of \(h\) (legacy option) |
| `ic` | the initial-condition system \(A_0, b_0, \Phi, \dot\Phi\) (section 6) |
| `init` | velocities \(v\) from the user's initial velocity input |
| `update` | \(q_{n+1} = q_n \circ \exp(\Delta)\) |
| `scale` | magnitude per velocity dof (error weights) |
| `out` | result channels from \((q, v, \dot v, p, t)\) |
| `post` | new element parameters after an accepted step (element state, e.g. continuous angles) |
| `mn_qd`, `jac_qd` | legacy modified Newmark formulas (comparison with the legacy kernel only) |

The system residual and iteration matrix are the sums of the element contributions, scattered
with a local-to-global index map (section 10):

$$
R(x) = \sum_e P_e^T\, r_e(P_e x), \qquad J = \sum_e P_e^T J_e P_e .
$$

**No derivative is written by hand.** Each element type is given by a few expressions (kinetic
energy or \(M\) and \(h\), the applied forces, the constraints); all functions above, including
all Jacobians, are derived symbolically and generated as C code (section 12).

## 3. Equations of motion

All elements together give the index-3 differential-algebraic system

$$
M(q)\,\dot v + h(q, v, t) + B(q, t)^T \lambda - Q(q, v, t) = 0, \qquad \Phi(q, t) = 0,
$$

with the kinematic relation between \(\dot q\) and \(v\) given by the configuration space of each
element.

### 3.1 Vector-space elements (Lagrange)

For an element with coordinates \(q \in \mathbb{R}^n\), \(v = \dot q\), kinetic energy
\(T(q, \dot q)\), potential \(V(q)\), \(L = T - V\) (`lagrange_terms`):

$$
M = \frac{\partial^2 T}{\partial \dot q^2}, \qquad
h = \frac{\partial^2 L}{\partial \dot q\, \partial q}\,\dot q - \frac{\partial L}{\partial q},
\qquad B = \Phi_q = \frac{\partial \Phi}{\partial q}.
$$

An element type may pass its own \(h\) (used by the legacy body, 3.3).

### 3.2 Rigid body on R³ × SO(3) (default)

`fdgen/rigid_body.py`, `build`. Configuration \(q = [R; \theta]\) with the center of mass \(R\)
and a unit quaternion \(\theta = [\theta_0; \theta_1; \theta_2; \theta_3]\) (scalar first) that
only stores the rotation; velocities \(v = [\dot R; \bar\omega]\) (global translational velocity,
body-frame angular velocity). The group is the direct product of the translations and the
rotations (not \(SE(3)\)):

$$
\dot R = v_{1:3}, \qquad \dot A = A\,[\bar\omega\times].
$$

Equations of motion (Newton and Euler):

$$
M = \begin{bmatrix} m I & 0 \\ 0 & \bar J \end{bmatrix}, \qquad
h = \begin{bmatrix} 0 \\ \bar\omega \times \bar J \bar\omega \end{bmatrix}, \qquad
Q_\text{gravity} = \begin{bmatrix} m g \\ 0 \end{bmatrix},
$$

with the principal moments \(\bar J = \operatorname{diag}(J_1, J_2, J_3)\) in the body frame. The
rotation matrix is that of Shabana (2005), Eq. 2.13:

$$
A(\theta) = \begin{bmatrix}
1 - 2(\theta_2^2 + \theta_3^2) & 2(\theta_1\theta_2 - \theta_0\theta_3) & 2(\theta_1\theta_3 + \theta_0\theta_2) \\
2(\theta_1\theta_2 + \theta_0\theta_3) & 1 - 2(\theta_1^2 + \theta_3^2) & 2(\theta_2\theta_3 - \theta_0\theta_1) \\
2(\theta_1\theta_3 - \theta_0\theta_2) & 2(\theta_2\theta_3 + \theta_0\theta_1) & 1 - 2(\theta_1^2 + \theta_2^2)
\end{bmatrix},
$$

with the Hamilton product, so that \(A(p \circ q) = A(p) A(q)\). There is **no normalization
constraint**: \(\theta\) stays on the unit sphere because it is only changed by the exponential
map (7.3).

Initial velocities: from the user input \(u = [v_0; \omega_0]\) (both global),
\(v = [v_0;\; A^T \omega_0]\).

### 3.3 Rigid body with Euler parameters (legacy profile)

`build_euler`, used for `.fds` models in the legacy profile and for the exact comparison with
the legacy FreeDyn kernel (`MBS_RigidBody_EulerP_PAI`). Absolute coordinates
\(q = [R; \theta]\), \(v = \dot q\) (7 numbers), \(\bar\omega = 2\bar G \dot\theta\),

$$
\bar G = \begin{bmatrix}
-\theta_1 & \theta_0 & \theta_3 & -\theta_2 \\
-\theta_2 & -\theta_3 & \theta_0 & \theta_1 \\
-\theta_3 & \theta_2 & -\theta_1 & \theta_0
\end{bmatrix},
\qquad
T = \tfrac12 m \dot R \cdot \dot R + \tfrac12 \bar\omega^T \bar J \bar\omega,
\qquad
\Phi_\text{norm} = \theta \cdot \theta - 1 .
$$

\(M\) follows from 3.1; the quadratic velocity vector is the legacy form

$$
h_\theta = 2 \bar G^T (\bar\omega \times \bar J \bar\omega),
$$

which agrees with the Lagrange form \(4 \dot{\bar G}^T \bar J \bar\omega\) on
\(|\theta| = 1,\ \theta \cdot \dot\theta = 0\) up to a component along \(\theta\) (absorbed by the
multiplier of \(\Phi_\text{norm}\)), but not off that manifold, where an index-3 integrator lives
numerically. Initial velocities \(\dot\theta = \tfrac12 G^T \omega_0\) with the global
\(G\) (\(\omega = 2 G \dot\theta\)).

## 4. Kinematics of connected bodies and derivatives

### 4.1 Body kinematics

`fdgen/kinematics.py`. Joints, forces, measures and sensors are written once against these
quantities of each connected body \(k\), for both body types:

$$
P(u) = R + A u, \qquad
v_P(u) = \dot R + A(\bar\omega \times u), \qquad
\omega = A \bar\omega, \qquad
\text{axis}(b) = A b,
$$

with body-fixed \(u, b\). Accelerations (for outputs only, never in force laws):

$$
a_P(u) = \ddot R + A\big(\dot{\bar\omega} \times u + \bar\omega \times (\bar\omega \times u)\big),
\qquad \dot\omega = A \dot{\bar\omega}.
$$

For Euler-parameter bodies the same quantities follow from \(A(\theta)\) and the time
derivatives along \((\dot q, \ddot q)\).

### 4.2 Derivatives on the Lie group

The derivatives with respect to the configuration are taken along the perturbation
\(q \circ \exp(\delta)\), \(\delta = [\delta_R; \delta_\phi] \in \mathbb{R}^6\) per body:

$$
\frac{\partial F}{\partial \delta} = \left.\frac{\partial}{\partial \delta} F\big(q \circ \exp(\delta)\big)\right|_{\delta = 0},
\qquad
R \mapsto R + \delta_R, \quad \theta \mapsto \theta \circ \big[1;\ \tfrac12 \delta_\phi\big] + O(\delta^2).
$$

So \(B = \partial\Phi/\partial\delta\), and the time derivative of a configuration-dependent
quantity is \(\dot F = (\partial F/\partial\delta)\, v\).

**Chain rule through intermediates** (`Chain`). The rotation matrix of a connected body is a
symbol \(A_k\) with the known derivative

$$
\frac{\partial A}{\partial \delta_{\phi,i}} = A\,[e_i \times], \qquad i = 1, 2, 3,
$$

so no derivative is ever taken through the quaternion-to-matrix map. Measures and the values of
force laws are intermediates too: for an expression \(e(w, S_1, \dots, S_m)\) with
\(S_k = s_k(w, S_1, \dots, S_{k-1})\),

$$
\frac{de}{dw} = \frac{\partial e}{\partial w} + \sum_k \frac{\partial e}{\partial S_k} \frac{dS_k}{dw},
\qquad w \in \{\delta, v, t\},
$$

with each \(dS_k/dw\) computed once (memoized) and the symbols replaced by their expressions at
the end. This keeps the generated code small; it does not change the values.

**Tangent operator.** For the configuration update \(q_n \circ \exp(\Delta)\) a change of
\(\Delta\) moves the configuration by \(\delta = T(\Delta)\, d\Delta\) to first order, with
\(T = \operatorname{diag}(I_3, T_{SO(3)}(\Delta_\phi))\) and

$$
T_{SO(3)}(\psi) = I - \frac{1 - \cos\varphi}{\varphi^2} [\psi\times]
+ \frac{\varphi - \sin\varphi}{\varphi^3} [\psi\times]^2, \qquad \varphi = |\psi|,
$$

evaluated by its Taylor series below \(\varphi = 10^{-4}\) (exact to round-off).

## 5. Constraints and applied forces

### 5.1 Joint primitives

`fdgen/joints.py`. A joint is a set of scalar primitives in a joint frame
\(K = [k_0, k_1, k_2]\) at a point \(r\) (both global at \(t = 0\)), as the legacy "Constraint
Generic":

| type | constraint |
|---|---|
| `transfix_ground` | \(\Phi = n \cdot (R + A u) - c\) |
| `rotfix_ground` | \(\Phi = (A b) \cdot g - c\) |
| `transfix_bb` | \(\Phi = (R_2 + A_2 u_2 - R_1) \cdot (A_1 n_1) - c\) |
| `rotfix_bb` | \(\Phi = (A_1 b_1) \cdot (A_2 b_2) - c\) |

with global \(n, g\), body-fixed \(b, b_1, b_2, n_1\), and the joint point \(u, u_1, u_2\) in body
coordinates. "Translation along \(k_i\) fixed" uses \(n = k_i\); "rotation about \(k_i\) fixed"
keeps the axes \(k_j\) (body 1) and \(k_l\) (body 2 or ground) perpendicular, with
\((j, l) = (1, 2), (0, 2), (0, 1)\) for \(i = 0, 1, 2\). The constant \(c\) is the initial value
of the expression, so \(\Phi(q_0) = 0\). The presets (`revolute`, `spherical`, ...) are lists of
these primitives.

### 5.2 Drivers (rheonomic constraints)

`fdgen/drivers.py`. Prescribed motion of a position measure \(m\) (displacement, distance,
angle):

$$
\Phi = m(q) - f(t),
$$

with \(f\) an expression of time, parameters and splines. With an activation time \(T_a > 0\),
\(f\) is replaced for \(t < T_a\) by the legacy polynomial (`MBS_RiBo_Time_Motion`) from the initial
value \(A = m(q_0)\) at rest, joining \(f\) with continuous value, rate and acceleration at
\(T_a\):

$$
f_a(t) = A + B\tau^5 + C\tau^6 + D\tau^7, \quad \tau = t / T_a,
$$

$$
\begin{aligned}
B &= -21A + 21 f_1 - 6 f_1' T_a + \tfrac12 f_1'' T_a^2, \\
C &= 35A - 35 f_1 + T_a(11 f_1' - f_1'' T_a), \\
D &= -15A + 15 f_1 - 5 f_1' T_a + \tfrac12 f_1'' T_a^2,
\end{aligned}
\qquad f_1^{(k)} = f^{(k)}(T_a).
$$

### 5.3 Applied forces by virtual power

`Connector.generalized_force`. Every force law returns loads; they become generalized forces
by virtual power \(Q \cdot v\):

$$
Q = \sum_{\text{loads}} \left(\frac{\partial v_P(u)}{\partial v}\right)^T F
+ \left(\frac{\partial \omega}{\partial v}\right)^T M
+ s \left(\frac{\partial \dot m}{\partial v}\right)^T,
$$

for a force \(F\) (global) at the body point \(u\), a torque \(M\) (global), and a scalar \(s\)
conjugate to a measure \(m\) whose rate \(\dot m\) is linear in \(v\). The third form makes a
spring written in a measure conservative in any configuration.

Library force laws (`fdgen/forces.py`, details in [Forces](reference/forces.md)):

- **Spring-damper** between \(P_1\) and \(P_2\): \(l = |P_2 - P_1|\), \(e = (P_2 - P_1)/l\),
  \(s = k(l - l_0) + d\,\dot l + f_0\); \(s e\) on body 1 at \(P_1\), \(-s e\) on body 2 at \(P_2\).
- **Force, torque**: constant global or body-fixed components.
- **Torsion spring** about the axis \(z_2 = x_2 \times y_2\) of body 2: angle
  \(\varphi = \operatorname{atan2}(x_1 \cdot y_2,\ x_1 \cdot x_2)\), its exact rate,
  \(\tau = -k(\varphi - \varphi_0) - d\dot\varphi + \tau_0\) conjugate to \(\varphi\).
- **Bushing** between frames \(K_1\) (body 1) and \(K_2\) (body 2), coincident at \(t = 0\):
  \(d = K_2^T(P_1 - P_2)\), \(\psi = \operatorname{axial}(\operatorname{skew}(K_2^T K_1))\);
  \(s = -(k_t d + d_t \dot d)\), \(m = -(k_r \psi + d_r \dot\psi)\), diagonal in \(K_2\), conjugate
  to \(d\) and \(\psi\).
- **Expression forces**: a force or torque given by a formula in measures, parameters, splines
  and time (`fdgen/expression_forces.py`), compiled into the model plugin (12.3).
- **Tire** (`fdgen/tire.py`): point contact below the wheel center on the road tangent plane,
  normal force \(F_z = \operatorname{softplus}(k_z \delta + c_z \dot\delta)\), slips from the contact
  point velocity, a simplified Magic Formula with load dependence and cosine weighting for
  combined slip, rolling resistance and aligning moment; \(F\) acts at the contact point (as
  \(F\) at the wheel center plus the moment, same virtual power). No tire states.

Force laws must be smooth, since Newton uses their exact derivatives: kinks are replaced by
smooth blends (\(|x| \to \sqrt{x^2 + \varepsilon}\), softplus, tanh).

### 5.4 Measures

`fdgen/measures.py`. Scalars of the state used in force laws, drivers and outputs:
displacement \(\big((P_1 - P_2) \cdot e\big)\) in a frame, velocity of the points, distance
\(|P_1 - P_2|\) and its rate, angle \(\operatorname{atan2}(a \cdot c,\ a \cdot b)\) with its exact
rate \((x\dot y - y\dot x)/(x^2 + y^2)\), angular velocity. A continuous angle counts
revolutions: its last accepted value is element state, updated after every accepted step by
the `post` function (it changes element parameters, never the state vector).

## 6. Initial conditions

`HhtIntegrator::initialize` and the `ic` functions.

```text
q, v := initial state from the model; v := init(q, u)
check  ||Phi(q, 0)||_inf <= 1e-10          (joints are built so that Phi(q0) = 0)

velocity projection, at most 10 iterations, until ||B v + Phi_t||_2 <= tol:
    [ M  B^T ] [ dv ]   [ 0              ]
    [ B   0  ] [ mu ] = [ -(B v + Phi_t) ]
    v := v + dv

consistent accelerations and multipliers:
    [ M  B^T ] [ vdot   ]   [ Q - h                                ]
    [ B   0  ] [ lambda ] = [ -(d/ddelta(B v)) v - 2 (B v)_t - Phi_tt ]
```

The right-hand side of the constraint rows is the acceleration-level constraint
\(\ddot\Phi = 0\) with all terms that do not contain \(\dot v\) (for vector-space elements
\(-(\Phi_q \dot q)_q \dot q - 2(\Phi_q \dot q)_t - \Phi_{tt}\)). Positions are not projected:
an inconsistent initial configuration is an error.

## 7. Time integration: HHT

`runtime/src/hht.cpp`. Hilber-Hughes-Taylor with \(\alpha \in [-\tfrac13, 0]\),

$$
\beta = \tfrac14 (1 - \alpha)^2, \qquad \gamma = \tfrac12 - \alpha .
$$

Two forms are implemented. The generated functions do not know which one runs: the runtime
passes **effective coefficients** \(c = (\alpha', \beta', \gamma', h)\).

### 7.1 Residual and iteration matrix

For every element (`lagrange_element`, `group_element`):

$$
r = \begin{bmatrix} \dfrac{M \dot v}{1 + \alpha'} + f \\[2mm] \dfrac{\Phi}{\beta' h^2} \end{bmatrix},
\qquad f = h + B^T \lambda - Q .
$$

The constraint rows are scaled by \(1/(\beta' h^2)\), so the iteration matrix stays well conditioned
as \(h \to 0\) (Negrut et al. 2007). With the dependence of the state on the unknowns
(7.2, 7.3), the Newton matrix with respect to \(x = [\dot v; \lambda]\) is, for vector-space elements,

$$
J = \left[\frac{\partial r}{\partial \dot v} + \gamma' h \frac{\partial r}{\partial v} + \beta' h^2 \frac{\partial r}{\partial q},\quad \frac{\partial r}{\partial \lambda}\right],
$$

and on the Lie group

$$
J = \left[\frac{\partial r}{\partial \dot v} + \gamma' h \frac{\partial r}{\partial v} + \beta' h^2 \frac{\partial r}{\partial \delta}\, T(\Delta),\quad \frac{\partial r}{\partial \lambda}\right].
$$

Both are exact derivatives of the discrete residual, including the derivatives of \(h\),
\(Q\) and \(B^T\lambda\) (the legacy option `iter_noqv` leaves out those of \(h\)).

### 7.2 Negrut form (legacy profile, vector-space elements)

Negrut, Rampalli, Ottarsson, Sharf (2007): effective coefficients \((\alpha, \beta, \gamma, h)\),
and the runtime subtracts the previous step's \(f\):

$$
R_\text{dyn} = \frac{M \ddot q_{n+1}}{1 + \alpha} + f_{n+1} - \frac{\alpha}{1 + \alpha} f_n = 0,
\qquad \frac{\Phi(q_{n+1}, t_{n+1})}{\beta h^2} = 0,
$$

$$
q_{n+1} = \underbrace{q_n + h \dot q_n + \tfrac{h^2}{2}(1 - 2\beta) \ddot q_n}_{q_\text{pred}} + \beta h^2 \ddot q_{n+1},
\qquad
\dot q_{n+1} = \underbrace{\dot q_n + (1 - \gamma) h \ddot q_n}_{\dot q_\text{pred}} + \gamma h \ddot q_{n+1}.
$$

This is the formulation of the legacy FreeDyn solver; with the legacy profile both programs
take the same steps with the same Newton iterations and agree to round-off. The legacy
"modified Newmark" option (Newmark on the global angular velocity of Euler-parameter bodies,
with the chain-rule correction of the Jacobian) is available for that comparison only.

### 7.3 Arnold-Bruls form on the Lie group (default)

Arnold and Brüls (2007), Brüls, Cardona, Arnold (2012): the equations hold at \(t_{n+1}\), the
Newmark formulas use a pseudo-acceleration \(a\):

$$
M \dot v_{n+1} + f(q_{n+1}, v_{n+1}, \lambda_{n+1}, t_{n+1}) = 0, \qquad \Phi(q_{n+1}, t_{n+1}) = 0,
$$

$$
a_{n+1} = (1 + \alpha)\,\dot v_{n+1} - \alpha\,\dot v_n, \qquad a_0 = \dot v_0,
$$

$$
\Delta = h\big(v_n + h(\tfrac12 - \beta)\,a_n + h\beta\, a_{n+1}\big),
\qquad q_{n+1} = q_n \circ \exp(\Delta),
\qquad v_{n+1} = v_n + h(1 - \gamma)\,a_n + h\gamma\, a_{n+1}.
$$

This is HHT in the generalized-\(\alpha\) family with \(\alpha_m = 0\), \(\alpha_f = -\alpha\).
Since \(dq/d\dot v_{n+1} = (1 + \alpha)\beta h^2\) and \(dv/d\dot v_{n+1} = (1 + \alpha)\gamma h\),
the effective coefficients are \((0,\ (1 + \alpha)\beta,\ (1 + \alpha)\gamma,\ h)\), and the same
generated residual and matrix serve both forms. The update of a rigid body is

$$
R_{n+1} = R_n + \Delta_R, \qquad
\theta_{n+1} = \frac{\theta_n \circ \exp_\mathbb{H}(\Delta_\phi)}{|\theta_n \circ \exp_\mathbb{H}(\Delta_\phi)|},
\qquad
\exp_\mathbb{H}(\psi) = \Big[\cos\tfrac{\varphi}{2};\ \tfrac{\sin(\varphi/2)}{\varphi}\,\psi\Big],
$$

(the normalization removes round-off only; series below \(\varphi = 10^{-4}\)). Vector-space
elements use \(q_{n+1} = q_n + \Delta\).

### 7.4 One step

```text
try_step(h, t_new):
    t := t_new; coefficients c := effective(alpha, beta, gamma, h)
    predictor (x = [vdot; lambda] := [vdot_n; lambda_n]):
        Arnold-Bruls:  Delta_pred = h (v_n + h ((1/2 - beta) a_n - beta alpha vdot_n))
                       v_pred     = v_n + h ((1 - gamma) a_n - gamma alpha vdot_n)
        Negrut:        q_pred, qdot_pred as in 7.2
    state(x):  vdot = x_v;  v = v_pred + gamma' h vdot
               Arnold-Bruls: Delta = Delta_pred + beta' h^2 vdot,  q = q_n o exp(Delta)
               Negrut:       q = q_pred + beta' h^2 vdot
               lambda = x_lambda
    R := residual(state(x))                  (Negrut: - alpha/(1+alpha) f_n)
    repeat (k = 1, 2, ... up to the iteration limit):
        factorize J(state(x))
        rhs = -R;  constraint rows with |R_i beta' h^2| < eps_machine set to 0
        dx = J^-1 rhs;  x := x + dx;  R := residual(state(x))
        convergence test (7.5)
    if not converged: reject (restore the accepted state)
    error estimate and step size control (8)
    accept: f_n := f;  a_n := (1 + alpha) vdot - alpha vdot_n;  post functions; save state
```

### 7.5 Newton convergence

**Default** (velocity criterion): the last correction changes the velocities
\(v = v_\text{pred} + \gamma' h \dot v\) by less than the tolerance, relative to their magnitude:

$$
\gamma' h\, \|\Delta \dot v\|_\infty \le \text{tol}\,\big(1 + \|v\|_\infty\big).
$$

**Legacy** (Negrut et al. 2007, Eq. 40, as the legacy solver): with the weighted norm
\(\|\Delta\| = \big(\sum_i (\Delta\dot v_i / w_i)^2\big)^{1/2}\) of the acceleration correction and its
rate \(\xi = \|\Delta_k\| / \|\Delta_{k-1}\|\),

$$
\left(\frac{\xi}{1 - \xi}\right)^2 \|\Delta_k\|^2 \le c^2\, \frac{\psi}{h^4}, \qquad c = 10^{-3},
\qquad \psi = \frac{n_v\, \varepsilon^2}{\big(\beta - \tfrac{1}{6(1 + \alpha)}\big)^2},
$$

after at least two corrections, and a step is rejected as soon as a residual norm (dynamic
rows, or constraint rows times \(\beta h^2\)) exceeds that of the predictor and its round-off
level \(10^3\,\varepsilon_\text{machine} \max(1, |\cdot|)\).

**Fixed** (real-time, section 9): exactly \(N\) corrections.

## 8. Error estimate and step size control

Local error of the Newmark position formula (Negrut et al. 2007, Eq. 37a) per velocity dof,

$$
e_i = \Big(\beta - \frac{1}{6(1 + \alpha)}\Big) h^2 \big(\dot v_{n+1,i} - \dot v_{n,i}\big),
$$

in a weighted RMS norm with the tolerance \(\varepsilon\):

$$
\theta = \frac{1}{n_v} \sum_i \left(\frac{e_i}{\varepsilon\, w_i}\right)^2
= \frac{h^4}{\psi} \sum_i \left(\frac{\dot v_{n+1,i} - \dot v_{n,i}}{w_i}\right)^2,
\qquad \text{accept if } \theta \le 1 .
$$

Weights:

- `mixed` (default): \(w_i = 1 + \max(s_{n,i}, s_{n+1,i})\) with the element scales \(s\) (\(|R|\)
  for translations, 0 for rotation increments): absolute and relative tolerance as in Hairer,
  Nørsett, Wanner (Sec. II.4);
- `legacy`: \(w_i = \max(1, \text{largest } |q_i| \text{ so far})\) (Negrut et al. Eq. 35b, as the legacy solver).

Step size (legacy controller, `SOL_SimTimeManagerHHT`):

```text
after an accepted step:  h := 0.9 h / theta^(1/6)      (theta^(1/6) = (error/eps)^(1/3))
after a rejected step:   h := h / 2
h := min(h, h_max);  stop with an error if h < h_min
exact output times: if 1.1 h > t_out - t then h := t_out - t
```

## 9. Real-time mode

For a fixed wall-clock budget per step (driving simulators), `newton: fixed` with a fixed step
size: exactly \(N\) Newton corrections per step whatever the state, no error estimate, and the
linear solver keeps the pivot order of its first factorization (numerical refactorization
only). With \(\alpha' = 0\) (Arnold-Bruls form) the residual after the last correction is not
needed and not evaluated. Every step then executes the same instructions; the measured
timing and determinism are in [Architecture](architecture.md#real-time-mode).

## 10. Linear algebra

`runtime/src/system.cpp`, `scatter.cpp`, `linear_solver.cpp`.

- **Assembly.** Each generated function has a fixed sparsity pattern. At model load the
  runtime maps every nonzero of every element matrix to its position in the global CSR matrix
  (scatter map); per evaluation it only adds numbers. The pattern of \(J\) is fixed for the run.
- **Factorization.** SuiteSparse KLU (default): symbolic analysis (block triangular form, AMD
  ordering) once, then numerical refactorization with the kept pivot order; a full
  factorization with new pivots when the refactorization fails or the reciprocal pivot growth
  estimate drops below \(10^{-3}\) of that of the last full factorization. Alternatives: Eigen
  SparseLU and a dense LU.
- The same machinery assembles the initial-condition matrix \([M\ B^T; B\ 0]\).

## 11. Outputs, accelerations, sensors

Result channels come from the `out` function of each element at \((q, v, \dot v, p, t)\). The
accelerations are the \(\dot v\) that HHT solves for, not difference quotients. A sensor at the
body point \(u\) in a body-fixed frame \(K\) gives position, velocity, acceleration
\(a_P\) (4.1), the specific force \(K^T A^T (a_P - g)\) (what an accelerometer measures), angular
velocity and angular acceleration, in \(K\).

## 12. Linearization and eigenvalues

`fdgen/linearize.py`. About a steady motion \((q_\text{ref}, v_\text{ref}, \lambda_\text{ref})\), the
system matrices follow from the generated iteration matrices: with \(\alpha' = 0\), \(h = 1\),
\(\Delta = 0\),

$$
J(\beta', \gamma') = \begin{bmatrix} M + \gamma' C + \beta' K & B^T \\ B & 0 \end{bmatrix},
$$

so three evaluations separate \(M\), the velocity Jacobian \(C\) (damping, gyroscopic terms) and
the stiffness \(K\) (including \(\partial (B^T\lambda)/\partial\delta\)). With
\(\dot\delta = \Delta v + D\delta\), \(D = -[\bar\omega_\text{ref}\times]\) on the rotation dofs,

$$
M \Delta\dot v + C \Delta v + K \delta + B^T \Delta\lambda = 0, \qquad B\delta = 0 .
$$

The constraints are eliminated with a null-space basis \(N\) of \(B\) (\(\delta = N\xi\)):

$$
M_r \ddot\xi + C_r \dot\xi + K_r \xi = 0, \quad
M_r = N^T M N,\ C_r = N^T (C - M D) N,\ K_r = N^T (K - C D) N,
$$

and the eigenvalues of the first-order form. Axisymmetric spinning bodies (wheels) are
described in a frame that does not spin with them, so the matrices do not depend on the wheel
angle; cyclic coordinates (global translations that the equations do not depend on) are left
out, since they only add eigenvalues 0.

## 13. Use of CasADi

CasADi (Andersson et al. 2019) is used as a **symbolic differentiation and C code generation
tool at build time**. It is not used at run time: the runtime is plain C++ and links only the
generated C code.

### 13.1 From expressions to element functions

```text
for each element type:
    symbols q, v, vdot, lambda, p, coef, Delta, t        (casadi.SX)
    the physics as expressions: M, h, Q, Phi             (or T, V, Q, Phi for vector-space elements)
    derived symbolically (casadi.jacobian, casadi.jtimes, the chain rule of 4.2):
        B = dPhi/ddelta, r, f, J, iter_noqv, ic system, init, update, scale, out, post
    casadi.Function per role, with named inputs and outputs and fixed sparsity
```

The scalar symbolic type `SX` gives expression graphs whose derivatives are again expression
graphs; algorithmic differentiation (forward and reverse) produces the Jacobians, so they are
exact up to round-off. Sparsity is structural: a nonzero exists where the expression depends on
the input.

### 13.2 Code generation

`fdgen/codegen.py`. `casadi.CodeGenerator` writes every function as self-contained C
(no CasADi library needed at run time), with its sparsity patterns and work-array sizes, and a
registry (`fd_element_type`: one function table per role, an ABI version) the runtime looks up
by element type name. The library element types (bodies, joint primitives, forces, sensor,
tire) are compiled into the runtime.

### 13.3 Model plugin

`fdgen/plugin.py`. Laws that belong to a model (expression forces, drivers, the element types
of extensions) are generated the same way per model and compiled with the system C compiler
into a shared library next to the model, loaded by the runtime at model load (ABI checked).
Numbers are parameters, not constants of the code, so a model whose values change reuses its
plugin (cache by source hash).

### 13.4 Division of work

| | where | when |
|---|---|---|
| physics (expressions), all derivatives | Python, CasADi | build time, per element type |
| C code | CasADi code generator | build time |
| assembly, Newton, HHT, step control, linear solver, roads | C++ runtime | run time |

## 14. Verification

- **Symbolic properties** (`tests/python`): the Shabana matrix, Lie group derivatives and
  tangent operator against finite differences, conservation laws, every force law against
  independent NumPy code, iteration matrices against central differences of the residual.
- **Runtime against analytic solutions** (`tests/cpp`): free body (momentum, energy,
  torque-free precession), pendulum period from the elliptic integral, forces against closed
  forms; the achieved error and its tolerance are printed per check.
- **Legacy comparison** (`legacy/compare.py`): *exact* level, legacy profile (Negrut form,
  Euler parameters, legacy Newton and error norm): the same steps, the same Newton iterations,
  results equal to round-off with the legacy FreeDyn kernel; *physics* level: both programs
  against independent references.
- **Convergence**: `freedyn2 compare` runs both programs against a converged solution each
  computes itself.

## 15. Points for a review

The statements below are what an algorithmic review would confirm or correct, with the section
that states each.

1. The equations of motion of the rigid body on \(\mathbb{R}^3 \times SO(3)\) and of the legacy
   Euler-parameter body, including the legacy form of \(h_\theta\) (3.2, 3.3).
2. The derivatives along \(q \circ \exp(\delta)\), the known derivative of \(A\), the chain rule
   through intermediates, and the tangent operator \(T_{SO(3)}\) (4.2).
3. The joint primitives and their composition into joints; the driver constraint and its
   activation polynomial (5.1, 5.2).
4. Generalized forces by virtual power, and the force laws (5.3).
5. The initial-condition procedure: velocity projection and the consistent accelerations,
   including the rheonomic terms (6).
6. The residual with the constraint scaling, and the iteration matrix as the exact Jacobian of
   the discrete equations in both forms (7.1).
7. The Negrut form (7.2) and the Arnold-Bruls form with the pseudo-acceleration, the
   effective coefficients and the configuration update (7.3), with predictor and corrector (7.4).
8. The Newton criteria (7.5), the local error estimate, its weights and the step size
   controller (8).
9. The linearization from the iteration matrices and the elimination of the constraints (12).

## References

- Andersson, J. A. E., Gillis, J., Horn, G., Rawlings, J. B., Diehl, M. (2019). CasADi: a
  software framework for nonlinear optimization and optimal control. *Mathematical
  Programming Computation* 11, 1-36.
- Arnold, M., Brüls, O. (2007). Convergence of the generalized-\(\alpha\) scheme for constrained
  mechanical systems. *Multibody System Dynamics* 18, 185-202.
- Arnold, M., Brüls, O., Cardona, A. (2015). Error analysis of generalized-\(\alpha\) Lie group
  time integration methods for constrained mechanical systems. *Numerische Mathematik* 129,
  149-179.
- Brüls, O., Cardona, A., Arnold, M. (2012). Lie group generalized-\(\alpha\) time integration of
  constrained flexible multibody systems. *Mechanism and Machine Theory* 48, 121-137.
- Davis, T. A., Palamadai Natarajan, E. (2010). Algorithm 907: KLU, a direct sparse solver
  for circuit simulation problems. *ACM Transactions on Mathematical Software* 37(3), 36.
- Hairer, E., Nørsett, S. P., Wanner, G. (1993). *Solving Ordinary Differential Equations I*,
  2nd ed. Springer.
- Hilber, H. M., Hughes, T. J. R., Taylor, R. L. (1977). Improved numerical dissipation for
  time integration algorithms in structural dynamics. *Earthquake Engineering and Structural
  Dynamics* 5, 283-292.
- Negrut, D., Rampalli, R., Ottarsson, G., Sharf, A. (2007). On an implementation of the
  Hilber-Hughes-Taylor method in the context of index 3 differential-algebraic equations of
  multibody dynamics. *Journal of Computational and Nonlinear Dynamics* 2(1), 73-85.
- Pacejka, H. B. (2006). *Tire and Vehicle Dynamics*, 2nd ed. Butterworth-Heinemann.
- Shabana, A. A. (2005). *Dynamics of Multibody Systems*, 3rd ed. Cambridge University Press.
