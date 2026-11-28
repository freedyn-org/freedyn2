# Bodies and geometry

## Bodies

A body is a rigid body with its mass, its principal moments of inertia about the center of
mass, and its position and orientation at t = 0:

```yaml
bodies:
  arm:
    mass: 2.0
    inertia: [0.002, 0.17, 0.17]        # about the principal axes
    position: [0.5, 0, 0]                # center of mass, global
    orientation: [1, 0, 0, 0]            # principal axes = global axes
    velocity: [0, 0, 0]                  # initial velocity of the center of mass
    angular_velocity: [0, 0, 5]          # initial angular velocity, global components
```

<!-- keys: body -->

- `inertia` are the **principal moments** in the body frame; `orientation` gives the principal
  axes. A body with a full inertia tensor: rotate its frame to the principal axes (the
  eigenvectors of the tensor) and give the eigenvalues.
- A real body satisfies I1 + I2 ≥ I3 for every ordering of its moments; the checks report an
  error otherwise.
- Bodies move on the Lie group R³ × SO(3): no singular angles, any rotation.

### Flexible bodies

A body with `flex` is flexible: a floating frame of reference with modal coordinates, from the
modal data of an Adams MTX file (MSC Nastran or Adams/Flex). Mass and inertia come from that
file; `position`, `orientation` and `velocity` are those of the body frame of the modal data
(its origin and axes), not of the center of mass.

```yaml
bodies:
  beam:
    flex:
      modal: data/flex_beam.mtx  # .fdflex, or an Adams MTX file (values in the units of the model)
      damping: 0.02              # modal damping ratio, or one per mode: [0.02, 0.05]
      max_frequency: 5000        # optional: keep the modes up to 5000 Hz
      exclude_modes: [3]         # optional: leave out mode 3
    position: [0, 0, 0]
joints:
  clamp: {type: fixed, body1: beam, node: 10001}   # at node 10001 of the modal data
```

- Instead of `modal`, `fem` reduces a **Nastran FE model** of the part (GRID, CTETRA with 4 or 10
  nodes, CHEXA with 8, CBAR with a PBARL ROD section, PSOLID, MAT1, RBE2, RBE3) in the units of
  the model: Craig-Bampton with the `interfaces` (RBE2 master or RBE3 reference nodes; default
  all) and `modes` fixed-interface modes (default 20), orthonormalized as Adams/Flex does. The
  result is kept in `cms/` (below); the first build of a large mesh takes a while (struct1,
  30000 dofs: about 20 s).

  ```yaml
  bodies:
    hook:
      flex: {fem: hook.bdf, interfaces: [1000001], modes: 20, damping: 0.05}
  ```

- **Rigid to flex.** With `material` and `mesh_size` the body is meshed from its one STL shape
  (or `stl`) with quadratic tetrahedra (Netgen, `pip install "freedyn2[fem]"`) and reduced
  automatically; the rest of the model stays as it is:

  ```yaml
  bodies:
    arm:
      mass: 0.314                      # kept for reference: a warning if the FE mass differs by 5 %
      inertia: [1.3e-5, 1.05e-3, 1.06e-3]
      position: [0.1, 0, 0]
      shapes: [{type: mesh, file: arm.stl, frame: body}]
      flex: {material: steel, mesh_size: 0.004, modes: 10}
  ```

  Every joint and force at the body gets an interface node at its point (a `torque` needs a
  `point`, a `torsion_spring` a `location` there), connected to the
  surface nodes around it by an RBE2 (`interface_type: rbe3` for an interpolating one, its nodes
  weighted by their share of the surface area).
  Joints act at that node (a joint with the flexible body as body1 swaps its bodies); forces act
  on a massless body `<body>_if<k>` fixed to it, so that every force law works. The spider of a
  revolute or cylindrical joint takes the **bore** around its axis (joint z): the surface at the
  smallest distance from the axis near the joint point, along the axis as far as it goes, all
  around. Otherwise, and where there is no bore, it takes the surface nodes within the distance
  to the nearest one plus `mesh_size` (a small patch around a point on a face). **The spider
  decides how stiff the connection is** (a hinge on a 6 mm patch of a 20 mm wide arm: the first
  in-plane bending mode at 260 Hz; with the whole end face: 440 Hz). The viewer shows the
  spiders (*spiders of flexible bodies*; a click on one: its joint or force, with the size of
  the spider); give `interfaces` with a `radius`, `axis` or `type` where they need to be
  different. Named materials are in SI units (m, kg, s); in other units give
  `{E, nu, rho}`. Position and orientation stay those of the rigid body; mass and inertia come
  from the FE model.

- **The CMS result is kept** next to the model, for `fem` and `material` bodies:
  `cms/<body>.fdflex`, the freedyn2 modal data file: modal data, mode shapes on the surface for
  the viewer, and what it was made from (frequencies, mesh size, time). A build
  reuses it while mesh, material, interfaces and modes are the same, and makes it again when
  they change. Other models use it with `flex: {modal: cms/arm.fdflex}`; put it under version
  control or pass it on like any data file. `cms` gives another place.
  `freedyn2 flex-info cms/arm.fdflex` shows what it holds; `--mtx arm.mtx` writes it as an
  Adams MTX file (and `--fdflex` the other way). The format: a ZIP archive of NumPy arrays,
  documented in `fdgen/flex/fdflex.py`.

- **Your own FE model and spiders.** `freedyn2 flex-export model.yaml arm` writes the meshed
  body as Nastran bulk data (`arm.bdf`, in its body frame; its interfaces with their points and
  the joints and forces at them as comments). Change it in a pre-processor or by hand (spiders,
  mesh refinement, materials) and use it with `flex: {fem: arm.bdf}`. Any Nastran model works
  the same way: joints and forces without `node` attach to the interface node (an RBE master or
  reference node) at their point, within 1 % of the body size.

- **Stresses for fatigue.** `freedyn2 modal-stress model.yaml arm [result.csv]` writes, for
  a body with an FE model (`fem` or `material`), `modal_stress/arm_modes.unv` (the solids of the FE
  model and the stress tensor of each elastic mode at their nodes, Universal file datasets
  2411, 2412, 2414, in the body frame and the units of the model) and `modal_stress/arm_eta.csv`
  (the modal coordinates of the run over time). The stress at any time is the superposition
  sigma(t) = sum_i sigma_i eta_i(t): in a fatigue tool the `.unv` gives one load
  case per mode and the CSV their load-time histories. Without a result file it simulates the
  model first. The modal stresses come from the reduction (exact at the nodes for a linear
  strain field in TET10, HEX8 with its incompatible modes; averaged over the elements at a
  node) and are kept in the `.fdflex`; MTX modal data have none. Check that your fatigue tool
  reads Universal files; checked here against a cantilever under a tip load (beam theory
  within 0.4 %).

- Joints act at a **node** of the modal data (`node` of the joint): its point and axes move and
  turn with the deformation. The flexible body is `body2`, or `body1` of a joint with the
  ground. Without `node` a joint acts on the body frame. `location` defaults to the node.
- `rotation_coupling: false` and `quadratic_inertia: false` leave out terms of the inertia
  coupling, as the legacy options `Is_Itf_active` and `Is_QuadraticTermOF_Itt_active`.
- Result channels: the body frame as for a rigid body (`beam.R_x`, `beam.theta0`, …) and per
  mode `beam.eta3.eta`, `beam.eta3.etadot`, `beam.eta3.etaddot` (modes numbered from 1 in the
  order of the file, rigid body modes left out).
- Any number of modes, no compiler needed. The cost grows with the square of the number of
  modes (full coupling); high-frequency modes need small time steps.

**Limits of the model** (the floating frame of reference with linear modes, as Adams/Flex and
legacy FreeDyn):

- **Small, linear deformation** about the body frame: the deformation is a sum of mode shapes,
  node rotations are linearized.
- **No geometric stiffening.** The centrifugal stiffening of a spinning part (and the
  stiffening by a preload) is not modelled: a body that spins at Omega lacks a stiffness share of
  the order of (Omega / omega_1)^2, omega_1 its first elastic frequency, and very fast spin can
  even make it unstable. `freedyn2 check` warns from 10 % of omega_1 (`spin_stiffening`). For
  rotors, turbine blades or crankshafts at high speed the results are then too soft.
- **The body frame is a floating reference frame**, not a material point (the mode shapes are
  orthogonal to the rigid body motions, nearly the mean axes). Joints, forces, measures and
  sensors that are not at an interface node act on that frame and do not see the deformation:
  joints and forces warn, measures and sensors too. Use the interface nodes (a force at a node
  acts through a massless body `<body>_if<k>` fixed to it; a measure on that body moves with the
  node).
- **Modal damping** `damping` acts on the free-free modes of the body, not on the modes of the
  system it is built into: a clamped body with `damping: 1` is not critically damped. Without
  `damping` the modes are damped as in Adams/Flex: 1 % below 100 Hz, 10 % from 100 to 1000 Hz,
  100 % from 1000 Hz (the stiff constraint modes of the interfaces do not ring); `damping: 0`
  for none. Critically damped modes are no oscillation in the eigenvalues (`modes`).
- **Mode selection**: `max_frequency` (Hz) keeps the modes up to it, `exclude_modes` leaves out
  modes by number; the kept modes are numbered from 1 again (`flex_info` shows which). Leaving
  out modes makes the body stiffer and its interfaces less accurate: check with
  `flex_convergence`. The cost of a body grows with the square of its number of modes.
- **Interfaces far from the surface** (more than 3 mesh sizes and a tenth of the part) warn: the
  spider is then a long rigid lever.
- **The modes** are those of the Craig-Bampton basis: `modes` fixed-interface modes plus 6 per
  interface node. The reduction checks by a Sturm count that they are the lowest ones; kept
  modes that end inside a multiple eigenvalue (a symmetric part) give a warning, as do modes
  that converged to a residual above the tolerance only.
- **FE models** (`fem`): solids (CTETRA, CHEXA), rods (CBAR with PBARL ROD), RBE2, RBE3 and
  INCLUDE files. Other elements, point masses (CONM2), other rigid elements and MPC are an
  error, not left out; loads, SPC and output requests are ignored.

## Shapes

Geometry is for the [viewer](../viewer.md) only; it does not change the simulation. Without
shapes a body is drawn as the box of its mass and inertia, a wheel with a tire as a cylinder.

```yaml
bodies:
  arm:
    ...
    shapes:
      - {type: box, size: [1.0, 0.05, 0.05]}                          # at the body frame
      - {type: cylinder, radius: 0.04, length: 0.1, position: [1, 0, 0],
         orientation: [0.7071068, -0.7071068, 0, 0], color: "#e15759"} # axis along global y
      - {type: mesh, file: arm.stl, scale: 0.001, position: [0, 0, 0]} # CAD export of the assembly, mm
ground:
  shapes:
    - {type: box, size: [2, 2, 0.02], position: [0, 0, -0.5], opacity: 0.5}
```

<!-- keys: shape -->

Like all model input, shapes are placed **globally at t = 0** (with `frame: body` in the body
frame instead). A mesh exported from a CAD assembly has its vertices in assembly coordinates:
give it `position: [0, 0, 0]` and it sits where it was in the assembly. Meshes: binary or ASCII
STL, OBJ.
