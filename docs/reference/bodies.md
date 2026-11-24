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
      mtx: data/flex_beam.mtx    # relative to the model file, values in the units of the model
      damping: 0.02              # modal damping ratio, or one per mode: [0.02, 0.05]
    position: [0, 0, 0]
joints:
  clamp: {type: fixed, body1: beam, node: 10001}   # at node 10001 of the modal data
```

- Instead of `mtx`, `fem` reduces a **Nastran FE model** of the part (GRID, CTETRA with 4 or 10
  nodes, CHEXA with 8, CBAR with a PBARL ROD section, PSOLID, MAT1, RBE2, RBE3) in the units of
  the model: Craig-Bampton with the `interfaces` (RBE2 master or RBE3 reference nodes; default
  all) and `modes` fixed-interface modes (default 20), orthonormalized as Adams/Flex does. The
  result is cached; the first build of a large mesh takes a while (struct1, 30000 dofs: about
  20 s).

  ```yaml
  bodies:
    hook:
      flex: {fem: hook.bdf, interfaces: [1000001], modes: 20, damping: 0.05}
  ```

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
