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
