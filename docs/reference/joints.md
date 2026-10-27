# Joints

A joint connects `body1` to `body2` (default: the ground) at a joint frame – a point
(`location`) and axes (`orientation`), given globally at t = 0 and fixed on both bodies. It
fixes some of the six relative directions of that frame: translations `dx dy dz` along and
rotations `ax ay az` about its axes.

```yaml
joints:
  hinge:
    type: revolute
    body1: rod
    body2: ground
    location: [0, 0, 0]
    orientation: [0.7071068, -0.7071068, 0, 0]   # joint z axis = global y: a hinge about y
```

| type | fixes | leaves free |
|---|---|---|
| `fixed` | dx dy dz ax ay az | nothing |
| `spherical` | dx dy dz | all rotations |
| `revolute` | dx dy dz ax ay | rotation about z |
| `cylindrical` | dx dy ax ay | translation along and rotation about z |
| `prismatic` | dx dy ax ay az | translation along z |
| `generic` | the directions in `fix` | the others |

```yaml
  planar_slide: {type: generic, body1: block, location: [0, 0, 0], fix: [dz, ax, ay]}
```

<!-- keys: joint -->

**The joint's free axis is always z.** Turn the joint frame with `orientation` to put z where
the motion is; the [orientation table](model-file.md#orientations) has the common cases.

The constraint force of each fixed direction is a result channel:
`hinge.dx.lambda`, `hinge.ay.lambda`, … ([Result channels](results.md)).

!!! tip "Redundant constraints"
    Closed loops built from joints often fix the same direction twice (a four-bar linkage of
    four revolute joints in a plane fixes the out-of-plane motion more than once). The checks
    report it as *more constraints than degrees of freedom*; replace a joint by one that fixes
    fewer directions (a spherical or a generic joint).
