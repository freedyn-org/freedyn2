# The model file

A model is one YAML file. It names the model, lists bodies, joints and forces, and says how
long and with which step to simulate. Everything else is optional.

```yaml
model: oscillator             # the name of the model (results: oscillator.csv)
gravity: [0, 0, -9.81]
parameters: {k: 200.0, d: 4.0}
bodies:
  mass:
    mass: 1.0
    inertia: [0.01, 0.01, 0.01]
    position: [0, 0, -0.5]
joints:
  slider: {type: prismatic, body1: mass, location: [0, 0, -0.5]}
forces:
  spring:
    type: spring_damper
    body1: mass
    point2: [0, 0, 0]          # on the ground
    stiffness: =k
    damping: =d
solver: {type: HHT, end_time: 2.0, step: 0.001, output_step: 0.01}
```

## Sections

<!-- keys: model -->

## Conventions

- **Units are SI**: m, kg, s, N, rad. The checks warn when gravity, lengths or masses look like
  another unit system.
- **Everything is given globally at t = 0**: the positions of bodies (their centers of mass),
  joint locations and frames, the points of springs and forces, the shapes. FreeDyn computes
  where each sits on its body. You never write a point in a body frame. Points used more than
  once get a name: [hardpoints and frames](hardpoints.md).
- **The ground** is the fixed frame; it is named `ground` where a body is expected (`body2`
  of a joint or spring defaults to it).
- **Names** of bodies, joints, forces, measures, parameters and splines are identifiers
  (letters, digits, `_`); they are the prefixes of the result channels (`rod.R_z`).

## Orientations

Orientations – of bodies (their principal axes), joint frames, bushing frames, shapes – are
unit quaternions `[θ0, θ1, θ2, θ3]` (Euler parameters, θ0 the scalar part), in global
components. They are normalized when read. The ones you need most:

| orientation | quaternion |
|---|---|
| identity (frame axes = global axes) | `[1, 0, 0, 0]` |
| z axis → global y (a hinge about y) | `[0.7071068, -0.7071068, 0, 0]` |
| z axis → global x | `[0.7071068, 0, 0.7071068, 0]` |
| 90° about z | `[0.7071068, 0, 0, 0.7071068]` |
| angle φ about a unit axis *n* | `[cos(φ/2), n_x sin(φ/2), n_y sin(φ/2), n_z sin(φ/2)]` |

With [formulas](#parameters-and-formulas) the last row can be written directly, e.g.
`orientation: [=cos(phi / 2), 0, 0, =sin(phi / 2)]` with a parameter `phi`.

## Parameters and formulas

`parameters` are named numbers. **Every number of the model may be a formula** of them,
written with `=` as in a spreadsheet – no quotes needed in YAML:

```yaml
parameters: {k_front: 30000, k_rear: =0.8 * k_front, h_mount: 0.55}
forces:
  susp_fl:
    type: spring_damper
    stiffness: =k_front
    point2: [2.6, 0.8, =h_mount]
```

- Formulas use the syntax of the [force expressions](measures.md#expressions): `+ - * / ^`,
  parentheses, `sqrt`, `sin`, `cos`, `exp`, `min`, `max`, `pi`, …, with the parameters as names.
- A parameter may be a formula of other parameters; the order does not matter, a cycle is
  reported.
- A formula is a **number fixed when the model is read** – not a function of time. Laws in
  time and state are force expressions, written without `=`.
- Parameters are used in force expressions and drivers by name (`force: -k * z`).
- In the viewer every parameter is a slider; everything written as a formula of it follows.

!!! warning "`#` starts a comment"
    `stiffness: #k` is an empty value in YAML. Formulas start with `=`; the error message
    says so if you slip.

## Data files

Spline tables, `.tir` tire files, OpenCRG roads, meshes and reference curves are given as paths
relative to the model file.
