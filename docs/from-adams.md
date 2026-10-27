# Coming from Adams

If you model in MSC Adams, most of FreeDyn 2.0 will be familiar – the same rigid bodies,
joints, forces and HHT integration. What is different is *how* a model is made and looked
at: a text file instead of a database, written by you or an assistant; a viewer that
re-simulates on every save; variant studies from a slider.

## Concepts side by side

| Adams | FreeDyn 2.0 |
|---|---|
| Part, Ground | [`bodies`](reference/bodies.md), `ground` |
| Marker, hardpoint, construction frame | none needed: points and frames are given **globally at t = 0** where they are used; shared ones are named [`points` and `frames`](reference/hardpoints.md) (`{mirror: ...}` for left / right, `{in: part, local: ...}` for part coordinates) |
| CM marker, inertia tensor | `position` (center of mass), `orientation` (principal axes), `inertia` (principal moments) |
| Revolute, Translational, Cylindrical, Spherical, Fixed | [joints](reference/joints.md) `revolute`, `prismatic`, `cylindrical`, `spherical`, `fixed` |
| Planar, JPRIMs (inline, inplane, orientation, parallel axes, perpendicular) | `generic` with the directions to fix |
| Motion (displacement, angle) | [`drivers`](reference/measures.md#drivers) on a measure |
| Spring-damper (translational, rotational) | [`spring_damper`](reference/forces.md#spring-damper), `torsion_spring` |
| Bushing | [`bushing`](reference/forces.md#bushing) |
| Single-component force / torque (SFORCE), VFORCE | [`expression_force`](reference/forces.md#expression-force-and-torque), `expression_torque` |
| DX, DZ, VZ, AZ, DM, … in expressions | [`measures`](reference/measures.md), named and used by name |
| STEP, IF, AKISPL, CUBSPL | `step()`, `if_else()`, [`splines`](reference/measures.md#splines) (`akima`) |
| Design variables, parametric expressions | [`parameters`](reference/model-file.md#parameters-and-formulas) and `=formulas` in any field |
| Tire (MF-Tyre / PAC), road (RDF, CRG) | [`tire`](reference/tires-roads.md) (a built-in Magic Formula tire; `.tir` data sets through [extensions](extensions.md)), analytic road or OpenCRG |
| Requests, measures for post-processing | result channels, formula channels in the viewer |
| Accelerometer requests (ACCX, ACCY, ACCZ, WDTX …) | [`sensors`](reference/sensors.md): acceleration, specific force, angular velocity and acceleration at a point |
| Dynamic simulation, HHT / GSTIFF | [`solver`](reference/solver.md): HHT on a Lie group, fixed step |
| Linear analysis (eigenvalues) | Eigenmodes and root locus in the viewer, `modes` / `locus` for the assistant |
| Design evaluation, DOE (one factor) | the fan (≋) in the viewer, `sweep` for the assistant |
| Adams PostProcessor | the [viewer](viewer.md): plots, x-y, FFT, cursors, run comparison, Δ plots, report |

## What is different

- **Units are SI**, always. An Adams model in mm, kg, N, s needs its lengths divided by 1000
  and its stiffnesses converted; the checks warn when a model looks like mm.
- **No markers.** Where Adams attaches a joint to marker I on part 1 and marker J on part 2,
  FreeDyn gives the joint one global location and orientation at t = 0; both bodies carry it
  from there. The same for springs (two global points) and forces.
- **Orientations are quaternions**, not Euler angles (body 313 etc.): no singular angles. A
  rotation by φ about an axis *n* is `[cos(φ/2), n sin(φ/2)]`; formulas make it readable
  (`[=cos(phi/2), 0, 0, =sin(phi/2)]`).
- **The model is a file.** Diff it, review it, keep it in git, let an assistant change it –
  and see every change in the viewer the moment it is saved.
- **Inertia as principal moments.** Give the principal moments and turn the body frame to the
  principal axes (eigen-decomposition of the Adams inertia tensor).

## Not (yet) in FreeDyn 2.0

Flexible bodies, contacts (other than tires), couplers and gears, universal and
constant-velocity joints, friction in joints, static and kinematic analyses (start from rest
and let it settle, or prescribe the motion with drivers), co-simulation interfaces. An
importer for Adams `.adm` (and `.cmd` with design variables) is planned; until then a model is
quickest rebuilt by an assistant from the Adams listing.

## A first translation

An Adams pendulum – a part on a revolute joint to ground, its I and J markers at the pivot
with z along global y – becomes:

```yaml
model: pendulum
gravity: [0, 0, -9.81]
bodies:
  link:                                  # PART/2, CM at (500, 0, 0) mm
    mass: 1.0
    inertia: [0.0008, 0.0834, 0.0834]    # IXX, IYY, IZZ about the CM, principal axes = global
    position: [0.5, 0, 0]
joints:
  pivot:                                 # JOINT/1, REVOLUTE, I = 2.3, J = 1.1
    type: revolute
    body1: link
    location: [0, 0, 0]
    orientation: [0.7071068, -0.7071068, 0, 0]   # z of the markers along global y
solver: {type: HHT, end_time: 2.0, step: 0.001}
```
