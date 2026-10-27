# Writing an extension

An extension adds to FreeDyn 2.0 without changing it: **new force types** (with their own
keys in the model file), or new variants of existing ones (another tire model), and text for
the assistant. It is a Python package next to FreeDyn; FreeDyn finds it at start and uses it
for every model: in `freedyn2 run` and `check`, the viewer, the assistant's tools and the
key tables of the documentation.

This page builds a complete one step by step: **air drag at a point of a body**, a new force
type `aero_drag`. The finished extension is in the repository under `examples/extension`
(one Python file, one `pyproject.toml`), and a test (`tests/python/test_extension_example.py`)
runs it with every change of FreeDyn.

```yaml
forces:
  drag: {type: aero_drag, body1: car, point: [1.0, 0, 0.6], coefficient: 0.42, wind: [-5, 0, 0]}
```

## How it fits together

| step | who | what |
|---|---|---|
| reading the model | FreeDyn calls `schema` | the extension's keys are allowed in the file |
| validating | `validate_force` | the extension checks its forces (required keys, data files) |
| building | `force_element` | for each force it handles: the element type, the bodies, the numbers `p` |
| generating | `element_types` | the force law as a CasADi expression → C code with exact derivatives, compiled into the model plugin |
| simulating | the runtime | evaluates the element like any built-in one |

The law is written once, symbolically; FreeDyn derives the Jacobians the solver needs and
generates C code from them, as it does for its own elements. No C or C++ is written, and the
runtime is not changed.

## 1. The force law

A law is a function of a **connector** `c`: the symbols of the bodies the element connects
and of its parameters. It returns the **loads** on the bodies and the **outputs** (result
channels):

```python
import casadi as ca

N_P = 7  # parameters of the element: point u in body coordinates (3), c (1), wind w (3)

def drag_law(c):
    p, body = c.p, c.bodies[0]
    u, coef, wind = p[0:3], p[3], p[4:7]
    rel = body.point_velocity(u) - wind                   # velocity of the point against the air
    speed = ca.sqrt(ca.dot(rel, rel) + 1e-12)             # smooth at 0: the derivatives exist everywhere
    F = -coef * speed * rel
    loads = [(0, F, u, None)]                             # (body index, force, point, torque)
    outputs = {"force": F, "airspeed": speed}             # channels drag.force_x/y/z, drag.airspeed
    return loads, outputs
```

**What a body offers** (`c.bodies[k]`, all global components unless noted):

| | |
|---|---|
| `point(u)` | position of the body-fixed point `u` (body coordinates) |
| `point_velocity(u)` | its velocity |
| `axis(b)` | the body-fixed direction `b`, in global components |
| `R`, `Rdot` | position and velocity of the center of mass |
| `A` | rotation matrix (body to global) |
| `omega`, `omega_body` | angular velocity, global / body components |
| `point_acceleration(u)`, `omegadot` | accelerations, **for outputs only** (not in loads: the law would depend on the accelerations) |

`c.p` are the element's parameters (numbers the model gives, step 4), `c.t` the time. Use
CasADi functions (`ca.sqrt`, `ca.dot`, `ca.cross`, `ca.if_else`, `ca.atan2`, …) on these
symbols.

**Loads** are a list of

- `(k, F, u, M)`: force `F` (global) at the body-fixed point `u` of body `k`, and / or torque
  `M` (global); `None` for what is not there;
- `(None, s, rate)`: a generalized force `s` conjugate to a scalar quantity whose rate is
  `rate` (linear in the velocities) – for springs written in a measure, as the torsion spring
  of the core.

FreeDyn turns them into the generalized forces of the bodies by virtual power.

**Outputs** are result channels `<force name>.<key>`: a scalar gives one channel, a vector of
three `_x`, `_y`, `_z`. An output `force` (global) of a force with a `point` is drawn by the
viewer as a force arrow at that point.

**Smoothness.** Newton needs derivatives everywhere: avoid kinks and divisions by zero (`abs(x)`
→ `sqrt(x*x + eps)`, `if_else` blends instead of jumps where possible).

## 2. The element type

```python
from fdgen import forces

element_types = {
    "aero_drag": lambda rep: forces.force_element_type("aero_drag", 1, N_P, drag_law, rep),
}
```

`force_element_type(name, n_bodies, n_p, law, rep)` builds the element: `n_bodies` 1 (the
element acts between body1 and the ground) or 2 (between body1 and body2, `c.bodies[1]`),
`n_p` parameters. `rep` is the body type, `"lie"` (FreeDyn 2.0) or `"euler"` (legacy `.fds`
models); FreeDyn asks for the one it needs. The element is generated and compiled once per
process and model plugin, cached on disk (`FREEDYN2_PLUGIN_CACHE`); a C compiler is needed, as
for expression forces.

## 3. Keys in the model file

```python
def schema(self, schema):
    force = schema["$defs"]["force"]
    force["properties"]["type"]["enum"].append("aero_drag")
    force["properties"]["coefficient"] = {"type": "number", "minimum": 0,
                                          "description": "aero_drag: rho cw A / 2 [kg/m]."}
    force["properties"]["wind"] = {"$ref": "#/$defs/vec3", "description": "aero_drag: wind [m/s]."}
```

`schema` gets the JSON schema of the model file (`schema/model.schema.json`) and adds to it.
The descriptions appear in the assistant's schema tool and in documentation built with the
key tables (`<!-- keys: force -->`).

## 4. From the model to the numbers

```python
from fdgen.errors import ModelError
from fdgen.geometry import local_point

def validate_force(self, f, base_dir):
    if f["type"] == "aero_drag" and "coefficient" not in f:
        raise ModelError(f"force {f['name']}: aero_drag needs coefficient")
    return f

def force_element(self, force, bodies):
    if force["type"] != "aero_drag":
        return None                                   # not mine: the core or another extension
    u = local_point({"bodies": bodies}, force["body1"], force.get("point"))
    p = [*u, force["coefficient"], *force.get("wind", [0.0, 0.0, 0.0])]
    return "aero_drag", [force["body1"]], p
```

- `validate_force` sees every force after the core's rules (`f["name"]` is set, `body2`
  defaults to `ground`). Raise `ModelError` with the force's name for mistakes; read data
  files relative to `base_dir` (the model file's directory) and keep their content in `f`.
- `force_element` returns `(element type, [bodies], p)` for the forces the extension handles,
  `None` for all others. Everything in the model is a number by now: formulas (`=k`) and
  hardpoints are resolved. Points and directions are global at t = 0; `fdgen.geometry` turns
  them into body coordinates (`local_point`, `local_axis`, `body_frame`).

## 5. The extension and its entry point

```python
from fdgen.extensions import Extension

class Drag(Extension):
    name = "drag"
    element_types = {...}                    # step 2
    conventions = "Air drag: a force of type aero_drag ..."   # for the assistant's guide
    def schema(self, schema): ...            # step 3
    def validate_force(self, f, base_dir): ...
    def force_element(self, force, bodies): ...

extension = Drag()
```

```toml
# pyproject.toml
[project]
name = "freedyn2-drag"
dependencies = ["freedyn2"]

[project.entry-points."freedyn2.extensions"]
drag = "freedyn2_drag:extension"
```

`pip install .` and FreeDyn uses it. While developing, without installing:

```sh
FREEDYN2_EXTENSIONS=freedyn2_drag PYTHONPATH=path/to/the/package freedyn2 run model.yaml
```

## 6. Testing

Test an extension the way the example is tested (`tests/python/test_extension_example.py`):

- **Physics against a known solution.** A ball falling with drag `c v²` reaches the terminal
  velocity `sqrt(m g / c)` along `v(t) = v_t tanh(g t / v_t)`.
- **Derivatives against finite differences.** Evaluate the element's residual `res` and its
  iteration matrix `iter` at a state and compare the matrix with central differences of the
  residual.
- **The model builds.** The `.fdm` lists the element and the plugin.
- **Without the extension** the core rejects the extension's keys.

When FreeDyn and the extension run in the same Python process, the found extensions and the
schema are cached: a test that activates an extension clears `fdgen.extensions.extensions`
and `fdgen.model._validator` (`cache_clear()`) before and after.

## Everything an extension can do

| | |
|---|---|
| `name` | its name |
| `schema(schema)` | add keys and types to the JSON schema of the model file |
| `validate_force(f, base_dir)` | check and complete a force after the core rules |
| `force_element(force, bodies)` | take over a force: `(element type, [bodies], p)` |
| `element_types` | `{element type: build(rep)}`, usually `fdgen.forces.force_element_type(...)` |
| `road_offsets` | `{element type: offset}`: a tire element reads the runtime's road plane (OpenCRG) at this offset of its parameters, as the built-in tire `tire_local` (`fdgen.tire.LOCAL_KEYS`) |
| `wheel(force)` | `(radius, width)` of the wheel of a tire it handles (default shapes in the viewer, start checks) |
| `conventions` | a paragraph for the assistant's guide |

An extension can also take over forces of the core's types: a `tire` with an extra key, for
example, becomes the extension's tire model while all other tires stay built-in (the
`force_element` of the extension decides by the keys).

**Not (yet) extensible:** joints, body types, measures and sensors; the 3D drawing beyond
markers and force arrows; checks of the extension's own.

**Stability of the interface.** The hooks above, `fdgen.forces.force_element_type`, the body
kinematics of step 1 and `fdgen.geometry` are the interface. FreeDyn 2.0 is at 0.x: it may
still change; an extension's tests show when it does.
