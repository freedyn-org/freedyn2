"""freedyn2 fmu: a model as a Functional Mock-up Unit for co-simulation (FMI 2.0 or 3.0).

  freedyn2 fmu model.yaml [-o model.fmu] [--fmi 2|3]

The FMU holds the generic FMU library of the runtime (freedyn2_fmu, the same for every model;
binaries/<platform>/<model>.so|dll), the runtime model file as resources/model.fdm with its model
plugin and data files (roads), and the modelDescription.xml. Variables (value references in the
order of freedyn2_run --channels, as runtime/fmu/slave.hpp reads them):

- inputs: the runtime inputs of the model (`inputs:`), Real / Float64, start = their default;
- outputs: every output channel of the run (as the columns of its CSV result);
- FMI 3.0 also the independent variable time.

The model's time starts at the start time of the co-simulation; a communication step is
integrated with the model's step (`solver: step`), the last step to the communication point;
events (contacts) are located within. The FMU runs on the platform it was made on (its binary
and the model plugin are compiled for it).
"""

import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import uuid
import zipfile
from pathlib import Path
from xml.sax.saxutils import quoteattr

from . import __version__, cli, resources
from .errors import ModelError

LIBRARY = "freedyn2_fmu" + (".dll" if sys.platform == "win32" else ".dylib" if sys.platform == "darwin" else ".so")


def find_library(path=None):
    """The FMU library of the runtime: path, FREEDYN2_FMU, the installed package, the build directory."""
    for c in [path, os.environ.get("FREEDYN2_FMU"), resources.PACKAGE / "bin" / LIBRARY,
              *(d / LIBRARY for d in cli.BUILD_DIRS)]:
        if c and Path(c).is_file():
            return Path(c).resolve()
    raise FileNotFoundError(f"{LIBRARY} not found: build it (cmake --build build --target freedyn2_fmu) "
                            "or set FREEDYN2_FMU")


def _platform(fmi):
    machine = {"amd64": "x86_64", "x86_64": "x86_64", "arm64": "aarch64", "aarch64": "aarch64"}.get(
        platform.machine().lower(), "x86_64")
    system = {"win32": "windows", "darwin": "darwin"}.get(sys.platform, "linux")
    if fmi == "2":
        if machine != "x86_64":
            raise ModelError("FMI 2.0 FMUs of freedyn2 are made for x86-64 only; use --fmi 3")
        return {"windows": "win64", "darwin": "darwin64", "linux": "linux64"}[system]
    return f"{machine}-{system}"


def identifier(name):
    """A C identifier of the model name (modelIdentifier: the file name of the binary)."""
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return s if re.match(r"[A-Za-z_]", s) else "m_" + s


def channels(runtime, fdm):
    """([(input name, default)], [output names]) of a model file (freedyn2_run --channels)."""
    out = subprocess.run([str(runtime), str(fdm), "--channels"], capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError(f"{runtime.name} --channels failed:\n{out.stdout}{out.stderr}")
    inputs, outputs = [], []
    for line in out.stdout.splitlines():
        kind, _, rest = line.partition(" ")
        if kind == "input":
            name, value = rest.rsplit(" ", 1)
            inputs.append((name, float(value)))
        elif kind == "output":
            outputs.append(rest)
    return inputs, outputs


def _resources(fdm, target):
    """model.fdm in target with its plugin and data files beside it (paths in it: file names)."""
    target.mkdir(parents=True)
    lines = []
    for line in Path(fdm).read_text(encoding="utf-8").splitlines():
        key, _, rest = line.partition(" ")
        if key in ("plugin", "road_crg", "road_friction_crg"):
            src = Path(rest.strip())
            src = src if src.is_absolute() else Path(fdm).parent / src
            shutil.copyfile(src, target / src.name)
            line = f"{key} {src.name}"
        lines.append(line)
    (target / "model.fdm").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _xml_fmi2(name, ident, guid, inputs, outputs, solver):
    v = []
    for k, (n, start) in enumerate(inputs):
        v.append(f'    <ScalarVariable name={quoteattr(n)} valueReference="{k}" causality="input" '
                 f'variability="continuous"><Real start="{start!r}"/></ScalarVariable>')
    for k, n in enumerate(outputs):
        v.append(f'    <ScalarVariable name={quoteattr(n)} valueReference="{len(inputs) + k}" causality="output" '
                 'variability="continuous" initial="calculated"><Real/></ScalarVariable>')
    unknowns = "\n".join(f'      <Unknown index="{len(inputs) + k + 1}"/>' for k in range(len(outputs)))
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="2.0" modelName={quoteattr(name)} guid="{{{guid}}}"
  description={quoteattr(f"freedyn2 model {name}")} generationTool="freedyn2 {__version__}"
  variableNamingConvention="flat" numberOfEventIndicators="0">
  <CoSimulation modelIdentifier="{ident}" canHandleVariableCommunicationStepSize="true"
    canInterpolateInputs="false" canBeInstantiatedOnlyOncePerProcess="false"
    canNotUseMemoryManagementFunctions="true" canGetAndSetFMUstate="false" canSerializeFMUstate="false"
    providesDirectionalDerivative="false"/>
  <DefaultExperiment startTime="0" stopTime="{solver['end_time']!r}" stepSize="{solver['output_step']!r}"/>
  <ModelVariables>
{chr(10).join(v)}
  </ModelVariables>
  <ModelStructure>
    <Outputs>
{unknowns}
    </Outputs>
    <InitialUnknowns>
{unknowns}
    </InitialUnknowns>
  </ModelStructure>
</fmiModelDescription>
'''


def _xml_fmi3(name, ident, guid, inputs, outputs, solver):
    v = []
    for k, (n, start) in enumerate(inputs):
        v.append(f'    <Float64 name={quoteattr(n)} valueReference="{k}" causality="input" variability="continuous" '
                 f'start="{start!r}"/>')
    for k, n in enumerate(outputs):
        v.append(f'    <Float64 name={quoteattr(n)} valueReference="{len(inputs) + k}" causality="output" '
                 'variability="continuous"/>')
    v.append(f'    <Float64 name="time" valueReference="{len(inputs) + len(outputs)}" causality="independent" '
             'variability="continuous"/>')
    unknowns = "\n".join(f'    <Output valueReference="{len(inputs) + k}"/>' for k in range(len(outputs)))
    unknowns += "\n" + "\n".join(f'    <InitialUnknown valueReference="{len(inputs) + k}"/>' for k in range(len(outputs)))
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription fmiVersion="3.0" modelName={quoteattr(name)} instantiationToken="{{{guid}}}"
  description={quoteattr(f"freedyn2 model {name}")} generationTool="freedyn2 {__version__}"
  variableNamingConvention="flat">
  <CoSimulation modelIdentifier="{ident}" canHandleVariableCommunicationStepSize="true"
    fixedInternalStepSize="{solver['step']!r}" canGetAndSetFMUState="false" canSerializeFMUState="false"
    providesDirectionalDerivatives="false" providesAdjointDerivatives="false" hasEventMode="false"
    providesIntermediateUpdate="false" canReturnEarlyAfterIntermediateUpdate="false"/>
  <DefaultExperiment startTime="0" stopTime="{solver['end_time']!r}" stepSize="{solver['output_step']!r}"/>
  <ModelVariables>
{chr(10).join(v)}
  </ModelVariables>
  <ModelStructure>
{unknowns}
  </ModelStructure>
</fmiModelDescription>
'''


def export(model_path, out=None, fmi="2", profile="v2", runtime=None, library=None):
    """Write the FMU of a model file; returns (path, {inputs, outputs})."""
    if fmi not in ("2", "3"):
        raise ModelError("fmi must be 2 or 3")
    runtime = cli.find_runtime(runtime)
    library = find_library(library)
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        (tmp / "build").mkdir()
        fdm, model, _ = cli.build(model_path, tmp / "build", profile)
        inputs, outputs = channels(runtime, fdm)
        name = model["model"]
        ident = identifier(name)
        out = Path(out) if out else Path(model_path).with_suffix(".fmu")
        root = tmp / "fmu"
        _resources(fdm, root / "resources")
        binaries = root / "binaries" / _platform(fmi)
        binaries.mkdir(parents=True)
        shutil.copyfile(library, binaries / (ident + Path(LIBRARY).suffix))
        xml = (_xml_fmi2 if fmi == "2" else _xml_fmi3)(name, ident, uuid.uuid4(), inputs, outputs, model["solver"])
        (root / "modelDescription.xml").write_text(xml, encoding="utf-8")
        (root / "documentation").mkdir()
        for lic in ("LICENSE", "NOTICE"):  # freedyn2's licenses (the runtime in the binary)
            src = resources.path(lic)
            if Path(src).is_file():
                shutil.copyfile(src, root / "documentation" / lic)
        out.parent.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
            for f in sorted(root.rglob("*")):
                if f.is_file():
                    z.write(f, f.relative_to(root).as_posix())
    return out, {"inputs": [n for n, _ in inputs], "outputs": outputs}
