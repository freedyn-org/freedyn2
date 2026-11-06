"""Build the Windows installer of FreeDyn 2 (Inno Setup 6) from the Windows wheel.

  python installer/windows/build.py stage   --wheel-dir wheelhouse [--work build/installer]
  python installer/windows/build.py compile [--work build/installer] [--out dist] [--iscc PATH]
  python installer/windows/build.py all     --wheel-dir wheelhouse [...]   (stage, then compile)

stage: downloads the pinned python-build-standalone CPython (SHA256 checked), extracts it, downloads
freedyn2[gui,compiler] and its dependencies as wheels (pip download), installs them offline from
there into that Python, makes the launchers (freedyn2.exe, freedyn2-gui.exe) relocatable, strips
caches and tests, precompiles the bytecode and writes three trees for the installer components:

  WORK/stage/core       {app}: python\\ (CPython with everything but Zig), bin\\ (the launchers),
                        licenses\\, freedyn2.ico
  WORK/stage/compiler   {app}: python\\Lib\\site-packages\\ziglang (component "compiler")
  WORK/stage/examples   {app}: examples\\ (component "examples")

Between stage and compile, CI may sign the programs in WORK/stage (.github/workflows/wheels.yml).

compile: writes WORK/stage/THIRD-PARTY-NOTICES.rtf (the license page of the wizard, from
THIRD-PARTY-NOTICES.txt) and runs ISCC on freedyn2.iss with the version of the wheel; output
OUT/FreeDyn2-<version>-setup.exe.

--dry-run (any OS, e.g. Linux): the same downloads and extraction, the dependencies resolved for
Windows with uv (pip on Linux would evaluate the environment markers for Linux) and installed for
the win_amd64 platform with pip --target; no launchers, no bytecode, ISCC not run but its inputs
checked. For testing the logic of this script without Windows.
"""

import argparse
import email.parser
import hashlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import urllib.request
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent

# python-build-standalone (https://github.com/astral-sh/python-build-standalone), CPython 3.12 for
# x86_64-pc-windows-msvc, install_only (stripped of debug symbols). To update: take the new file
# name and its SHA256 from the release's SHA256SUMS.
PYTHON_RELEASE = "20261021"
PYTHON_FILE = "cpython-3.12.15+20261021-x86_64-pc-windows-msvc-install_only_stripped.tar.gz"
PYTHON_SHA256 = "aac5d331fc3307343f95928fdbea4583f0d53973d08c7e1d081884fbd229a0e1"
PYTHON_URL = ("https://github.com/astral-sh/python-build-standalone/releases/download/"
              f"{PYTHON_RELEASE}/{PYTHON_FILE.replace('+', '%2B')}")
PY_TAGS = ["--platform", "win_amd64", "--python-version", "3.12", "--implementation", "cp", "--abi", "cp312"]

EXTRAS = "gui,compiler"
COMPILER_DISTS = ["ziglang"]  # the packages of the component "compiler" (not installed if unchecked)
# Directory names removed from site-packages (tests of the packages) and from the standard library.
# Not touched: fdgen, the compiler packages.
STRIP_DIRS = {"tests"}
STRIP_STDLIB = ["test", "idlelib", "turtledemo"]
KEEP_TESTS_IN = {"fdgen", *COMPILER_DISTS}


def log(*a):
    print("[build_installer]", *a, flush=True)


def run(*cmd, **kw):
    print("+", " ".join(map(str, cmd)), flush=True)
    subprocess.run([str(c) for c in cmd], check=True, **kw)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def download_python(downloads):
    """The pinned CPython archive (cached in downloads), its SHA256 checked."""
    downloads.mkdir(parents=True, exist_ok=True)
    target = downloads / PYTHON_FILE
    if not (target.is_file() and sha256(target) == PYTHON_SHA256):
        log("downloading", PYTHON_URL)
        part = target.with_suffix(".part")
        with urllib.request.urlopen(PYTHON_URL) as r, open(part, "wb") as f:
            shutil.copyfileobj(r, f)
        got = sha256(part)
        if got != PYTHON_SHA256:
            part.unlink()
            raise SystemExit(f"SHA256 mismatch for {PYTHON_FILE}: {got}, expected {PYTHON_SHA256}")
        part.replace(target)
    return target


def extract_python(archive, dest):
    """The archive holds python/ (python.exe, Lib, DLLs, ...): extracted to dest/python."""
    if (dest / "python").exists():
        shutil.rmtree(dest / "python")
    dest.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive) as tar:
        names = tar.getnames()
        if not all(n == "python" or n.startswith("python/") for n in names):
            raise SystemExit(f"unexpected layout of {archive.name}: not everything under python/")
        tar.extractall(dest, filter="data")
    if not (dest / "python" / "python.exe").is_file():
        raise SystemExit("python.exe missing in the extracted CPython")
    return dest / "python"


def find_wheel(wheel_dir, dry_run):
    found = sorted(Path(wheel_dir).glob("freedyn2-*.whl"))
    windows = [w for w in found if w.name.endswith("win_amd64.whl")]
    if len(windows) == 1:
        return windows[0]
    if dry_run and len(found) == 1:
        return found[0]
    raise SystemExit(f"need exactly one freedyn2-*-win_amd64.whl in {wheel_dir}, found {[w.name for w in found]}")


def wheel_version(wheel):
    # freedyn2-0.1.0-py3-none-win_amd64.whl (the version is normalized in wheel names)
    return wheel.name.split("-")[1]


def numeric_version(version):
    """0.1.0 -> 0.1.0.0, 0.2.0rc1 -> 0.2.0.0: the Windows file version (four numbers)."""
    nums = re.match(r"\d+(\.\d+)*", version).group(0).split(".")
    return ".".join((nums + ["0"] * 4)[:4])


def download_packages(py, wheel, wheels, find_links, dry_run):
    """freedyn2[gui,compiler] and all its dependencies as wheels into `wheels` (for pip install --no-index)."""
    wheels.mkdir(parents=True, exist_ok=True)
    shutil.copy2(wheel, wheels / wheel.name)
    req = f"{wheels / wheel.name}[{EXTRAS}]"
    if not dry_run:
        # The Python of the installer itself (markers and tags of Windows, CPython 3.12). pip wheel:
        # downloads the wheels like pip download, and builds one for a dependency that has only a
        # source distribution (proxy_tools, of pywebview), so that the installation is offline.
        run(py, "-I", "-m", "pip", "wheel", "--disable-pip-version-check", "--prefer-binary",
            "--wheel-dir", wheels, "--find-links", find_links, req)
        return
    # Linux: resolve for Windows with uv (environment markers), then fetch the wheels for win_amd64
    uv = shutil.which("uv") or sys.exit("--dry-run needs uv on the PATH (pip install uv) to resolve for Windows")
    lock = wheels.parent / "requirements-windows.txt"
    (wheels.parent / "requirements.in").write_text(f"freedyn2[{EXTRAS}] @ {wheels.joinpath(wheel.name).as_uri()}\n")
    run(uv, "pip", "compile", "--quiet", "--python-platform", "x86_64-pc-windows-msvc", "--python-version", "3.12",
        "--no-header", "--no-annotate", "-o", lock, wheels.parent / "requirements.in")
    reqs = [line for line in lock.read_text().splitlines() if line.strip() and not line.startswith("freedyn2")]
    lock.write_text("\n".join(reqs) + "\n")
    for r in reqs:  # a wheel for Windows, else (a source distribution only) a wheel built here: pure Python
        try:
            run(sys.executable, "-m", "pip", "download", "-q", "--disable-pip-version-check", "--only-binary=:all:",
                "--no-deps", *PY_TAGS, "--dest", wheels, r)
        except subprocess.CalledProcessError:
            log(f"no Windows wheel of {r}: building one from its source distribution")
            run(sys.executable, "-m", "pip", "wheel", "-q", "--disable-pip-version-check", "--no-deps",
                "--wheel-dir", wheels, r)


def site_packages(pyhome):
    return pyhome / "Lib" / "site-packages"


def install_packages(py, pyhome, wheel, wheels, dry_run):
    req = f"{wheels / wheel.name}[{EXTRAS}]"
    common = ["--disable-pip-version-check", "--no-index", "--find-links", wheels, "--no-compile", "--no-cache-dir"]
    if not dry_run:
        run(py, "-I", "-m", "pip", "install", "--no-warn-script-location", *common, req)
    else:
        run(sys.executable, "-m", "pip", "install", *common, "--only-binary=:all:", *PY_TAGS,
            "--target", site_packages(pyhome), "-r", wheels.parent / "requirements-windows.txt", "--no-deps")
        run(sys.executable, "-m", "pip", "install", *common, *PY_TAGS, "--target", site_packages(pyhome),
            "--no-deps", wheels / wheel.name)
        shutil.rmtree(site_packages(pyhome) / "bin", ignore_errors=True)  # POSIX scripts of --target


def dist_infos(sp):
    """{normalized name: (dist-info dir, metadata message)} of the installed distributions."""
    out = {}
    for d in sorted(sp.glob("*.dist-info")):
        meta = email.parser.Parser().parsestr((d / "METADATA").read_text(encoding="utf-8", errors="replace"))
        out[re.sub(r"[-_.]+", "-", meta["Name"]).lower()] = (d, meta)
    return out


def entry_point_names(dist_info):
    """The console and GUI scripts of a distribution: {name: "console"|"gui"}."""
    f = dist_info / "entry_points.txt"
    names, section = {}, None
    for line in (f.read_text().splitlines() if f.is_file() else []):
        line = line.strip()
        if line.startswith("["):
            section = line.strip("[]")
        elif "=" in line and section in ("console_scripts", "gui_scripts"):
            names[line.split("=")[0].strip()] = "gui" if section == "gui_scripts" else "console"
    return names


def relocate_launcher(data, relative):
    """A pip launcher (distlib: stub + shebang + zip) with the shebang `#!<launcher_dir>\\RELATIVE\\pythonX.exe`.

    pip writes the absolute path of the build's python.exe; the distlib launchers (t64.exe, w64.exe)
    resolve a shebang path that starts with <launcher_dir>\\ relative to the launcher itself.
    """
    with zipfile.ZipFile(io.BytesIO(data)) as zf:  # where the appended zip starts
        zip_start = min(i.header_offset for i in zf.infolist())
    start = data.rfind(b"#!", 0, zip_start)
    line = data[start:zip_start]
    m = re.fullmatch(rb'#!"?[^\n"]*?(pythonw?\.exe)"?[ \t]*\r?\n', line, re.I)
    if start < 0 or not m:
        raise ValueError(f"no python shebang before the appended zip: {line[:200]!r}")
    new = b"#!<launcher_dir>\\" + relative.encode() + b"\\" + m.group(1) + b"\n"
    return data[:start] + new + data[zip_start:]


def make_launchers(core, pyhome, dists):
    """freedyn2's launchers (Scripts\\*.exe): relocatable, and copied to bin\\ (the directory on the PATH)."""
    scripts, bin_dir = pyhome / "Scripts", core / "bin"
    bin_dir.mkdir(exist_ok=True)
    ours = entry_point_names(dists["freedyn2"][0])
    log("launchers:", ", ".join(f"{n} ({k})" for n, k in ours.items()))
    if "freedyn2-gui" not in ours:
        log("WARNING: freedyn2 has no gui script freedyn2-gui (the installer's shortcuts need it)")
    for exe in sorted(scripts.glob("*.exe")):
        name = exe.stem
        if name in ours:
            data = exe.read_bytes()
            exe.write_bytes(relocate_launcher(data, ".."))                      # python\Scripts -> python\
            (bin_dir / exe.name).write_bytes(relocate_launcher(data, r"..\python"))  # bin -> python\
        else:  # launchers of the dependencies (pip, f2py, ...): absolute paths of this build, useless
            exe.unlink()
    for f in scripts.glob("*"):  # e.g. activate scripts with absolute paths
        if f.is_file() and f.suffix.lower() not in (".exe",):
            f.unlink()


def split_compiler(core_sp, compiler_sp, dists):
    """The compiler's packages (ziglang) out of the core tree into the tree of the component "compiler"."""
    compiler_sp.mkdir(parents=True, exist_ok=True)
    for name in COMPILER_DISTS:
        if name not in dists:
            log(f"WARNING: {name} is not installed (freedyn2[compiler]): the component compiler will be empty")
            continue
        dist_info, _ = dists[name]
        record = (dist_info / "RECORD").read_text().splitlines()
        tops = {line.split(",")[0].split("/")[0] for line in record if line and not line.startswith("..")}
        for top in sorted(tops):
            src = core_sp / top
            if src.exists():
                shutil.move(str(src), str(compiler_sp / top))
        log(f"{name}: {', '.join(sorted(tops))} -> component compiler")


def strip(pyhome, extra_trees=()):
    removed = 0
    for tree in (pyhome, *extra_trees):
        for d in sorted(tree.rglob("__pycache__"), reverse=True):
            shutil.rmtree(d, ignore_errors=True)
            removed += 1
    for name in STRIP_STDLIB:
        if (pyhome / "Lib" / name).is_dir():
            shutil.rmtree(pyhome / "Lib" / name)
            removed += 1
    sp = site_packages(pyhome)
    for top in sorted(sp.iterdir()):
        if not top.is_dir() or top.name in KEEP_TESTS_IN or top.name.endswith((".dist-info", ".data")):
            continue
        for d in sorted((p for p in top.rglob("*") if p.is_dir() and p.name in STRIP_DIRS), reverse=True):
            if d.exists():
                shutil.rmtree(d)
                removed += 1
    log(f"stripped {removed} directories (caches, tests)")


def compile_bytecode(py, trees):
    """Bytecode now: {autopf} is read-only for its users, and the first start would compile everything.

    checked-hash: valid independently of the files' times (the installer keeps them, but be safe).
    Files of the standard library that do not compile (none expected after strip) only warn.
    """
    for tree in trees:
        r = subprocess.run([str(py), "-I", "-m", "compileall", "-q", "-j", "0", "--invalidation-mode",
                            "checked-hash", str(tree)])
        if r.returncode:
            log(f"WARNING: compileall reported errors in {tree}")


def collect_licenses(core, pyhome, dists):
    """licenses\\python-packages.txt: every bundled distribution with its license (from its metadata);
    licenses\\Python-LICENSE.txt: CPython's license (python-build-standalone ships it)."""
    lic = core / "licenses"
    lic.mkdir(exist_ok=True)
    for cand in (pyhome / "LICENSE.txt", pyhome / "LICENSE"):
        if cand.is_file():
            shutil.copyfile(cand, lic / "Python-LICENSE.txt")
            break
    else:
        log("WARNING: no LICENSE.txt in the CPython archive")
    rows = []
    for name, (d, meta) in sorted(dists.items()):
        lic_text = meta.get("License-Expression") or ""
        if not lic_text:
            short = (meta.get("License") or "").strip().splitlines()
            lic_text = short[0][:80] if short and len(short) == 1 else ""
        if not lic_text:
            classifiers = [c.split("::")[-1].strip() for c in meta.get_all("Classifier") or [] if c.startswith("License ::")]
            lic_text = "; ".join(classifiers) or "see its license files"
        files = sorted(str(p.relative_to(pyhome.parent)).replace("/", "\\") for p in d.rglob("*")
                       if p.is_file() and re.match(r"(LICEN[CS]E|COPYING|NOTICE|AUTHORS)", p.name, re.I))
        rows.append((meta["Name"], meta["Version"], lic_text, files))
    with open(lic / "python-packages.txt", "w", encoding="utf-8", newline="\r\n") as f:
        f.write("Python packages bundled with FreeDyn 2 (generated by installer/windows/build.py from their metadata).\n"
                "Their license texts are in the dist-info directories named below (under the installation directory).\n\n")
        for name, version, text, files in rows:
            f.write(f"{name} {version}\n  License: {text}\n")
            for p in files:
                f.write(f"  {p}\n")
            f.write("\n")
    return rows


def stage(args):
    work = Path(args.work).resolve()
    wheel = find_wheel(args.wheel_dir, args.dry_run)
    version = wheel_version(wheel)
    log(f"freedyn2 {version} from {wheel.name}{' (dry run)' if args.dry_run else ''}")
    st = work / "stage"
    if st.exists():
        shutil.rmtree(st)
    core, compiler, examples = st / "core", st / "compiler", st / "examples"
    if args.python_archive:  # a local copy (offline builds; tests)
        archive = Path(args.python_archive).resolve()
        if sha256(archive) != PYTHON_SHA256:
            if not args.dry_run:
                raise SystemExit(f"{archive}: SHA256 differs from the pinned {PYTHON_FILE}")
            log(f"dry run: {archive.name} is not the pinned {PYTHON_FILE} (SHA256 differs), used anyway")
    else:
        archive = download_python(work / "downloads")
    pyhome = extract_python(archive, core)
    py = pyhome / "python.exe"
    wheels = work / "wheels"
    if wheels.exists():
        shutil.rmtree(wheels)
    download_packages(py, wheel, wheels, Path(args.wheel_dir).resolve(), args.dry_run)
    install_packages(py, pyhome, wheel, wheels, args.dry_run)
    dists = dist_infos(site_packages(pyhome))
    if "freedyn2" not in dists:
        raise SystemExit("freedyn2 not installed")
    if not args.dry_run:
        make_launchers(core, pyhome, dists)
    else:
        log("dry run: launchers not made (pip --target makes none for Windows)")
    split_compiler(site_packages(pyhome), site_packages(compiler / "python"), dists)
    strip(pyhome, [compiler])
    fdgen = site_packages(pyhome) / "fdgen"
    ex = fdgen / "examples"
    if ex.is_dir():
        shutil.copytree(ex, examples / "examples", ignore=shutil.ignore_patterns("__pycache__"))
    else:
        log("WARNING: no examples in the wheel (fdgen/examples)")
    icon = fdgen / "app" / "freedyn2.ico"
    if icon.is_file():
        shutil.copyfile(icon, core / "freedyn2.ico")
    else:
        log("WARNING: no icon fdgen/app/freedyn2.ico in the wheel: installer without icon")
    rows = collect_licenses(core, pyhome, dists)
    if not args.dry_run:
        compile_bytecode(py, [pyhome, compiler])
    info = {"version": version, "version_numeric": numeric_version(version), "wheel": wheel.name,
            "python": PYTHON_FILE, "packages": {r[0]: r[1] for r in rows}, "dry_run": args.dry_run}
    (st / "buildinfo.json").write_text(json.dumps(info, indent=2))
    sizes = {t.name: sum(f.stat().st_size for f in t.rglob("*") if f.is_file()) for t in (core, compiler, examples)}
    log("stage ready:", ", ".join(f"{k} {v / 1e6:.0f} MB" for k, v in sizes.items()))
    return info


def find_iscc(explicit):
    cands = [explicit, os.environ.get("ISCC"), shutil.which("ISCC"), shutil.which("iscc")]
    for base in (os.environ.get("ProgramFiles(x86)"), os.environ.get("ProgramFiles"),
                 os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs")):
        if base:
            cands.append(os.path.join(base, "Inno Setup 6", "ISCC.exe"))
    return next((c for c in cands if c and Path(c).is_file()), None)


def check_iss_sources(iss, defines):
    """The files that freedyn2.iss takes (Source: and the setup's own files) exist: what ISCC would check first."""
    missing = []
    for line in iss.read_text(encoding="utf-8").splitlines():
        m = re.match(r'\s*(?:Source:|LicenseFile=|InfoBeforeFile=|InfoAfterFile=)\s*"?([^";\r\n]+)"?', line)
        if not m or "skipifsourcedoesntexist" in line:
            continue
        raw = m.group(1).strip()
        path = re.sub(r"\{#(\w+)\}", lambda k: defines.get(k.group(1), k.group(0)), raw).replace("\\", "/")
        if "{" in path:
            continue
        p = Path(path) if Path(path).is_absolute() else iss.parent / path
        if "*" in p.name:
            if not p.parent.is_dir():
                missing.append(raw)
        elif not p.exists():
            missing.append(raw)
    return missing


def rtf_escape(text):
    out = []
    for ch in text:
        if ch in "\\{}":
            out.append("\\" + ch)
        elif ord(ch) > 127:
            out.append(f"\\u{ord(ch) if ord(ch) < 32768 else ord(ch) - 65536}?")
        else:
            out.append(ch)
    return "".join(out)


def notices_blocks(text):
    """THIRD-PARTY-NOTICES.txt as blocks (kind, text): kind "title" (underlined with =), "heading"
    (underlined with -), "item" (a line starting with "- ", continued by indented lines) or "para"
    (lines up to a blank line). Wrapped lines are joined: the wizard page wraps them itself."""
    blocks, lines = [], text.replace("\r\n", "\n").split("\n")
    i = 0
    while i < len(lines):
        line = lines[i].rstrip()
        nxt = lines[i + 1].strip() if i + 1 < len(lines) else ""
        if not line.strip():
            i += 1
        elif nxt and set(nxt) <= {"="}:
            blocks.append(("title", line.strip()))
            i += 2
        elif nxt and set(nxt) <= {"-"}:
            blocks.append(("heading", line.strip()))
            i += 2
        elif line.startswith("- "):
            parts = [line[2:].strip()]
            i += 1
            while i < len(lines) and lines[i].startswith("  ") and lines[i].strip():
                parts.append(lines[i].strip())
                i += 1
            blocks.append(("item", " ".join(parts)))
        else:
            parts = []
            while i < len(lines) and lines[i].strip() and not lines[i].startswith("- "):
                parts.append(lines[i].strip())
                i += 1
            blocks.append(("para", " ".join(parts)))
    return blocks


def notices_rtf(text):
    """The notices for the wizard page (InfoBeforeFile): Inno Setup shows a .txt there in a narrow
    box with a proportional font and wraps it again, which breaks hard-wrapped lines and aligned
    columns; as RTF, with real paragraphs, bold headings and hanging list indents."""
    out = [r"{\rtf1\ansi\ansicpg1252\deff0\uc1{\fonttbl{\f0\fswiss Segoe UI;}}\viewkind4\f0\fs17"]
    for kind, body in notices_blocks(text):
        t = rtf_escape(body)
        if kind == "title":
            out.append(rf"\pard\sa120\b\fs22 {t}\b0\fs17\par")
        elif kind == "heading":
            out.append(rf"\pard\sb160\sa60\b {t}\b0\par")
        elif kind == "item":
            out.append(rf"\pard\fi-180\li180\sa60 \bullet\tab {t}\par")
        else:
            out.append(rf"\pard\sa100 {t}\par")
    out.append("}")
    return "\r\n".join(out) + "\r\n"


def compile_installer(args):
    work = Path(args.work).resolve()
    st = work / "stage"
    info = json.loads((st / "buildinfo.json").read_text())
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    iss = HERE / "freedyn2.iss"
    defines = {"AppVersion": info["version"], "AppVersionNumeric": info["version_numeric"],
               "StageDir": str(st), "OutputDir": str(out), "RepoDir": str(REPO)}
    notices = (HERE / "THIRD-PARTY-NOTICES.txt").read_text(encoding="utf-8")
    (st / "THIRD-PARTY-NOTICES.rtf").write_text(notices_rtf(notices), encoding="ascii", newline="")
    missing = check_iss_sources(iss, defines)
    if missing:
        raise SystemExit(f"files missing for {iss.name}: {missing}")
    cmd = [f"/D{k}={v}" for k, v in defines.items()]
    if (st / "core" / "freedyn2.ico").is_file():
        cmd.append("/DHaveIcon=1")
    iscc = find_iscc(args.iscc)
    if info.get("dry_run") or args.dry_run:
        log("dry run: ISCC", " ".join(cmd), iss)
        log("dry run: inputs of", iss.name, "present")
        return None
    if not iscc:
        raise SystemExit("ISCC.exe (Inno Setup 6) not found: install it (choco install innosetup) or pass --iscc")
    run(iscc, "/Qp", *cmd, iss)
    setup = out / f"FreeDyn2-{info['version']}-setup.exe"
    if not setup.is_file():
        raise SystemExit(f"ISCC did not write {setup}")
    log(f"{setup} ({setup.stat().st_size / 1e6:.0f} MB, SHA256 {sha256(setup)})")
    return setup


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["stage", "compile", "all"])
    ap.add_argument("--wheel-dir", default="wheelhouse", help="directory with freedyn2-*-win_amd64.whl")
    ap.add_argument("--work", default=str(REPO / "build" / "installer"), help="work directory (downloads, stage)")
    ap.add_argument("--out", default=str(REPO / "dist"), help="directory for the installer")
    ap.add_argument("--iscc", help="path of ISCC.exe")
    ap.add_argument("--python-archive", help=f"local copy of {PYTHON_FILE} instead of the download")
    ap.add_argument("--dry-run", action="store_true", help="no Windows needed: see the description")
    args = ap.parse_args(argv)
    if sys.platform != "win32" and not args.dry_run:
        ap.error("the installer is built on Windows; elsewhere use --dry-run")
    if args.command in ("stage", "all"):
        stage(args)
    if args.command in ("compile", "all"):
        compile_installer(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
