"""The winget manifests of a release, from the templates in this directory (for the first submission).

  python installer/winget/render.py VERSION SETUP_EXE [-o OUT]

SETUP_EXE: the released FreeDyn2-VERSION-setup.exe (its SHA256 goes into the manifest; download it
from the release, so that the hash is that of the published file). Writes
OUT/manifests/f/FreeDynOrg/FreeDyn2/VERSION/*.yaml (OUT: build/winget), the layout of
microsoft/winget-pkgs. Check with `winget validate --manifest DIR` and test with
`winget install --manifest DIR` on Windows. Later versions: wingetcreate in the release workflow.
"""

import argparse
import datetime
import hashlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
ID = "FreeDynOrg.FreeDyn2"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("version")
    ap.add_argument("setup_exe")
    ap.add_argument("-o", "--out", default=str(HERE.parent.parent / "build" / "winget"))
    ap.add_argument("--date", default=datetime.date.today().isoformat(), help="release date (YYYY-MM-DD)")
    a = ap.parse_args()
    values = {"VERSION": a.version, "SHA256": hashlib.sha256(Path(a.setup_exe).read_bytes()).hexdigest().upper(),
              "RELEASE_DATE": a.date}
    dest = Path(a.out) / "manifests" / ID[0].lower() / Path(*ID.split(".")) / a.version
    dest.mkdir(parents=True, exist_ok=True)
    for t in sorted(HERE.glob(f"{ID}*.yaml")):
        text = t.read_text(encoding="utf-8")
        for k, v in values.items():
            text = text.replace("{{" + k + "}}", v)
        text = "\n".join(line for line in text.splitlines() if not line.startswith("# Template")) + "\n"
        (dest / t.name).write_text(text, encoding="utf-8")
        print(dest / t.name)


if __name__ == "__main__":
    main()
