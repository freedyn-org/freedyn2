"""Links that carry a model (fdgen.share, runtime/wasm/demo/link.mjs): the same formats in Python
and in the browser."""

import json
import shutil
import subprocess
from pathlib import Path

import pytest

from fdgen import cli, share

ROOT = Path(__file__).resolve().parents[2]
PENDULUM = ROOT / "examples/pendulum.yaml"
TRICKY = "model: x  # ä, ±, 😀\nparameters: {k: =a + b & c}\n" + "bodies: {}\n" * 200


def test_roundtrip():
    for text in (PENDULUM.read_text(encoding="utf-8"), TRICKY, ""):
        value = share.encode(text)
        assert set(value) <= set("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_")
        assert share.decode(value) == text
    url = share.link(TRICKY, "https://example.org/demo/?pyodide=x#old")
    assert url.startswith("https://example.org/demo/?pyodide=x#model=")
    assert len(url) < len(TRICKY) / 4  # compressed


def test_notes():
    assert share.notes("model: m\n", "x") == []
    assert "not in the link" in share.notes("shapes: [{mesh: parts/arm.STL}]\n", "x")[0]
    assert "characters" in share.notes("model: m\n", "x" * (share.LONG + 1))[0]


def test_cli(tmp_path, capsys):
    assert cli.main(["link", str(PENDULUM), "--base", "http://localhost:8000/"]) == 0
    url = capsys.readouterr().out.strip()
    assert url.startswith("http://localhost:8000/#model=")
    assert share.decode(url.split("#model=")[1]) == PENDULUM.read_text(encoding="utf-8")


def test_mcp_tool():
    from fdgen.mcp_server import Session

    s = Session()
    try:
        out = s.link(str(PENDULUM)).splitlines()
        assert out[0].startswith(share.DEMO_URL + "#model=")
        assert s.link("/no/such/file.yaml").startswith("NO LINK:")
    finally:
        s.close()


JS = r"""
import { encodeModel, decodeModel, linkFor, modelFrom } from "%s";
const input = JSON.parse(process.argv[2]);
const out = {
  fromPython: await decodeModel(input.python),
  fromJs: await encodeModel(input.text),
  link: await linkFor(input.text, "https://example.org/demo/#old"),
  hashModel: await modelFrom("https://example.org/demo/#model=" + input.python),
  // by hand: '+' and '&' unencoded must survive, %%0A is a newline
  hashYaml: await modelFrom("https://example.org/demo/#yaml=model: x%%0Ak: =a + b & c"),
  src: await modelFrom("https://example.org/demo/?src=m.yaml"),
  none: await modelFrom("https://example.org/demo/"),
};
try { await modelFrom("https://example.org/demo/#model=" + input.python.slice(0, -9)); out.cut = "read"; }
catch (e) { out.cut = e.message; }
console.log(JSON.stringify(out));
"""


@pytest.mark.skipif(shutil.which("node") is None, reason="needs Node.js (CompressionStream deflate-raw: 21.2 and newer)")
def test_javascript_reads_and_writes_the_same(tmp_path):
    script = tmp_path / "t.mjs"
    script.write_text(JS % (ROOT / "runtime/wasm/demo/link.mjs").as_uri(), encoding="utf-8")
    arg = json.dumps({"text": TRICKY, "python": share.encode(TRICKY)})
    r = subprocess.run(["node", str(script), arg], capture_output=True, text=True, encoding="utf-8")
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout)
    assert out["fromPython"] == TRICKY
    assert share.decode(out["fromJs"]) == TRICKY
    assert out["link"].startswith("https://example.org/demo/#model=")
    assert out["hashModel"] == {"yaml": TRICKY}
    assert out["hashYaml"] == {"yaml": "model: x\nk: =a + b & c"}
    assert out["src"] == {"src": "https://example.org/demo/m.yaml"}
    assert out["none"] is None
    assert "cut off" in out["cut"]
