"""Notes on a model: what the person tells the assistant from the viewer, and the answers.

  <model dir>/<model stem>.notes/notes.json    [{id, created, text, context, image, status, reply}]
  <model dir>/<model stem>.notes/note_<id>.png  the view when the note was written (optional)

A note carries its context: the time, the selected element, the plotted channels, the values
tuned in the viewer (overrides, not yet in the model file). The assistant reads them with the
MCP tool `notes`, changes the model and answers (`answer_note`); the viewer shows the answer.
The files sit next to the model, so an assistant without MCP (coding in the repository) finds them too.
"""

import base64
import json
import threading
import time
from pathlib import Path

_LOCK = threading.Lock()


def notes_dir(model_path):
    p = Path(model_path).resolve()
    return p.parent / f"{p.stem}.notes"


def _file(model_path):
    return notes_dir(model_path) / "notes.json"


def read_notes(model_path):
    f = _file(model_path)
    if not f.is_file():
        return []
    return json.loads(f.read_text(encoding="utf-8"))


def _write(model_path, notes):
    d = notes_dir(model_path)
    d.mkdir(exist_ok=True)
    tmp = d / "notes.json.tmp"
    tmp.write_text(json.dumps(notes, indent=1), encoding="utf-8")
    tmp.replace(d / "notes.json")


def add_note(model_path, text, context=None, image=None):
    """A new open note; image: PNG bytes or a data URL. Returns the note."""
    text = str(text).strip()
    if not text:
        raise ValueError("a note needs a text")
    with _LOCK:
        notes = read_notes(model_path)
        nid = 1 + max([n["id"] for n in notes] or [0])
        note = {"id": nid, "created": time.strftime("%Y-%m-%d %H:%M:%S"), "text": text, "context": context or {},
                "image": None, "status": "open", "reply": None}
        if image:
            if isinstance(image, str):
                image = base64.b64decode(image.split(",", 1)[-1])
            notes_dir(model_path).mkdir(exist_ok=True)
            (notes_dir(model_path) / f"note_{nid}.png").write_bytes(image)
            note["image"] = f"note_{nid}.png"
        notes.append(note)
        _write(model_path, notes)
    return note


def answer_note(model_path, nid, reply=None, done=True):
    """Answer a note (reply text) and/or close it (done) or open it again (done=False)."""
    with _LOCK:
        notes = read_notes(model_path)
        for n in notes:
            if n["id"] == int(nid):
                if reply is not None:
                    n["reply"] = str(reply)
                n["status"] = "done" if done else "open"
                _write(model_path, notes)
                return n
    raise ValueError(f"no note {nid}")


def delete_note(model_path, nid):
    with _LOCK:
        notes = read_notes(model_path)
        keep = [n for n in notes if n["id"] != int(nid)]
        if len(keep) == len(notes):
            raise ValueError(f"no note {nid}")
        (notes_dir(model_path) / f"note_{int(nid)}.png").unlink(missing_ok=True)
        _write(model_path, keep)


def stamp(model_path):
    """Changes when the notes change (for the viewer's polling)."""
    f = _file(model_path)
    return f.stat().st_mtime_ns if f.is_file() else 0


def note_text(n, model_path=None):
    """One note as text for the assistant."""
    c = n.get("context") or {}
    lines = [f"note {n['id']} ({n['status']}, {n['created']}): {n['text']}"]
    if "time" in c:
        lines.append(f"  at t = {c['time']:.4g} s")
    if c.get("selection"):
        lines.append(f"  selected: {c['selection']}")
    if c.get("channels"):
        lines.append("  plotted: " + ", ".join(c["channels"]))
    if c.get("cursors"):
        lines.append(f"  cursors A|B at t = {c['cursors'][0]:.4g} s, {c['cursors'][1]:.4g} s")
    if c.get("overrides"):
        lines.append("  values tuned in the viewer (not in the model file): "
                     + ", ".join(f"{k} = {v:g}" for k, v in c["overrides"].items()))
    if c.get("mode"):
        m = c["mode"]
        lines.append(f"  eigenmode shown: {m.get('frequency', 0):.4g} Hz, damping ratio {m.get('damping', 0):.3g}")
    if n.get("image") and model_path is not None:
        lines.append(f"  view: {notes_dir(model_path) / n['image']}")
    if n.get("reply"):
        lines.append(f"  answer: {n['reply']}")
    return "\n".join(lines)
