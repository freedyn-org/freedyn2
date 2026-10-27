"""Block structure of .fds files and typed access to block values."""

from ..errors import ModelError

IGNORED_BLOCKS = {"Header", "Globals"}


def parse_blocks(text):
    """Return a list of (type, subtype, {key: value string}) in file order."""
    blocks, cur = [], None
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("%"):
            continue
        if cur is None:
            parts = line.split(None, 1)
            cur = (parts[0], parts[1].strip() if len(parts) > 1 else "", {})
            continue
        if line == "End" + cur[0]:
            blocks.append(cur)
            cur = None
            continue
        if "=" in line:
            key, value = line.split("=", 1)
            cur[2][key.strip()] = value.strip()
    if cur is not None:
        raise ModelError(f"block {cur[0]} {cur[1]} is not closed")
    return blocks


def numbers(block, key, n=None, default=None):
    if key not in block[2]:
        if default is None:
            raise ModelError(f"{block[0]} {block[1]}: missing key {key}")
        return default
    vals = [float(v) for v in block[2][key].replace(",", " ").split()]
    if n is not None and len(vals) != n:
        raise ModelError(f"{block[0]} {block[1]}: {key} needs {n} values")
    return vals


def flag(block, key, default):
    v = block[2].get(key)
    if v is None:
        return default
    v = v.strip().lower()
    if v in ("yes", "1", "true"):
        return True
    if v in ("no", "0", "false"):
        return False
    raise ModelError(f"{block[0]} {block[1]}: {key} must be yes or no")


def ident(prefix, label):
    return prefix + "".join(c if c.isalnum() or c == "_" else "_" for c in label.lower())
