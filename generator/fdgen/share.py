"""Links to the browser demo (runtime/wasm/demo) that carry a model: open the link, the model
is simulated in the browser tab, nothing is installed and nothing is sent to a server.

    #model=<base64url of the raw-deflated UTF-8 YAML>   compact; link() writes it
    #yaml=<the YAML, percent-encoded>                    readable; easy to write by hand
    ?src=<URL of a YAML file or a zip with model and meshes>

The part after '#' never leaves the browser (it is not sent with the request for the page).
runtime/wasm/demo/link.mjs reads the same formats.
"""

import base64
import re
import zlib

DEMO_URL = "https://freedyn-org.github.io/freedyn2/demo/"
LONG = 8000  # characters; longer links may be cut off by chat programs and mail clients


def encode(yaml_text):
    """The #model= value of a YAML text: raw deflate (no zlib header), base64url without padding."""
    c = zlib.compressobj(9, zlib.DEFLATED, -15)
    data = c.compress(yaml_text.encode("utf-8")) + c.flush()
    return base64.urlsafe_b64encode(data).decode("ascii").rstrip("=")


def decode(value):
    """The YAML text of a #model= value."""
    value = value.strip()
    data = base64.urlsafe_b64decode(value + "=" * (-len(value) % 4))
    return zlib.decompress(data, -15).decode("utf-8")


def link(yaml_text, base=DEMO_URL):
    """The demo link that carries this model."""
    return f"{base.split('#')[0]}#model={encode(yaml_text)}"


def notes(yaml_text, url):
    """What the receiver of the link should know: files the model reads, the length."""
    out = []
    if re.search(r"\.(stl|obj|csv|txt|dat|crg)\b", yaml_text, re.I):
        out.append("the model names files (meshes, data): they are not in the link; the page reports "
                   "them missing (a zip with model and files: the download button of the page)")
    if len(url) > LONG:
        out.append(f"{len(url)} characters: chat programs and mail clients may cut the link off")
    return out
