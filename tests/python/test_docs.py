"""The user documentation (docs/, mkdocs.yml): every complete model in it is valid, and the
key tables of the reference are generated from the schema."""

import importlib.util
import re
from pathlib import Path

import pytest
import yaml

from fdgen import model

ROOT = Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs"
BLOCK = re.compile(r"^```yaml\n(.*?)^```", re.M | re.S)


def models():
    for page in sorted(DOCS.rglob("*.md")):
        for i, block in enumerate(BLOCK.findall(page.read_text(encoding="utf-8"))):
            if block.startswith("model:"):
                yield pytest.param(block, id=f"{page.relative_to(DOCS)}#{i}")


@pytest.mark.parametrize("text", list(models()))
def test_models_in_the_docs_are_valid(text):
    model.validate(yaml.safe_load(text), base_dir=ROOT / "examples")


def test_there_are_models_in_the_docs():
    assert len(list(models())) >= 3


def hooks():
    spec = importlib.util.spec_from_file_location("docs_hooks", ROOT / "tools" / "docs_hooks.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_key_tables_come_from_the_schema():
    schema = model.load_schema()
    markers = [m for page in DOCS.rglob("*.md") for m in hooks().MARKER.findall(page.read_text(encoding="utf-8"))]
    assert markers
    for name in markers:
        table = hooks().table(name, schema)
        assert table.startswith("| key |") and table.count("\n") > 2, name


def test_the_docs_theme_takes_its_colors_from_the_brand():
    """brand/tokens.css is the one source of the colors: the site serves it from brand/ (the hook),
    the docs' own CSS has no color values, and tokens.css agrees with tokens.json."""
    import json

    config = (ROOT / "mkdocs.yml").read_text(encoding="utf-8")
    served = [f"brand/{name}" for name in hooks().BRAND_FILES]
    assert "brand/tokens.css" in served
    for name in served:
        assert (ROOT / name).is_file(), name
    for ref in re.findall(r"\bbrand/[\w./-]+", config + (DOCS / "index.md").read_text(encoding="utf-8")):
        assert ref.split("#")[0] in served, ref
    for css in (DOCS / "stylesheets").glob("*.css"):
        text = re.sub(r"/\*.*?\*/", "", css.read_text(encoding="utf-8"), flags=re.S)
        assert not re.search(r"#[0-9a-fA-F]{3,8}\b|\b(rgba?|hsla?)\(", text), css.name
    tokens = json.loads((ROOT / "brand" / "tokens.json").read_text(encoding="utf-8"))
    css = (ROOT / "brand" / "tokens.css").read_text(encoding="utf-8")
    for name, value in tokens["color"].items():
        assert re.search(rf"--fd-{name}:\s*{value};", css, re.I), name
