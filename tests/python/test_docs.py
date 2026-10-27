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
