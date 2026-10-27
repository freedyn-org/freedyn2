"""Data files of FreeDyn 2.0: in an installed package next to the modules (fdgen/schema,
fdgen/examples, fdgen/bin: put there by the wheel build), in the repository at its top level."""

from pathlib import Path

PACKAGE = Path(__file__).resolve().parent
REPOSITORY = PACKAGE.parents[1]


def path(relative):
    """The file or directory `relative` (e.g. "schema/model.schema.json"), installed or in the
    repository; the installed one if neither exists."""
    for base in (PACKAGE, REPOSITORY):
        p = base / relative
        if p.exists():
            return p
    return PACKAGE / relative
