"""Errors of FreeDyn 2 (also raised by the Python API, freedyn2)."""


class FreeDynError(Exception):
    """Base of the errors below."""


class ModelError(FreeDynError, ValueError):
    """Invalid or unsupported model input."""


class SimulationError(FreeDynError, RuntimeError):
    """The runtime freedyn2_run failed (message: its output)."""


class RuntimeNotFoundError(FreeDynError, FileNotFoundError):
    """The runtime freedyn2_run was not found."""
