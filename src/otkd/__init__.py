"""otkd — open textbook kinematics and dynamics.

Config files in, mathematical problems out. The public surface is five names:
`load`, `check`, `compile`, `solve`, `emit`. See docs/entry-points.md.
"""

from __future__ import annotations

from .config.model import Model
from .problem.compile import Problem

__all__ = ["Model", "Problem", "load", "check", "compile", "solve", "emit", "config", "problem"]

# Written by hatch-vcs at build time from the git tag. The fallback keeps a plain source tree
# importable; it is never what a released build reports.
try:
    from ._version import __version__  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - only without an install
    __version__ = "0.0.0"


def load(path, *, study=None) -> Model:
    """Read a model directory (or a topology path) into a `Model`."""
    return Model.load(path, study=study)


def compile(model: Model) -> Problem:  # noqa: A001 - the public name is `compile`
    """Derive the symbolic problem for a model's topology."""
    return Problem(model)


def check(model: Model):
    from .report import check as _check

    return _check(model)


def solve(model: Model):
    from .solvers.sweep import sweep as _sweep

    return _sweep(model)


def emit(model: Model, path):
    from .emitters.js import emit_js

    return emit_js(model, path)
