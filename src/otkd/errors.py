"""Errors a user of the config layer will actually see.

A configuration error is a *list* of faults, not the first one found, because a draft with
four typos should cost one run rather than four. A solver failure is a different type and a
different exit code: "your model is malformed" and "your model has no solution here" are
different problems.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Fault:
    file: str
    key: str
    message: str

    def __str__(self) -> str:
        where = f"{self.file}: {self.key}" if self.key else self.file
        return f"{where}: {self.message}"


class ConfigError(Exception):
    """One or more faults that make the model unusable."""

    def __init__(self, faults: list[Fault] | tuple[Fault, ...]):
        self.faults = list(faults)
        super().__init__("\n".join(str(f) for f in self.faults))

    def __str__(self) -> str:
        return "\n".join(str(f) for f in self.faults)


class SolverError(RuntimeError):
    """The model is well formed but the solve did not converge."""

    def __init__(self, message: str, *, frame: int | None = None, residual: float | None = None):
        self.frame = frame
        self.residual = residual
        super().__init__(message)
