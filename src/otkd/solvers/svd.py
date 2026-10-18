"""SVD diagnostics: which constraints are redundant, and in what combination."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .system import System, left_null


@dataclass(frozen=True)
class Diagnosis:
    singular_values: np.ndarray
    rank: int
    redundancy: int
    #: Row names that participate in a dependence, and the basis of the left null space.
    dependent_rows: tuple[str, ...]
    null_space: np.ndarray
    condition: float


def diagnose(system: System, q: np.ndarray, *, rows: list[int] | None = None, rtol: float = 1e-9) -> Diagnosis:
    rows = system.constrained if rows is None else rows
    jac = system.jacobian(q, rows)
    rank, null, s = left_null(jac, rtol=1e-10)
    names = [system.problem.rows[i].name for i in rows]
    if null.shape[1]:
        weight = np.max(np.abs(null), axis=1)
        dependent = tuple(name for name, w in zip(names, weight) if w > rtol)
    else:
        dependent = ()
    condition = float(s[0] / s[rank - 1]) if rank else float("inf")
    return Diagnosis(
        singular_values=s,
        rank=rank,
        redundancy=len(rows) - rank,
        dependent_rows=dependent,
        null_space=null,
        condition=condition,
    )
