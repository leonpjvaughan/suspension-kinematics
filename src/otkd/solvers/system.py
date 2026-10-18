"""The numeric system for one frame: which rows, at which geometry and drive values."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..errors import SolverError
from ..problem.compile import DRIVER, HARD, NORM, SPRING, Problem


@dataclass
class System:
    problem: Problem
    g: np.ndarray
    theta: np.ndarray
    drop: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        unknown = set(self.drop) - set(self.problem.row_index)
        if unknown:
            raise SolverError(f"drop names rows that do not exist: {sorted(unknown)}")
        dropped = {self.problem.row_index[name] for name in self.drop}
        for i in dropped:
            if self.problem.rows[i].group not in (HARD, NORM, DRIVER):
                raise SolverError(
                    f"cannot drop {self.problem.rows[i].name!r}: it is a {self.problem.rows[i].group} row"
                )
        self.rows = tuple(i for i in range(len(self.problem.rows)) if i not in dropped)

    @property
    def hard(self) -> list[int]:
        return [i for i in self.rows if self.problem.rows[i].group in (HARD, NORM)]

    @property
    def drivers(self) -> list[int]:
        return [i for i in self.rows if self.problem.rows[i].group == DRIVER]

    @property
    def _zero_compliance(self) -> set[str]:
        return {name for name, value in self.problem.compliance(self.g).items() if value == 0.0}

    @property
    def springs(self) -> list[int]:
        """Compliant rows that actually deflect. A zero compliance is a rigid row."""
        return [
            i
            for i in self.rows
            if self.problem.rows[i].group == SPRING
            and tuple(self.problem.rows[i].name.split(".", 1)) not in self._zero_compliance
        ]

    @property
    def constrained(self) -> list[int]:
        """Rows that are equalities: hard rows, plus springs whose compliance is zero."""
        springs = set(self.springs)
        return [i for i in self.rows if i not in springs]

    # -- evaluation --------------------------------------------------------------------

    def values(self, q: np.ndarray) -> np.ndarray:
        return self.problem.evaluate(q, self.g, self.theta)

    def derivatives(self, q: np.ndarray) -> np.ndarray:
        return self.problem.jacobian(q, self.g, self.theta)

    def residual(self, q: np.ndarray, rows: list[int] | None = None) -> np.ndarray:
        rows = self.constrained if rows is None else rows
        return self.values(q)[rows]

    def jacobian(self, q: np.ndarray, rows: list[int] | None = None) -> np.ndarray:
        rows = self.constrained if rows is None else rows
        return self.derivatives(q)[rows]

    # -- diagnostics -------------------------------------------------------------------

    def classify(self) -> str:
        """`square`, `redundant`, `under`, or `equilibrium`."""
        if self.springs:
            return "equilibrium"
        rank, _, _ = rank_and_null(self.jacobian(self.problem.q0()), rtol=1e-10)
        n = self.problem.n
        if rank < n:
            return "under"
        return "square" if len(self.constrained) == rank else "redundant"


def rank_and_null(matrix: np.ndarray, rtol: float = 1e-10) -> tuple[int, np.ndarray, np.ndarray]:
    """Rank, an orthonormal basis of the right null space, and all singular values."""
    _, s, vt = np.linalg.svd(matrix, full_matrices=True)
    if s.size == 0:
        return 0, np.eye(matrix.shape[1]), s
    rank = int((s > rtol * s[0]).sum())
    return rank, vt[rank:].T, s


def left_null(matrix: np.ndarray, rtol: float = 1e-10) -> tuple[int, np.ndarray, np.ndarray]:
    """Rank, a basis of the left null space (over rows), and all singular values."""
    u, s, _ = np.linalg.svd(matrix, full_matrices=True)
    if s.size == 0:
        return 0, np.zeros((matrix.shape[0], 0)), s
    rank = int((s > rtol * s[0]).sum())
    return rank, u[:, rank:], s
