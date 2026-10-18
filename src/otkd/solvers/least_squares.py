"""Least squares for a redundant system."""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares as scipy_least_squares

from ..errors import SolverError
from .system import System


def least_squares(
    system: System,
    q0: np.ndarray,
    *,
    rows: list[int] | None = None,
    tol: float = 1e-12,
    max_steps: int = 200,
) -> tuple[np.ndarray, int, list[tuple[np.ndarray, np.ndarray]], float]:
    """Solve the over-determined system. Returns the solution, evaluations, history, cost."""
    history: list[tuple[np.ndarray, np.ndarray]] = []

    def residual(x: np.ndarray) -> np.ndarray:
        return system.residual(x, rows)

    def record(x: np.ndarray) -> None:
        history.append((np.asarray(x).copy(), residual(x).copy()))

    x0 = np.asarray(q0, dtype=float)
    record(x0)
    result = scipy_least_squares(
        residual,
        x0,
        method="trf",
        ftol=tol,
        xtol=tol,
        gtol=tol,
        max_nfev=max_steps * (system.problem.n + 1),
        callback=record,
    )
    record(result.x)
    if float(np.max(np.abs(result.fun))) > 1e-8:
        raise SolverError(
            f"least squares did not converge: {result.message}",
            residual=float(np.max(np.abs(result.fun))),
        )
    return result.x, int(result.nfev), history, float(result.cost)
