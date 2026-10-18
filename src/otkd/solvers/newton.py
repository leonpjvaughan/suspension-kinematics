"""Damped Newton for a square system."""

from __future__ import annotations

import numpy as np

from ..errors import SolverError
from .system import System


def newton(
    system: System,
    q0: np.ndarray,
    *,
    rows: list[int] | None = None,
    tol: float = 1e-12,
    max_steps: int = 60,
) -> tuple[np.ndarray, int, list[tuple[np.ndarray, np.ndarray]]]:
    """Solve `rows(q) = 0` from `q0`. Returns the solution, step count and residual history."""
    q = np.array(q0, dtype=float)
    history: list[tuple[np.ndarray, np.ndarray]] = []
    for step in range(max_steps):
        r = system.residual(q, rows)
        history.append((q.copy(), r.copy()))
        error = float(np.max(np.abs(r))) if r.size else 0.0
        if error < tol:
            return q, step, history
        jac = system.jacobian(q, rows)
        try:
            delta = np.linalg.solve(jac, -r)
        except np.linalg.LinAlgError:
            delta, *_ = np.linalg.lstsq(jac, -r, rcond=None)
        if not np.all(np.isfinite(delta)):
            raise SolverError("Newton step is not finite", residual=error)
        base = float(np.max(np.abs(r)))
        alpha = 1.0
        for _ in range(40):
            trial = q + alpha * delta
            if float(np.max(np.abs(system.residual(trial, rows)))) < base or alpha < 1e-8:
                break
            alpha *= 0.5
        q = q + alpha * delta

    r = system.residual(q, rows)
    index = (rows or system.constrained)[int(np.argmax(np.abs(r)))]
    raise SolverError(
        f"Newton did not converge in {max_steps} steps: |r|inf = {np.max(np.abs(r)):.3e}, "
        f"worst row {system.problem.rows[index].name}",
        residual=float(np.max(np.abs(r))),
    )
