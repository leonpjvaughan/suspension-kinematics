"""Compliant components: the equilibrium of a loaded, bushed mechanism.

With a `spring` component a joint's coordinate is free but resisted, so its equation is not
`g = 0`. The equilibrium is the saddle point of

    Π(q) = ½ Σ gᵢ(q)²/cᵢ − Σ F·x(q)     subject to     Φ(q) = 0

and it is solved as the KKT system it is, in `(q, λ)`:

    [ ∇Π(q) − Φ_q(q)ᵀ λ ]   = 0
    [ Φ(q)              ]

which is square, smooth, and needs no null-space basis. `λ` is also the number a suspension
engineer wants: the reaction in every constrained row, drivers included.

**Why not the projected form.** The first implementation projected the potential onto a null-space
basis of `Φ_q` and solved the smaller system `Z(q)ᵀ∇Π = 0`. It was right for one compliant
component and wrong for three at one frame joint: `Z(q)` comes from an SVD, and that basis rotates
— and its columns can reorder — as `q` moves, so the numerically differentiated residual is noise
and the trust region stops on `xtol` with the force balance still out. The tell was that the
residual did not move when the load changed by three orders of magnitude, which no force-balance
residual can do. [`decisions.md`](../docs/decisions.md) round 23 has the measurements.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares as scipy_least_squares

from ..errors import SolverError
from .system import System

LoadCase = list[tuple[str, str, np.ndarray]]


def equilibrium(
    system: System,
    q0: np.ndarray,
    loads: LoadCase,
    *,
    tol: float = 1e-12,
) -> tuple[np.ndarray, int, list[tuple[np.ndarray, np.ndarray]]]:
    problem = system.problem
    rows = system.constrained
    springs = system.springs
    n = problem.n
    free = n - len(rows)
    if free < 1:
        raise SolverError("equilibrium needs at least one compliant degree of freedom")
    force_scale = max(1.0, float(sum(np.linalg.norm(force) for _, _, force in loads)))
    history: list[tuple[np.ndarray, np.ndarray]] = []

    def residual(x: np.ndarray) -> np.ndarray:
        q, lam = x[:n], x[n:]
        grad = problem.gradient(q, system.g, system.theta, springs)
        for body, point, force in loads:
            point_jac, _ = problem.point_derivatives(body, point)
            jacobian = np.asarray(point_jac(*q, *system.g), dtype=float).reshape(3, n)
            grad = grad - jacobian.T @ force
        stationarity = (grad - system.jacobian(q, rows).T @ lam) / force_scale
        return np.concatenate([stationarity, system.residual(q, rows)])

    def record(x: np.ndarray, *args) -> None:
        """Accepted iterates only, so `iterations[]` means what it means elsewhere."""
        history.append((np.asarray(x[:n]).copy(), residual(x)))

    start = np.concatenate([np.asarray(q0, dtype=float), np.zeros(len(rows))])
    # `max_nfev` is generous because a joint at 100 MN/m needs a few hundred evaluations on the
    # frame where the load first swings it; a well-behaved frame uses five.
    result = scipy_least_squares(
        residual,
        start,
        method="trf",
        x_scale="jac",
        callback=record,
        xtol=tol,
        ftol=tol,
        gtol=tol,
        max_nfev=1000,
    )
    # The two blocks are measured separately, because they are not the same unit: a constraint
    # left at 1e-8 is a metre of violation, which is a different claim from a force balance left
    # at 1e-8 of the load. The projected formulation could report one number because it had one
    # kind of row; this one cannot.
    constraints = float(np.max(np.abs(result.fun[n:]))) if len(rows) else 0.0
    stationarity = float(np.max(np.abs(result.fun[:n])))
    if constraints > 1e-12 or stationarity > 1e-9:
        raise SolverError(
            f"equilibrium did not converge: {result.message}",
            residual=max(constraints, stationarity),
        )
    return result.x[:n], int(result.nfev), history
