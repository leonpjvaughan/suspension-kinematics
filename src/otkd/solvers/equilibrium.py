"""Compliant components: the equilibrium of a loaded, bushed mechanism.

With a `spring` component the joint's coordinate is free but resisted, so its equation is not
`g = 0`; the system is closed by stationarity of the potential in the remaining free
directions. The residual is

    r(q) = [ g_hard(q) ; Z(q)ᵀ ∇Π(q) / f_ref ]

where `Z` is an orthonormal basis of the null space of the hard-plus-driver Jacobian and
`Π = ½ Σ g_i²/c_i − Σ F·x` is the spring energy minus the work of the loads. Every root of
`r` satisfies all the hard constraints and force balance along each compliance, exactly —
this is the algebraic equilibrium of the book's §4, not a stiff-constraint approximation.

The constraint Jacobian inside `r` is symbolic and exact. The derivative of the *projected
potential* is not: `∂/∂q (Zᵀ∇Π)` contains `(∂Z/∂q)ᵀ∇Π`, which is zero at the solution but
not on the way to it, and the null-space basis genuinely changes with `q`. Supplying the
truncated analytic block makes the trust region stall, so the solve differentiates the
residual numerically. That is a deliberate, measured choice: eight residual evaluations per
frame against a hundred and ninety-seven.
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
    n = problem.n
    free = n - len(rows)
    if free < 1:
        raise SolverError("equilibrium needs at least one compliant degree of freedom")
    force_scale = max(1.0, float(sum(np.linalg.norm(force) for _, _, force in loads)))
    history: list[tuple[np.ndarray, np.ndarray]] = []

    def null_basis(q: np.ndarray) -> np.ndarray:
        jac = system.jacobian(q, rows)
        _, _, vt = np.linalg.svd(jac, full_matrices=True)
        basis = vt[len(rows) :].T
        # Singular vectors are defined up to sign, and an arbitrary anchor can flip between
        # two nearby evaluations — a discontinuity no Jacobian survives. Anchor against the
        # compliant rows' own gradients, which point along the free directions: the dot
        # product is O(1) and smooth, so the sign cannot flicker.
        springs = system.springs
        if springs:
            anchor = system.jacobian(q, springs)
            for column in range(basis.shape[1]):
                reference = anchor[min(column, len(springs) - 1)]
                if float(basis[:, column] @ reference) < 0:
                    basis[:, column] *= -1.0
        return basis

    def gradient(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        grad = problem.gradient(q, system.g, system.theta)
        for body, point, force in loads:
            point_jac, _ = problem.point_derivatives(body, point)
            jacobian = np.asarray(point_jac(*q, *system.g), dtype=float).reshape(3, n)
            grad = grad - jacobian.T @ force
        return grad, null_basis(q)

    def residual(q: np.ndarray) -> np.ndarray:
        values = system.residual(q, rows)
        grad, basis = gradient(q)
        return np.concatenate([values, basis.T @ grad / force_scale])

    def record(x: np.ndarray, *args) -> None:
        """Accepted iterates only, so `iterations[]` means what it means elsewhere."""
        history.append((np.asarray(x).copy(), residual(x)))

    result = scipy_least_squares(
        residual,
        np.asarray(q0, dtype=float),
        method="trf",
        callback=record,
        xtol=tol,
        ftol=tol,
        gtol=tol,
        max_nfev=200,
    )
    error = float(np.max(np.abs(result.fun)))
    if error > 1e-8:
        raise SolverError(f"equilibrium did not converge: {result.message}", residual=error)
    return result.x, int(result.nfev), history
