"""Accelerations and reactions at a state: the v2.0 of the dynamics work.

The equations are the standard index-1 form for constrained rigid bodies:

    M(q) q̈ = f(q, q̇) − Φ_q(q)ᵀ λ
    Φ_q(q) q̈ = γ(q, q̇),    γ = −q̇ᵀ ∇²Φ q̇

solved together as one symmetric saddle-point system. There is no integration here: given a
pose and a velocity, this returns the acceleration and the reaction in every constrained row —
which is what a figure of bush loads or of a prescribed-motion run needs, and what the time
stepping in v2.1 will call.

`M` is not block diagonal in general, because a body's centre of mass need not be its origin:
the translation and rotation of one body couple through the offset, so each body contributes a
dense 7×7 block assembled from two 3×7 Jacobians. Different bodies do not couple.

`γ` is differentiated numerically along `q̇` — two Jacobian evaluations, not twenty-one
lambdified Hessians — for the same reason the equilibrium solve differentiates its residual:
the exact expression is large and the difference is accurate to well below the tolerances that
matter.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config.model import Model
from ..errors import SolverError
from .compile import DRIVER, HARD, NORM, SPRING, Problem


@dataclass(frozen=True)
class Accelerations:
    qddot: np.ndarray
    #: Reaction in each row of the constrained system, keyed by row name.
    reactions: dict[str, float]
    #: The generalised force, before the constraints react.
    force: np.ndarray


def _omega_map(w: float, x: float, y: float, z: float) -> np.ndarray:
    """`W(q)` with `ω = W(q) q̇` for the world-frame angular velocity."""
    return 2.0 * np.array(
        [
            [-x, w, -z, y],
            [-y, z, w, -x],
            [-z, -y, x, w],
        ]
    )


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def _rotation(w: float, x: float, y: float, z: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    return np.array(
        [
            [w * w + xx - yy - zz, 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), w * w - xx + yy - zz, 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), w * w - xx - yy + zz],
        ]
    )


def _inertia(values: list[float]) -> np.ndarray:
    ixx, iyy, izz, ixy, ixz, iyz = values
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)


def _body_indices(problem: Problem, body: str) -> slice:
    start = problem._qstart[body]
    return slice(start, start + 7)


def mass_matrix(problem: Problem, q: np.ndarray, g: np.ndarray) -> np.ndarray:
    """The 7×7 mass block per moving body, assembled into the full matrix."""
    model: Model = problem.model
    dynamics = model.geometry.dynamics
    if dynamics is None:
        raise SolverError("this model has no `dynamics:` block, so it has no mass")
    matrix = np.zeros((problem.n, problem.n))
    for body in problem.moving:
        numbers = dynamics.bodies.get(body)
        if numbers is None:
            raise SolverError(f"body {body!r} has no mass, so the model has no dynamics")
        block = _body_indices(problem, body)
        w, x, y, z = q[block][3:7]
        rotation = _rotation(w, x, y, z)
        omega = _omega_map(w, x, y, z)
        if isinstance(numbers.com, str):
            offset = problem.resolved.points[numbers.com]
        else:
            offset = np.asarray(numbers.com, dtype=float)
        r = rotation @ offset
        jv = np.hstack([np.eye(3), -_skew(r) @ omega])
        jw = np.hstack([np.zeros((3, 3)), omega])
        matrix[block, block] = numbers.mass * (jv.T @ jv) + jw.T @ _inertia(numbers.inertia) @ jw
    return matrix


def forces(
    problem: Problem,
    q: np.ndarray,
    qdot: np.ndarray,
    g: np.ndarray,
    theta: np.ndarray,
    loads: list[tuple[str, str, np.ndarray]] | None = None,
    gravity: np.ndarray | None = None,
) -> np.ndarray:
    """Gravity, applied loads, and the springs and dampers' internal forces."""
    model = problem.model
    dynamics = model.geometry.dynamics
    if dynamics is None:
        raise SolverError("this model has no `dynamics:` block, so it has no forces")
    force = np.zeros(problem.n)

    weight = np.asarray(dynamics.gravity if gravity is None else gravity, dtype=float)
    for body in problem.moving:
        numbers = dynamics.bodies.get(body)
        if numbers is None:
            raise SolverError(f"body {body!r} has no mass, so the model has no dynamics")
        block = _body_indices(problem, body)
        w, x, y, z = q[block][3:7]
        rotation = _rotation(w, x, y, z)
        omega = _omega_map(w, x, y, z)
        offset = problem.resolved.points[numbers.com] if isinstance(numbers.com, str) else np.asarray(numbers.com)
        r = rotation @ np.asarray(offset, dtype=float)
        jv = np.hstack([np.eye(3), -_skew(r) @ omega])
        force[block] += jv.T @ (numbers.mass * weight)

    for body, point, applied in loads or []:
        point_jac, _ = problem.point_derivatives(body, point)
        force += np.asarray(point_jac(*q, *g), dtype=float).reshape(3, problem.n).T @ np.asarray(applied)
        point_jac = None

    values = problem.evaluate(q, g, theta)
    jac = problem.jacobian(q, g, theta)
    for index, row in enumerate(problem.rows):
        if row.group != SPRING:
            continue
        joint, component = row.name.split(".", 1)
        compliance = problem.compliance(g).get((joint, component), 0.0)
        gradient = jac[index]
        tension = values[index] / compliance if compliance else 0.0
        damping = _damping(model, joint, component)
        force -= (tension + damping * float(gradient @ qdot)) * gradient
    return force


def _damping(model: Model, joint: str, component: str) -> float:
    numbers = model.geometry.joints.get(joint)
    if numbers is None:
        return 0.0
    return float(numbers.damping.get(component, 0.0))


def acceleration_terms(problem: Problem, q: np.ndarray, qdot: np.ndarray, g: np.ndarray, theta: np.ndarray):
    """`Φ_q` and `γ`, the two pieces of the acceleration-level constraint."""
    rows = [i for i, row in enumerate(problem.rows) if row.group != SPRING]
    jac = problem.jacobian(q, g, theta)[rows]
    speed = float(np.linalg.norm(qdot))
    if speed == 0.0:
        return jac, np.zeros(len(rows))
    step = 1e-6 / speed
    plus = problem.jacobian(q + step * qdot, g, theta)[rows]
    minus = problem.jacobian(q - step * qdot, g, theta)[rows]
    gamma = -((plus - minus) / (2.0 * step)) @ qdot
    return jac, gamma


def accelerations(
    problem: Problem,
    q: np.ndarray,
    qdot: np.ndarray,
    *,
    loads: list[tuple[str, str, np.ndarray]] | None = None,
    gravity: np.ndarray | None = None,
    theta: np.ndarray | None = None,
) -> Accelerations:
    """Solve the saddle-point system for the acceleration and every reaction."""
    g = problem.numbers()
    theta = problem.theta0() if theta is None else theta
    rows = [i for i, row in enumerate(problem.rows) if row.group != SPRING]
    mass = mass_matrix(problem, q, g)
    force = forces(problem, q, qdot, g, theta, loads, gravity)
    constraint, gamma = acceleration_terms(problem, q, qdot, g, theta)

    size = problem.n + len(rows)
    system = np.zeros((size, size))
    system[: problem.n, : problem.n] = mass
    system[: problem.n, problem.n :] = constraint.T
    system[problem.n :, : problem.n] = constraint
    rhs = np.concatenate([force, gamma])
    try:
        solution = np.linalg.solve(system, rhs)
    except np.linalg.LinAlgError:
        solution, *_ = np.linalg.lstsq(system, rhs, rcond=None)
    multipliers = solution[problem.n :]
    reactions = {problem.rows[index].name: float(value) for index, value in zip(rows, multipliers)}
    return Accelerations(qddot=solution[: problem.n], reactions=reactions, force=force)
