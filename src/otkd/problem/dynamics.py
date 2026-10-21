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


def _point_jacobian(offset: np.ndarray, w: float, x: float, y: float, z: float) -> np.ndarray:
    """`∂(p + R(q) o)/∂q` for the seven pose parameters — the *full* derivative.

    The skew-only form (`[I, −[r]×W]`) is the derivative along the unit-quaternion tangent space;
    it is what most codes use because they never leave the manifold. Here the pose is a plain
    seven-vector, and a force built from the tangent-only form disagrees with the potential's own
    gradient in the radial direction. On a rotating body that direction is not orthogonal to the
    motion, so the disagreement leaks energy — which is exactly what the quarter-car test found.
    """
    v = np.array([x, y, z], dtype=float)
    o = np.asarray(offset, dtype=float)
    jac = np.zeros((3, 7))
    jac[:, :3] = np.eye(3)
    jac[:, 3] = 2.0 * (w * o + np.cross(v, o))
    for j in range(3):
        axis = np.zeros(3)
        axis[j] = 1.0
        jac[:, 4 + j] = 2.0 * (-v[j] * o + axis * (v @ o) + v * o[j] + w * np.cross(axis, o))
    return jac


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
        jv = _point_jacobian(np.asarray(offset, dtype=float), w, x, y, z)
        jw = np.hstack([np.zeros((3, 3)), omega])
        matrix[block, block] = numbers.mass * (jv.T @ jv) + jw.T @ _inertia(numbers.inertia) @ jw
    return matrix


def velocity_products(problem: Problem, q: np.ndarray, qdot: np.ndarray, g: np.ndarray) -> np.ndarray:
    """`C(q, q̇) = Ṁ q̇ − ∂(½ q̇ᵀM q̇)/∂q`, the term the Euler–Lagrange equations add.

    `M q̈ = f` is only right when `M` is constant. Here it is not: a body's mass matrix depends
    on its orientation, and the quaternion parametrisation makes even a pure rotation change `M`.
    Omitting `C` leaves the total energy varying along the motion — which is exactly what the
    quarter-car test caught, at a magnitude that did not shrink with the step size.

    **Per body, not per model.** `M` is block diagonal, so a body's kinetic energy depends only on
    its own seven coordinates. Differentiating the whole model's energy forty times to find the
    gradient ignores that; differentiating each body's sixteen times is the same number for less
    work — and the block diagonal is the reason, so the code says so.
    """
    dynamics = _dynamics_block(problem)
    gradient = np.zeros(problem.n)
    speed = float(np.linalg.norm(qdot))
    step = 1e-7 / speed if speed else 0.0

    for body in problem.moving:
        numbers = dynamics.bodies.get(body)
        if numbers is None:
            raise SolverError(f"body {body!r} has no mass, so the model has no dynamics")
        block = _body_indices(problem, body)
        start = block.start
        assert start is not None
        offset = _centre_of_mass(problem, numbers)
        inertia = _inertia(numbers.inertia)
        mass = float(numbers.mass)
        velocity = qdot[block]

        for local in range(7):
            j = start + local
            h = 1e-7 * (1.0 + abs(q[j]))
            plus, minus = q.copy(), q.copy()
            plus[j] += h
            minus[j] -= h
            gradient[j] = (
                _body_kinetic(offset, inertia, mass, plus[block], velocity)
                - _body_kinetic(offset, inertia, mass, minus[block], velocity)
            ) / (2.0 * h)

        if step:
            forward = _body_block(offset, inertia, mass, (q + step * qdot)[block]) @ velocity
            backward = _body_block(offset, inertia, mass, (q - step * qdot)[block]) @ velocity
            gradient[block] -= (forward - backward) / (2.0 * step)

    return -gradient


def _dynamics_block(problem: Problem):
    dynamics = problem.model.geometry.dynamics
    if dynamics is None:
        raise SolverError("this model has no `dynamics:` block, so it has no mass")
    return dynamics


def _centre_of_mass(problem: Problem, numbers) -> np.ndarray:
    if isinstance(numbers.com, str):
        return np.asarray(problem.resolved.points[numbers.com], dtype=float)
    return np.asarray(numbers.com, dtype=float)


def _body_block(offset: np.ndarray, inertia: np.ndarray, mass: float, pose: np.ndarray) -> np.ndarray:
    """One body's 7×7 block of `M`: translation through its centre of mass, rotation through `I`."""
    w, x, y, z = pose[3:7]
    jv = _point_jacobian(offset, w, x, y, z)
    jw = np.hstack([np.zeros((3, 3)), _omega_map(w, x, y, z)])
    return mass * (jv.T @ jv) + jw.T @ inertia @ jw


def _body_kinetic(
    offset: np.ndarray, inertia: np.ndarray, mass: float, pose: np.ndarray, velocity: np.ndarray
) -> float:
    return 0.5 * float(velocity @ _body_block(offset, inertia, mass, pose) @ velocity)


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
        jv = _point_jacobian(np.asarray(offset, dtype=float), w, x, y, z)
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
    force = force - velocity_products(problem, q, qdot, g)
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


@dataclass(frozen=True)
class Trajectory:
    t: np.ndarray
    q: np.ndarray
    qdot: np.ndarray
    energy: np.ndarray
    constraint: np.ndarray


def energy(
    problem: Problem,
    q: np.ndarray,
    qdot: np.ndarray,
    gravity: np.ndarray | None = None,
    loads: list[tuple[str, str, np.ndarray]] | None = None,
) -> tuple[float, float, float]:
    """Kinetic, potential and total energy — the quantity a frictionless run must conserve.

    The applied loads are part of the potential: a constant force `F` at a point `x` contributes
    `−F·x`. Without that term the "energy" of a loaded mechanism is not conserved at all, and a
    conservation test would be measuring the load's work instead of the integrator.
    """
    model = problem.model
    dynamics_block = model.geometry.dynamics
    if dynamics_block is None:
        raise SolverError("this model has no `dynamics:` block, so it has no energy")
    g = problem.numbers()
    kinetic = 0.5 * float(qdot @ mass_matrix(problem, q, g) @ qdot)

    potential = 0.0
    values = problem.evaluate(q, g, problem.theta0())
    compliance = problem.compliance(g)
    for index, row in enumerate(problem.rows):
        if row.group != SPRING:
            continue
        joint, component = row.name.split(".", 1)
        c = compliance.get((joint, component), 0.0)
        if c:
            potential += 0.5 * values[index] ** 2 / c

    weight = np.asarray(dynamics_block.gravity if gravity is None else gravity, dtype=float)
    for body in problem.moving:
        numbers = dynamics_block.bodies.get(body)
        if numbers is None:
            continue
        block = _body_indices(problem, body)
        w, x, y, z = q[block][3:7]
        offset = (
            problem.resolved.points[numbers.com]
            if isinstance(numbers.com, str)
            else np.asarray(numbers.com, dtype=float)
        )
        position = q[block][:3] + _rotation(w, x, y, z) @ np.asarray(offset, dtype=float)
        potential -= numbers.mass * float(weight @ position)

    if loads:
        world = problem.world_points(q)
        for _, point, applied in loads:
            potential -= float(np.asarray(applied, dtype=float) @ world[point])
    return kinetic, potential, kinetic + potential


def _project(problem: Problem, q: np.ndarray, qdot: np.ndarray, g: np.ndarray, theta: np.ndarray):
    """Put the state back on the manifold: `Φ(q) = 0` and `Φ_q q̇ = 0`."""
    rows = [i for i, row in enumerate(problem.rows) if row.group != SPRING]
    for _ in range(4):
        values = problem.evaluate(q, g, theta)[rows]
        if np.max(np.abs(values)) < 1e-13:
            break
        jac = problem.jacobian(q, g, theta)[rows]
        step, *_ = np.linalg.lstsq(jac, -values, rcond=None)
        q = q + step
    for _ in range(2):
        jac = problem.jacobian(q, g, theta)[rows]
        drift = jac @ qdot
        if np.max(np.abs(drift)) < 1e-13:
            break
        step, *_ = np.linalg.lstsq(jac, -drift, rcond=None)
        qdot = qdot + step
    return q, qdot


def integrate(
    problem: Problem,
    q: np.ndarray,
    qdot: np.ndarray,
    *,
    duration: float,
    steps: int,
    loads: list[tuple[str, str, np.ndarray]] | None = None,
    gravity: np.ndarray | None = None,
    theta: np.ndarray | None = None,
) -> Trajectory:
    """Classical RK4 with projection after every step.

    Projection is not an ornament: RK4 drifts off `Φ = 0` by `O(h⁴)` per step, which over a few
    hundred steps is a visible violation of a rigid link. Projecting `q` onto `Φ = 0` and `q̇` onto
    `Φ_q q̇ = 0` after each accepted step keeps the constraint error at roundoff and is what makes
    the energy test meaningful — the alternative, a Baumgarte term, trades constraint drift for
    energy drift.
    """
    if steps < 2 or duration <= 0:
        raise SolverError("an integration needs a positive duration and at least two steps")
    g = problem.numbers()
    theta = problem.theta0() if theta is None else theta
    h = duration / (steps - 1)
    q = np.array(q, dtype=float)
    qdot = np.array(qdot, dtype=float)

    def derivative(state_q, state_v):
        # A linear combination of unit quaternions is not a unit quaternion, so the intermediate
        # RK4 stages leave the gauge surface. Renormalising before evaluating the acceleration
        # keeps every stage on it; without this the constraint reaction leaks through the radial
        # direction and the energy drifts by an amount that does not shrink with the step.
        stage = state_q.copy()
        for body in problem.moving:
            start = problem._qstart[body] + 3
            norm = float(np.linalg.norm(stage[start : start + 4]))
            if norm:
                stage[start : start + 4] /= norm
        return state_v, accelerations(problem, stage, state_v, loads=loads, gravity=gravity, theta=theta).qddot

    times = np.linspace(0.0, duration, steps)
    states = np.zeros((steps, problem.n))
    velocities = np.zeros((steps, problem.n))
    energies = np.zeros(steps)
    constraints = np.zeros(steps)
    rows = [i for i, row in enumerate(problem.rows) if row.group != SPRING]

    for index, time in enumerate(times):
        if index > 0:
            k1q, k1v = derivative(q, qdot)
            k2q, k2v = derivative(q + 0.5 * h * k1q, qdot + 0.5 * h * k1v)
            k3q, k3v = derivative(q + 0.5 * h * k2q, qdot + 0.5 * h * k2v)
            k4q, k4v = derivative(q + h * k3q, qdot + h * k3v)
            q = q + (h / 6.0) * (k1q + 2.0 * k2q + 2.0 * k3q + k4q)
            qdot = qdot + (h / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v)
            q, qdot = _project(problem, q, qdot, g, theta)
        states[index] = q
        velocities[index] = qdot
        energies[index] = energy(problem, q, qdot, gravity, loads)[2]
        constraints[index] = float(np.max(np.abs(problem.evaluate(q, g, theta)[rows])))
    return Trajectory(t=times, q=states, qdot=velocities, energy=energies, constraint=constraints)
