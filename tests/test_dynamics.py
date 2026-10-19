"""Accelerations and reactions, checked where they can be checked exactly.

There is no integration here: the tests are the two statements that must hold at a state — the
converged equilibrium has no acceleration, and the acceleration satisfies the velocity
constraint to the order the constraint is differentiated.
"""

from __future__ import annotations

import numpy as np
import pytest
import yaml

import otkd
from otkd.problem import dynamics
from otkd.problem.compile import Problem
from otkd.solvers.equilibrium import equilibrium
from otkd.solvers.newton import newton
from otkd.solvers.system import System
from tests.helpers import DYNAMICS_MASSES, add_dynamics, build

MASSES = DYNAMICS_MASSES
LOADS = [("up", "wc", np.array([0.0, 0.0, -3500.0])), ("up", "wc", np.array([1800.0, 0.0, 0.0]))]


@pytest.fixture
def compliant(tmp_path):
    """The compliant corner plus mass properties, which the kinematic examples do not carry."""
    return otkd.load(add_dynamics(build(tmp_path, "double-wishbone-compliant")))


def _solve(problem: Problem, theta: float) -> np.ndarray:
    system = System(problem, problem.numbers(), problem.theta_for({"lca-drive": theta}))
    if system.springs:
        q, _, _ = equilibrium(system, problem.q0(), LOADS)
        return q
    q, _, _ = newton(system, problem.q0())
    return q


def _equilibrium(problem: Problem, compute: np.ndarray) -> np.ndarray:
    system = System(problem, compute, problem.theta0())
    q, _, _ = equilibrium(system, problem.q0(), LOADS)
    return q


def _tangent(problem: Problem, theta: float = 0.03, step: float = 1e-4) -> tuple[np.ndarray, np.ndarray]:
    """A pose and the velocity of the one-degree-of-freedom motion at it, per unit drive rate."""
    centre = _solve(problem, theta)
    plus = _solve(problem, theta + step)
    minus = _solve(problem, theta - step)
    return centre, (plus - minus) / (2.0 * step)


def test_the_mass_matrix_is_symmetric_and_positive_on_the_motion(compliant):
    problem = Problem(compliant)
    g = problem.numbers()
    q, velocity = _tangent(problem)
    mass = dynamics.mass_matrix(problem, q, g)
    assert np.allclose(mass, mass.T, atol=1e-12)
    # The quaternion directions are gauges, so the whole matrix is singular; what matters is
    # that the actual motion has positive kinetic energy.
    assert float(velocity @ mass @ velocity) > 0.0
    assert np.all(np.linalg.eigvalsh(mass) > -1e-9)


def test_translation_only_kinetic_energy_is_one_half_m_v_squared(compliant):
    problem = Problem(compliant)
    g = problem.numbers()
    q = problem.q0()
    velocity = np.zeros(problem.n)
    for body in problem.moving:
        velocity[problem._qstart[body]] = 1.0  # unit speed along x, no rotation
    mass = dynamics.mass_matrix(problem, q, g)
    total = sum(mass_ for mass_, _, _ in MASSES.values())
    assert float(0.5 * velocity @ mass @ velocity) == pytest.approx(0.5 * total, rel=1e-12)


def test_the_equilibrium_has_no_acceleration(compliant):
    problem = Problem(compliant)
    q = _equilibrium(problem, problem.numbers())
    result = dynamics.accelerations(problem, q, np.zeros(problem.n), loads=LOADS, gravity=np.zeros(3))
    assert np.max(np.abs(result.qddot)) < 1e-6
    assert all(np.isfinite(value) for value in result.reactions.values())


def test_the_acceleration_satisfies_the_constraint_to_third_order(compliant):
    """`Φ(q + h v + ½ h² a)` must be smaller than `Φ(q + h v)` by about `h`."""
    problem = Problem(compliant)
    g = problem.numbers()
    theta = problem.theta_for({"lca-drive": 0.03})
    q, velocity = _tangent(problem, theta=0.03)
    result = dynamics.accelerations(problem, q, velocity, loads=LOADS, theta=theta)
    rows = [i for i, row in enumerate(problem.rows) if row.group != "spring"]

    step = 1e-4
    # The drive moves with the state: at `q + h v` the driven coordinate is `theta + h`, because
    # `v = dq/dtheta`. Comparing against a fixed theta would leave an O(h) term in the driver row.
    first = problem.evaluate(q + step * velocity, g, theta + step)[rows]
    second = problem.evaluate(q + step * velocity + 0.5 * step**2 * result.qddot, g, theta + step)[rows]
    assert np.linalg.norm(second) < 1e-2 * np.linalg.norm(first), (np.linalg.norm(first), np.linalg.norm(second))


def test_gravity_is_reported_as_a_force_before_the_constraints(compliant):
    problem = Problem(compliant)
    q = problem.q0()
    resting = dynamics.forces(problem, q, np.zeros(problem.n), problem.numbers(), problem.theta0(), gravity=np.zeros(3))
    falling = dynamics.forces(problem, q, np.zeros(problem.n), problem.numbers(), problem.theta0())
    assert np.linalg.norm(resting) < 1e-9
    assert np.linalg.norm(falling) > 1.0


def _free_direction(problem: Problem, q: np.ndarray) -> np.ndarray:
    rows = [i for i, row in enumerate(problem.rows) if row.group != "spring"]
    jac = problem.jacobian(q, problem.numbers(), problem.theta0())[rows]
    _, _, vt = np.linalg.svd(jac, full_matrices=True)
    return vt[-1]


def test_a_frictionless_swing_conserves_energy(compliant):
    problem = Problem(compliant)
    q = _equilibrium(problem, problem.numbers())
    kick = 0.2 * _free_direction(problem, q)
    trajectory = dynamics.integrate(problem, q, kick, duration=0.4, steps=401, loads=LOADS)
    assert np.max(trajectory.constraint) < 1e-9
    reference = trajectory.energy[0]
    drift = np.max(np.abs(trajectory.energy - reference)) / max(abs(reference), 1.0)
    assert drift < 1e-6, drift  # RK4's own energy error at this step; a stiffer spring needs less
    assert np.max(np.abs(trajectory.q - trajectory.q[0])) > 1e-3  # it actually moved


def test_a_static_state_stays_static(compliant):
    problem = Problem(compliant)
    q = _equilibrium(problem, problem.numbers())
    trajectory = dynamics.integrate(
        problem, q, np.zeros(problem.n), duration=0.2, steps=201, loads=LOADS, gravity=np.zeros(3)
    )
    assert np.max(np.abs(trajectory.q - q)) < 1e-9
    assert np.ptp(trajectory.energy) / max(abs(trajectory.energy[0]), 1.0) < 1e-9
