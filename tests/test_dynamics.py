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
from tests.helpers import build

MASSES = {
    "lca": (3.2, [0.500, 0.0, 0.150], [0.021, 0.140, 0.131, 0.0, 0.002, 0.0]),
    "uca": (1.8, [0.530, 0.0, 0.500], [0.008, 0.043, 0.041, 0.0, 0.0, 0.0]),
    "up": (12.0, [0.730, 0.0, 0.310], [0.310, 0.180, 0.310, 0.0, 0.0, 0.0]),
}
LOADS = [("up", "wc", np.array([0.0, 0.0, -3500.0])), ("up", "wc", np.array([1800.0, 0.0, 0.0]))]


@pytest.fixture
def compliant(tmp_path):
    """The compliant corner plus mass properties, which the kinematic examples do not carry."""
    root = build(tmp_path, "double-wishbone-compliant")
    topology = yaml.safe_load((root / "topology.yaml").read_text())
    for body in MASSES:
        topology["bodies"][body].setdefault("points", []).append(f"com-{body}")
    (root / "topology.yaml").write_text(yaml.safe_dump(topology, sort_keys=False))

    geometry = yaml.safe_load((root / "geometry.yaml").read_text())
    for body, (_, com, _) in MASSES.items():
        geometry["points"][f"com-{body}"] = com
    geometry["dynamics"] = {
        "gravity": [0.0, 0.0, -9.81],
        "bodies": {
            body: {"mass": mass, "com": f"com-{body}", "inertia": inertia}
            for body, (mass, _, inertia) in MASSES.items()
        },
    }
    (root / "geometry.yaml").write_text(yaml.safe_dump(geometry, sort_keys=False))
    return otkd.load(root)


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
