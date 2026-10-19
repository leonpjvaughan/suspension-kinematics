"""A quarter car: a chassis that can heave, a coil-over, and a tyre on the road.

This is the multi-body case. What it adds over the compliant corner is a body with real mass and
a degree of freedom the *world* does not fix, which is where the equations of motion earn their
velocity-product term.
"""

from __future__ import annotations

import numpy as np
import pytest

import otkd
from otkd.problem import dynamics
from otkd.problem.compile import Problem


@pytest.fixture(scope="module")
def quarter_car():
    return otkd.load("examples/quarter-car")


def test_a_heaving_chassis_is_one_degree_of_freedom(quarter_car):
    report = otkd.check(quarter_car)
    assert report.mobility == 1
    assert report.redundancy == 0


def test_the_released_chassis_conserves_energy(quarter_car):
    problem = Problem(quarter_car)
    trajectory = dynamics.integrate(
        problem, problem.q0(), np.zeros(problem.n), duration=0.3, steps=301
    )
    assert np.max(trajectory.constraint) < 1e-9
    reference = trajectory.energy[0]
    drift = np.max(np.abs(trajectory.energy - reference)) / abs(reference)
    assert drift < 1e-6, drift
    heave = trajectory.q[:, problem._qstart["chassis"] + 2]
    assert heave.min() < -1e-3  # it falls, and the coil-over brings it back


def test_the_energy_error_shrinks_with_the_step(quarter_car):
    """The signature of a correct integrator, and of a correct force field."""
    problem = Problem(quarter_car)
    coarse = dynamics.integrate(problem, problem.q0(), np.zeros(problem.n), duration=0.2, steps=101)
    fine = dynamics.integrate(problem, problem.q0(), np.zeros(problem.n), duration=0.2, steps=401)
    coarse_drift = np.ptp(coarse.energy) / abs(coarse.energy[0])
    fine_drift = np.ptp(fine.energy) / abs(fine.energy[0])
    assert fine_drift < coarse_drift / 10.0, (coarse_drift, fine_drift)
