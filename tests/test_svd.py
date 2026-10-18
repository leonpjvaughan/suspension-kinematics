"""Acceptance 3: the SVD finds exactly one null direction, and names what is in it."""

from __future__ import annotations

import numpy as np

from otkd.problem.compile import Problem
from otkd.report import diagnose_redundancy
from otkd.solvers.system import System, left_null, rank_and_null


def test_exactly_one_null_direction(two_leg):
    diagnosis = diagnose_redundancy(two_leg)
    assert diagnosis.redundancy == 1
    assert diagnosis.null_space.shape[1] == 1
    assert diagnosis.rank == 21
    # 22 rows, 21 unknowns: one singular value more than the unknown count would be needed
    # to see the zero explicitly, so assert the count, not a threshold.
    assert len(diagnosis.singular_values) == 21
    assert diagnosis.singular_values[-1] > 1e-3


def test_dependent_rows_are_named(two_leg):
    diagnosis = diagnose_redundancy(two_leg)
    assert "uca.unit" in diagnosis.dependent_rows
    assert any(name.startswith("uca-front") for name in diagnosis.dependent_rows)


def test_dropping_a_dependent_row_restores_a_well_conditioned_system(two_leg):
    problem = Problem(two_leg)
    g = problem.numbers()
    q0 = problem.q0()
    theta = problem.theta0()
    for name in ("uca-front.ty", "uca-front.tz", "uca-rear.ty"):
        system = System(problem, g, theta, (name,))
        rank, null, singular = rank_and_null(system.jacobian(q0))
        assert rank == problem.n
        assert null.shape[1] == 0
        assert singular[0] / singular[-1] < 1e3


def test_dropping_an_independent_row_does_not(two_leg):
    problem = Problem(two_leg)
    system = System(problem, problem.numbers(), problem.theta0(), ("uca-front.tx",))
    rank, null, _ = rank_and_null(system.jacobian(problem.q0()))
    assert rank == problem.n - 1
    assert null.shape[1] == 1
