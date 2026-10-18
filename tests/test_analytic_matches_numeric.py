"""Acceptance 1: the closed form and the symbolic solver are the same answer.

Two derivations that share nothing but the input file: explicit arm angles and circle
intersections on one side, quaternions, a symbolic Jacobian and Newton on the other.
"""

from __future__ import annotations

import numpy as np
import pytest

import otkd
from otkd.analysis import analytic
from otkd.problem.compile import Problem


def test_camber_agrees_over_the_sweep(wishbone):
    sweep = otkd.solve(wishbone)
    assert sweep.analytic is not None, "the wishbone must have a closed form"
    assert sweep.agreement is not None
    assert sweep.agreement < 1e-9


def test_positions_agree_over_the_sweep(wishbone):
    problem = Problem(wishbone)
    wish = analytic.extract(problem)
    assert wish is not None
    sweep = otkd.solve(wishbone)
    poses = analytic.walk(wish, sweep.theta)
    for frame, pose in zip(sweep.frames, poses):
        for name, expected in (("lbj", pose["lbj"]), ("ubj", pose["ubj"]), ("tro", pose["tro"])):
            assert np.allclose(frame.points[name], expected, atol=1e-9), (name, frame.index)


def test_closed_form_finds_the_design_position(wishbone):
    problem = Problem(wishbone)
    wish = analytic.extract(problem)
    pose = analytic.walk(wish, np.array([0.0]))[0]
    assert np.allclose(pose["lbj"], wish.lbj, atol=1e-12)
    assert np.allclose(pose["ubj"], wish.ubj, atol=1e-12)
    assert abs(pose["camber"]) < 1e-12
