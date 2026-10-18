"""Outputs: the travel the examples sweep, and the instant-centre construction."""

from __future__ import annotations

import numpy as np
import yaml

import otkd
from otkd.analysis import outputs
from otkd.problem.compile import Problem
from otkd.solvers.newton import newton
from otkd.solvers.system import System
from tests.helpers import build


def test_the_wheel_travel_is_the_stated_stroke(wishbone):
    """The sweep is ±0.1 rad, not the handoff's ±0.35: its own geometry makes that 290 mm."""
    sweep = otkd.solve(wishbone)
    height = np.array([frame.points["wc"][2] for frame in sweep.frames])
    stroke = float(height.max() - height.min())
    assert 0.07 < stroke < 0.10, f"{stroke:.3f} m of wheel travel"
    steps = np.diff(height)
    assert np.all(steps > 0) or np.all(steps < 0), "the wheel centre should move monotonically"


def _solve(problem: Problem, theta: float) -> np.ndarray:
    system = System(problem, problem.numbers(), problem.theta_for({"lca-drive": theta}))
    q, _, _ = newton(system, problem.q0())
    return q


def test_the_instant_centre_is_exact_for_a_perpendicular_axis(tmp_path):
    """With the hinge axes along y the front view *is* a rotation, and the line through the
    ball joint and the instant centre passes through the hinge origin."""
    root = build(tmp_path, "double-wishbone")
    geometry = yaml.safe_load((root / "geometry.yaml").read_text())
    geometry["axes"]["lca-axis"] = [0.0, -1.0, 0.0]
    geometry["axes"]["uca-axis"] = [0.0, -1.0, 0.0]
    (root / "geometry.yaml").write_text(yaml.safe_dump(geometry, sort_keys=False))

    model = otkd.load(root)
    problem = Problem(model)
    reference = outputs.make_reference(problem)
    for theta in (-0.05, 0.0, 0.05):
        q = _solve(problem, theta)
        result = outputs.compute(problem, q, reference)
        assert result.ic is not None
        points = problem.world_points(q)
        for ball, hinge in (("lbj", "lcaf"), ("ubj", "ucaf")):
            a = np.array([points[ball][0], points[ball][2]])
            b = np.array([points[hinge][0], points[hinge][2]])
            c = np.array(result.ic)
            # collinear: the 2D cross product of the two directions vanishes
            u, v = b - a, c - a
            assert abs(u[0] * v[1] - u[1] * v[0]) < 1e-6 * np.linalg.norm(u) * np.linalg.norm(v), (
                theta,
                ball,
            )


def test_the_two_leg_model_has_the_same_instant_centre(wishbone, two_leg):
    """The same mechanism, expressed with two bushings instead of a hinge."""
    left = otkd.solve(wishbone).frames[4].outputs
    right = otkd.solve(two_leg).frames[4].outputs
    assert np.allclose(left.ic, right.ic, atol=1e-9)
    assert abs(left.rc[1] - right.rc[1]) < 1e-9


def test_the_roll_centre_lies_on_the_line_from_the_contact_patch(wishbone):
    sweep = otkd.solve(wishbone)
    for frame in sweep.frames[::2]:
        out = frame.outputs
        assert out.ic is not None and out.rc is not None
        contact = np.array(out.contact)
        # rc is at x = 0 on the line through the contact patch and the instant centre
        line = np.array(out.ic) - contact
        predicted = contact[1] + (-contact[0]) * line[1] / line[0]
        assert abs(predicted - out.rc[1]) < 1e-9
