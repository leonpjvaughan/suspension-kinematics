"""Driving the road and driving the arm are the same mechanism.

For each road-driven frame, read the lower arm's angle out of the solved pose, solve the
arm-driven model at that angle, and require the poses to agree. That is the test that says
"which end do you drive?" is a property of the question, not of the model.
"""

from __future__ import annotations

import numpy as np

import otkd
from otkd.problem.compile import Problem
from otkd.solvers.newton import newton
from otkd.solvers.system import System


def test_road_drive_reproduces_arm_drive(ride_height, wishbone):
    road = Problem(ride_height)
    arm = Problem(wishbone)
    g_road = road.numbers()
    g_arm = arm.numbers()
    axis = road.resolved.sites["lca-hinge"].normal
    index = road._qstart["lca"]

    for frame in otkd.solve(ride_height).frames:
        w, x, y, z = frame.q[index + 3 : index + 7]
        theta = 2.0 * float(np.arctan2(np.array([x, y, z]) @ axis, w))
        system = System(arm, g_arm, arm.theta_for({"lca-drive": theta}))
        q, _, _ = newton(system, arm.q0())
        for name in ("lca", "uca", "up"):
            assert np.allclose(frame.q[road._qstart[name] : road._qstart[name] + 7],
                               q[arm._qstart[name] : arm._qstart[name] + 7], atol=1e-9), (frame.index, name)


def test_the_contact_patch_is_solved_not_projected(ride_height):
    sweep = otkd.solve(ride_height)
    for frame in sweep.frames:
        assert frame.outputs.contact is not None
        assert abs(frame.outputs.contact[1] - frame.points["patch"][2]) < 1e-12


def test_the_driven_coordinate_is_a_length(ride_height):
    sweep = otkd.solve(ride_height)
    assert sweep.unit == "m"
    assert abs(sweep.frames[0].values["road-height"] + 0.042) < 1e-12
