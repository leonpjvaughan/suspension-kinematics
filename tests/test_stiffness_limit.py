"""Acceptance 4: compliance converges to rigidity as the compliance goes to zero.

The ladder is swept in *compliance*, not stiffness, so the limit is `c -> 0`: the rigid model
is the endpoint of the family, not an approximation to it.
"""

from __future__ import annotations

import numpy as np

import otkd


def _camber(sweep):
    return np.array([frame.outputs.camber for frame in sweep.frames])


def test_camber_approaches_the_rigid_curve_monotonically(compliant, wishbone):
    rigid = _camber(otkd.solve(wishbone))
    deviations = []
    for compliance in (1e-4, 1e-5, 1e-6):
        model = otkd.load(compliant.files["topology"])
        model.geometry.joints["tie-rod"].compliance["length"] = compliance
        sweep = otkd.solve(model)
        assert sweep.kind == "equilibrium"
        deviations.append(float(np.max(np.abs(_camber(sweep) - rigid))))
    assert all(later < earlier for earlier, later in zip(deviations, deviations[1:])), deviations
    assert deviations[-1] < 1e-3


def test_zero_compliance_is_the_rigid_model_exactly(compliant, wishbone):
    model = otkd.load(compliant.files["topology"])
    model.geometry.joints["tie-rod"].compliance["length"] = 0.0
    sweep = otkd.solve(model)
    assert sweep.kind == "square"
    rigid = otkd.solve(wishbone)
    for left, right in zip(sweep.frames, rigid.frames):
        assert abs(left.outputs.camber - right.outputs.camber) < 1e-12
        assert np.max(np.abs(left.q - right.q)) < 1e-12


def test_without_a_load_compliance_is_invisible(compliant):
    model = otkd.load(compliant.files["topology"])
    model.study.loads = []
    soft = otkd.solve(model)
    model.geometry.joints["tie-rod"].compliance["length"] = 0.0
    hard = otkd.solve(model)
    for left, right in zip(soft.frames, hard.frames):
        assert abs(left.outputs.camber - right.outputs.camber) < 1e-9
