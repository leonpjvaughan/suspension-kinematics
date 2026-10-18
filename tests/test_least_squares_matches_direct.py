"""Acceptance 2: the least-squares solution equals the square one once the redundancy is gone."""

from __future__ import annotations

import numpy as np
import yaml

import otkd
from tests.helpers import build


def test_dropping_the_dependent_row_reproduces_least_squares(two_leg, tmp_path):
    redundant = otkd.solve(two_leg)
    assert redundant.kind == "redundant"

    root = build(tmp_path / "dropped", "double-wishbone-two-leg")
    study = yaml.safe_load((root / "study.yaml").read_text())
    study["drop"] = ["uca-front.ty"]
    (root / "study.yaml").write_text(yaml.safe_dump(study, sort_keys=False))

    square = otkd.solve(otkd.load(root))
    assert square.kind == "square"
    assert len(square.frames) == len(redundant.frames)
    for left, right in zip(square.frames, redundant.frames):
        assert abs(left.outputs.camber - right.outputs.camber) < 1e-9
        assert np.max(np.abs(left.q - right.q)) < 1e-9


def test_the_redundant_residual_floor_is_roundoff(two_leg):
    sweep = otkd.solve(two_leg)
    for frame in sweep.frames:
        residual = frame.history[-1][1]
        assert np.max(np.abs(residual)) < 1e-12
