"""The emitted module is stable, complete, and the same object `solve --json` would print."""

from __future__ import annotations

import json

import pytest

import otkd
from otkd.emitters.js import payload, render
from tests.helpers import EXAMPLES


def test_the_committed_module_is_reproducible(tmp_path):
    target = tmp_path / "out.js"
    otkd.emit(otkd.load(EXAMPLES / "double-wishbone"), target)
    assert target.read_text() == (EXAMPLES / "double-wishbone" / "out.js").read_text()


def test_two_exports_are_byte_identical(wishbone):
    first = render(payload(otkd.solve(wishbone)))
    second = render(payload(otkd.solve(wishbone)))
    assert first == second


def test_the_payload_has_the_contract_keys(wishbone):
    sweep = otkd.solve(wishbone)
    data = payload(sweep)
    assert data["meta"]["frames"] == len(sweep.frames) == len(data["sweep"]["frames"])
    assert data["meta"]["units"] == {"length": "m", "angle": "deg"}
    assert data["meta"]["travel"]["unit"] == "rad"
    # The payload is data; the version lives on the release, not in every `out.js`.
    assert data["meta"]["generator"] == "otkd"
    assert data["agreement"]["max_abs_diff"] <= 1e-9
    assert len(data["agreement"]["analytic"]) == len(sweep.frames)
    frame = data["sweep"]["frames"][0]
    for key in ("theta", "lbj", "ubj", "wc", "tro", "points", "camber", "toe", "ic", "rc",
                "track", "branchChanged"):
        assert key in frame, key
    assert len(frame["points"]) == len(sweep.problem.resolved.points)
    assert len(frame["lbj"]) == 2


def test_the_module_imports_nothing(wishbone):
    text = render(payload(otkd.solve(wishbone)))
    assert "import" not in text
    assert "fetch(" not in text
    for name in ("meta", "sweep", "iterations", "agreement"):
        assert f"export const {name} = " in text


def test_the_redundant_example_exports_its_floor(two_leg):
    data = payload(otkd.solve(two_leg))
    assert data["agreement"]["max_abs_diff"] < 1e-9  # the redundant model is still the mechanism
    iterations = data["iterations"]
    assert iterations
    assert len(iterations[-1]["residual"]) == 22  # 21 constrained rows plus the redundant one
    assert max(abs(value) for value in iterations[-1]["residual"]) < 1e-12


def test_numbers_are_rounded_not_merely_short(tmp_path):
    text = render(payload(otkd.solve(otkd.load(EXAMPLES / "double-wishbone"))))
    assert "-0.0," not in text
    assert "NaN" not in text
    assert "Infinity" not in text
    body = text.split("export const sweep = ", 1)[1].split(";", 1)[0]
    json.loads(body)
