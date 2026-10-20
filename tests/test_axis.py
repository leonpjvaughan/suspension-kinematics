"""A study axis: one run, a ladder.

An axis sweeps a geometry number *outside* the drivers, so one export carries the stiffness ladder
a chapter asks for instead of one export per stiffness. These tests cover the parts that are easy
to get wrong — the number's name, the seeding of each pass, and the shape of what comes out —
rather than the arithmetic, which is the equilibrium solve's job.
"""

from __future__ import annotations

import numpy as np
import pytest
import yaml

import otkd
from otkd.emitters.js import payload
from otkd.errors import ConfigError
from tests.helpers import build

TIE_ROD = "compliance.tie-rod.length"


def _axis(root, numbers, values):
    study = yaml.safe_load((root / "study.yaml").read_text())
    study["axes"] = [{"numbers": numbers, "law": {"kind": "table", "values": values}}]
    (root / "study.yaml").write_text(yaml.safe_dump(study, sort_keys=False))
    return root


@pytest.fixture(scope="module")
def two_pass(tmp_path_factory):
    root = _axis(
        build(tmp_path_factory.mktemp("axis"), "double-wishbone-compliant", steps=3),
        [TIE_ROD],
        [1e-4, 1e-6],
    )
    return otkd.solve(otkd.load(root))


def test_a_number_is_named_without_the_compilers_index(tmp_path):
    """`compliance.tie-rod.length`, not `…length.0`: the index is the compiler's bookkeeping.

    A point's three components keep theirs, because there the index is part of the name.
    """
    from otkd.problem.compile import Problem

    problem = Problem(otkd.load(build(tmp_path, "double-wishbone-compliant", steps=3)))
    assert TIE_ROD in problem.number_names
    assert f"{TIE_ROD}.0" not in problem.number_names
    assert "point.lcaf.0" in problem.number_names
    assert problem.number_index(TIE_ROD) == problem.number_names.index(TIE_ROD)


def test_the_drivers_are_swept_once_per_axis_value(two_pass):
    assert len(two_pass.frames) == 6
    assert [frame.axis for frame in two_pass.frames] == [(1e-4,)] * 3 + [(1e-6,)] * 3
    assert two_pass.axis_numbers == [[TIE_ROD]]
    assert two_pass.axis_values == [[1e-4, 1e-6]]
    travel = [frame.values["lca-drive"] for frame in two_pass.frames]
    assert travel[:3] == travel[3:]


def test_each_pass_is_a_fresh_solve_from_the_design_pose(tmp_path):
    """A soft bush and a stiff bush are different assemblies.

    The second pass must be what a single run at that stiffness gives, not a continuation of the
    first pass's solution — which is what `seed: previous` would have made it.
    """
    root = _axis(build(tmp_path / "swept", "double-wishbone-compliant", steps=3), [TIE_ROD], [1e-4, 1e-6])
    two_pass = otkd.solve(otkd.load(root))

    single_root = build(tmp_path / "single", "double-wishbone-compliant", steps=3)
    geometry = yaml.safe_load((single_root / "geometry.yaml").read_text())
    geometry["joints"]["tie-rod"]["compliance"]["length"] = 1e-6
    (single_root / "geometry.yaml").write_text(yaml.safe_dump(geometry, sort_keys=False))
    single = otkd.solve(otkd.load(single_root))

    assert np.allclose(two_pass.frames[3].q, single.frames[0].q)


def test_the_payload_says_which_numbers_moved(two_pass):
    data = payload(two_pass)
    assert data["sweep"]["axes"] == [{"numbers": [TIE_ROD], "values": [1e-4, 1e-6]}]
    assert [frame["axis"] for frame in data["sweep"]["frames"]] == [[1e-4]] * 3 + [[1e-6]] * 3


def test_an_unknown_number_names_the_file_the_axis_and_the_nearest_number(tmp_path):
    root = _axis(
        build(tmp_path, "double-wishbone-compliant", steps=3),
        ["compliance.tie-rod.lenth"],
        [1e-6, 1e-5],
    )
    with pytest.raises(ConfigError) as failure:
        otkd.solve(otkd.load(root))
    assert failure.value.faults[0].key == "axes[0].numbers"
    assert failure.value.faults[0].file.endswith("study.yaml")
    assert "did you mean compliance.tie-rod.length?" in str(failure.value)
