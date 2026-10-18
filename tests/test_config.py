"""The config layer refuses what it cannot mean, and says which file and key."""

from __future__ import annotations

import shutil

import pytest
import yaml

import otkd
from otkd.errors import ConfigError
from tests.helpers import EXAMPLES, build


def mutate(root, file, change):
    path = root / file
    data = yaml.safe_load(path.read_text())
    change(data)
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return root


def test_missing_library_names_the_path_and_the_key(tmp_path):
    root = build(tmp_path, "double-wishbone")
    mutate(root, "topology.yaml", lambda d: d.__setitem__("constraints", "nowhere.yaml"))
    with pytest.raises(ConfigError) as caught:
        otkd.check(otkd.load(root))
    text = str(caught.value)
    assert "nowhere.yaml" in text
    assert "constraints" in text


def test_a_kind_defined_twice_is_refused(tmp_path):
    root = build(tmp_path, "double-wishbone")

    def add(data):
        data["constraint_kinds"] = {
            "revolute": {"family": "frame", "modes": {c: "free" for c in ("tx", "ty", "tz", "rx", "ry", "rz")}}
        }

    mutate(root, "topology.yaml", add)
    with pytest.raises(ConfigError, match="defined twice"):
        otkd.check(otkd.load(root))


def test_unknown_kind(tmp_path):
    root = build(tmp_path, "double-wishbone")
    mutate(root, "topology.yaml", lambda d: d["joints"][0].__setitem__("kind", "magic"))
    with pytest.raises(ConfigError, match="unknown kind"):
        otkd.check(otkd.load(root))


def test_every_point_must_belong_to_a_body(tmp_path):
    root = build(tmp_path, "double-wishbone")
    mutate(root, "topology.yaml", lambda d: d["bodies"]["up"]["points"].remove("wco"))
    with pytest.raises(ConfigError, match="belongs to no body"):
        otkd.check(otkd.load(root))


def test_a_spring_needs_a_compliance(tmp_path):
    root = build(tmp_path, "double-wishbone-compliant")
    mutate(root, "geometry.yaml", lambda d: d["joints"]["tie-rod"].pop("compliance"))
    with pytest.raises(ConfigError, match="compliance"):
        otkd.check(otkd.load(root))


def test_a_locked_component_may_not_have_one(tmp_path):
    root = build(tmp_path, "double-wishbone")
    mutate(root, "geometry.yaml", lambda d: d.setdefault("joints", {}).__setitem__("tie-rod", {"compliance": {"length": 1e-6}}))
    with pytest.raises(ConfigError, match="silently ignored"):
        otkd.check(otkd.load(root))


def test_an_actuator_may_not_drive_a_locked_component(tmp_path):
    root = build(tmp_path, "double-wishbone")
    mutate(root, "topology.yaml", lambda d: d["actuators"][0].__setitem__("dof", "tx"))
    with pytest.raises(ConfigError, match="not `free`"):
        otkd.check(otkd.load(root))


def test_a_length_that_disagrees_with_the_geometry_is_named(tmp_path):
    root = build(tmp_path, "double-wishbone")
    mutate(root, "geometry.yaml", lambda d: d.setdefault("joints", {}).__setitem__("tie-rod", {"length": 0.371}))
    with pytest.raises(ConfigError) as caught:
        otkd.check(otkd.load(root))
    assert "0.371" in str(caught.value)
    assert "design position" in str(caught.value)


def test_an_unknown_version_is_refused(tmp_path):
    root = build(tmp_path, "double-wishbone")
    mutate(root, "geometry.yaml", lambda d: d.__setitem__("version", 2))
    with pytest.raises(ConfigError, match="version 1"):
        otkd.check(otkd.load(root))


def test_an_under_determined_model_names_the_free_directions(tmp_path):
    root = build(tmp_path, "double-wishbone")
    mutate(root, "topology.yaml", lambda d: d.__setitem__("actuators", []))
    with pytest.raises(ConfigError) as caught:
        otkd.check(otkd.load(root))
    assert "unconstrained" in str(caught.value)


def test_the_report_counts_what_it_should(wishbone):
    report = otkd.check(wishbone)
    assert report.mobility == 1
    assert report.redundancy == 0
    assert report.actuators == 1
    assert report.rank == 20


def test_two_leg_is_redundant_by_one(two_leg):
    report = otkd.check(two_leg)
    assert report.mobility == 1
    assert report.redundancy == 1


def test_ride_height_is_one_degree_of_freedom(ride_height):
    report = otkd.check(ride_height)
    assert report.mobility == 1
    assert report.redundancy == 0
