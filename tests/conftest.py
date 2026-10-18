from __future__ import annotations

import pytest

import otkd
from tests.helpers import build


@pytest.fixture
def wishbone(tmp_path):
    return otkd.load(build(tmp_path, "double-wishbone"))


@pytest.fixture
def two_leg(tmp_path):
    return otkd.load(build(tmp_path, "double-wishbone-two-leg"))


@pytest.fixture
def compliant(tmp_path):
    return otkd.load(build(tmp_path, "double-wishbone-compliant"))


@pytest.fixture
def ride_height(tmp_path):
    return otkd.load(build(tmp_path, "ride-height"))
