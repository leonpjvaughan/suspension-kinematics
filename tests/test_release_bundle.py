"""The bundle's manifest witnesses the model it was built from.

A release is transport for someone else's build. What it carries about *provenance* has to be
checkable from the bundle alone, because a dynamics model legitimately ships no payload and no
kernel — so the config files' digests are the only thing that says which bytes produced a
trajectory a consumer vendored separately.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import otkd
from otkd.emitters.wasm import copy_sources, sources

from tests.helpers import EXAMPLES, build

ROOT = Path(__file__).resolve().parents[1]


def _load():
    spec = importlib.util.spec_from_file_location("otkd_build_release", ROOT / "scripts" / "build_release.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


bundle = _load()


def _paths(model) -> dict[str, Path]:
    return {Path(path).name: Path(path) for path in model.files.values() if path}


def test_a_dynamics_model_ships_a_kernel_but_no_payload(tmp_path):
    """A trajectory is not a sweep, so there is no payload — and the integrator is a kernel."""
    status = bundle.build_model(EXAMPLES / "quarter-car", tmp_path)
    assert status["payload"] == "none (dynamics)"
    assert status["kernel"] == "kernel.wasm", status["kernel"]
    # Three files and the library: the library is the one a consumer cannot see from the model
    # directory, and the one a release could change without touching the model.
    assert set(status["sources"]) == {
        "topology.yaml",
        "geometry.yaml",
        "study.yaml",
        "constraints.yaml",
    }


def test_every_recorded_digest_is_the_digest_of_the_file(tmp_path):
    status = bundle.build_model(EXAMPLES / "quarter-car", tmp_path)
    by_name = _paths(otkd.load(EXAMPLES / "quarter-car"))
    for name, digest in status["sources"].items():
        assert digest == hashlib.sha256(by_name[name].read_bytes()).hexdigest(), name


def test_the_library_is_witnessed_like_the_model_files(tmp_path):
    """A kind's mode map lives in the library, so a change there changes the model."""
    root = build(tmp_path, "double-wishbone")
    before = sources(otkd.load(root))

    geometry = root / "geometry.yaml"
    geometry.write_text(geometry.read_text() + "\n# a byte, and nothing else\n")
    after = sources(otkd.load(root))
    assert after["geometry.yaml"] != before["geometry.yaml"]
    assert after["topology.yaml"] == before["topology.yaml"]

    library = tmp_path / "constraints.yaml"
    library.write_text(library.read_text() + "\n# a byte, and nothing else\n")
    assert sources(otkd.load(root))["constraints.yaml"] != before["constraints.yaml"]


def test_the_manifest_names_the_bundle_format_and_no_build_time(tmp_path):
    bundle.write_manifest(tmp_path, "9.9.9", {"example": {"id": "x"}})
    manifest = json.loads((tmp_path / "MANIFEST.json").read_text())
    assert manifest["bundle"] == "otkd-bundle-2"
    assert manifest["version"] == "9.9.9"
    assert manifest["models"]["example"]["id"] == "x"
    # Reproducible transport: no clock, no host, nothing that differs between two builds.
    assert not {"built", "date", "timestamp"} & set(manifest)


def test_a_dynamics_model_ships_its_sources_and_its_kernel(tmp_path):
    """The bytes, not only the digests: a consumer compares a trajectory against them."""
    status = bundle.build_model(EXAMPLES / "quarter-car", tmp_path)
    assert status["payload"] == "none (dynamics)"
    written = tmp_path / "models" / "quarter-car"
    assert sorted(path.name for path in (written / "source").iterdir()) == sorted(status["sources"])
    assert not (written / "out.js").exists()
    assert (written / "kernel" / "kernel.wasm").exists()
    source = (written / "source" / "geometry.yaml")
    assert source.read_bytes() == (EXAMPLES / "quarter-car" / "geometry.yaml").read_bytes()
