"""The generated kernel is the same derivation, compiled.

Two things are worth testing and this file tests both: that the Rust really is generated from
the model (so it is rebuilt, not maintained), and that it agrees with the Python engine on
poses, residuals and the Jacobian. The numeric comparison runs the *wasm* module under node,
because that is the artefact a page will load.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest
import yaml

import otkd
from otkd.emitters.wasm import TARGET, emit_wasm
from otkd.analysis import outputs
from otkd.problem.compile import Problem
from otkd.solvers.equilibrium import equilibrium
from otkd.solvers.least_squares import least_squares
from otkd.solvers.newton import newton
from otkd.solvers.system import System
from tests.helpers import EXAMPLES, build

THETAS = (-0.1, -0.03, 0.0, 0.06, 0.1)


def _reference(model):
    """Poses and outputs from the Python engine, at the same drive values."""
    problem = Problem(model)
    g = problem.numbers()
    reference = outputs.make_reference(problem)
    frames = []
    for theta in THETAS:
        system = System(problem, g, problem.theta_for({"lca-drive": theta}))
        kind = system.classify()
        if kind == "redundant":
            q = least_squares(system, problem.q0())[0]
        elif kind == "equilibrium":
            loads = [
                (problem.resolved.point_body[problem.topology.sites[load.site].origin],
                 problem.topology.sites[load.site].origin,
                 np.asarray(load.force, dtype=float))
                for load in (model.study.loads if model.study else [])
            ]
            q = equilibrium(system, problem.q0(), loads)[0]
        else:
            q = newton(system, problem.q0())[0]
        metrics = outputs.compute(problem, q, reference)
        frames.append(
            {
                "theta": float(theta),
                "q": q.tolist(),
                "points": {name: list(value) for name, value in problem.world_points(q).items()},
                "metrics": {
                    "camber": metrics.camber,
                    "toe": metrics.toe,
                    "track": metrics.track,
                    "contact": list(metrics.contact),
                    # `None` where the model has no front-view construction: a bushed ball joint
                    # is not a rigid one, so the instant centre is undefined rather than wrong.
                    "ic": list(metrics.ic) if metrics.ic is not None else None,
                    "rc": list(metrics.rc) if metrics.rc is not None else None,
                },
            }
        )
    return {
        "points": model.geometry.points,
        "axes": model.geometry.axes,
        "numbers": [None],
        "frames": frames,
        "design": {
            "rows": problem.evaluate(problem.q0(), g, problem.theta0()).tolist(),
            "jac": problem.jacobian(problem.q0(), g, problem.theta0()).tolist(),
        },
    }


def _node_available() -> bool:
    return shutil.which("node") is not None


SCRIPT = """
import { readFile, writeFile } from 'node:fs/promises';
import kernel from './kernel.js';

const ref = JSON.parse(await readFile('./reference.json'));
await kernel.ready();
const status = kernel.setGeometry(ref);
kernel.design();
kernel.setDrive(0);
const at_design = kernel.evaluate();

const frames = [];
for (const frame of ref.frames) {
  kernel.design();
  kernel.setDrive(frame.theta);
  const residual = kernel.solve(40);
  frames.push({ residual, q: kernel.pose(), points: kernel.world(), metrics: kernel.metrics() });
}
await writeFile('./result.json', JSON.stringify({
  status, rows: at_design.rows, jac: at_design.jac, frames,
  per_frame_ms: (() => {
    const t = performance.now();
    for (let i = 0; i < 200; i++) { kernel.design(); kernel.setDrive(0.05); kernel.solve(40); }
    return (performance.now() - t) / 200;
  })(),
}));
"""


def _emit_and_run(tmp_path_factory, name: str):
    root = tmp_path_factory.mktemp(f"wasm-{name}")
    model = otkd.load(build(root, name))
    out = root / "kernel"
    written = emit_wasm(model, out)
    if written["wasm"] is None:
        pytest.skip(written["note"] or "no wasm toolchain")
    if not _node_available():
        pytest.skip("node is not installed: cannot run the module")
    reference = _reference(model)
    manifest = json.loads((out / "kernel.json").read_text())
    reference["numbers"] = manifest["default_numbers"]
    (out / "reference.json").write_text(json.dumps(reference))
    (out / "run.mjs").write_text(SCRIPT)
    result = subprocess.run(
        ["node", "run.mjs"], cwd=out, capture_output=True, text=True, timeout=300
    )
    if result.returncode != 0:
        pytest.fail(f"node failed:\n{result.stdout}\n{result.stderr}")
    return out, json.loads((out / "result.json").read_text()), reference


@pytest.fixture(scope="module")
def kernel(tmp_path_factory):
    return _emit_and_run(tmp_path_factory, "double-wishbone")


@pytest.fixture(scope="module")
def redundant_kernel(tmp_path_factory):
    return _emit_and_run(tmp_path_factory, "double-wishbone-two-leg")


@pytest.fixture(scope="module")
def compliant_kernel(tmp_path_factory):
    return _emit_and_run(tmp_path_factory, "double-wishbone-compliant")


DYNAMICS_SCRIPT = """
import { readFile, writeFile } from 'node:fs/promises';
import kernel from './kernel.js';

const ref = JSON.parse(await readFile('./reference.json'));
await kernel.ready();
const status = kernel.setGeometry(ref);
kernel.design();
kernel.setDrive(0);
kernel.setVelocity(new Array(kernel.manifest.layout.sizes.q).fill(0));
const resting = kernel.accelerations();
const worst = kernel.integrate(ref.steps);
// `design()` has to restore the whole pose, not just the rotations: a page that integrates and
// then re-runs must not start from wherever the last run stopped.
kernel.design();
const redesign = kernel.pose();
// And the manifest's own default geometry has to be something the kernel accepts.
const defaultsStatus = kernel.setGeometry(kernel.manifest.default_geometry);
await writeFile('./dynamics.json', JSON.stringify({
  status,
  acceleration: resting.acceleration,
  accelerationResidual: resting.residual,
  constraint: worst,
  trajectory: kernel.trajectory(ref.steps).map((q) => Array.from(q)),
  redesign,
  defaultsStatus,
  velocities: kernel.velocities(ref.steps).map((v) => Array.from(v)),
}));
"""


@pytest.fixture(scope="module")
def dynamics_kernel(tmp_path_factory):
    """The quarter car: a trajectory, from the kernel, against Python's own integration."""
    from otkd.problem import dynamics
    from otkd.problem.compile import Problem

    root = tmp_path_factory.mktemp("wasm-dynamics")
    model = otkd.load(EXAMPLES / "quarter-car")
    out = root / "kernel"
    written = emit_wasm(model, out)
    if written["wasm"] is None:
        pytest.skip(written["note"] or "no wasm toolchain")
    if not _node_available():
        pytest.skip("node is not installed: cannot run the module")
    problem = Problem(model)
    manifest = json.loads((out / "kernel.json").read_text())
    (out / "reference.json").write_text(
        json.dumps(
            {
                "points": model.geometry.points,
                "axes": model.geometry.axes,
                "numbers": manifest["default_numbers"],
                "steps": manifest["integration"]["steps"],
            }
        )
    )
    (out / "run.mjs").write_text(DYNAMICS_SCRIPT)
    result = subprocess.run(["node", "run.mjs"], cwd=out, capture_output=True, text=True, timeout=600)
    if result.returncode != 0:
        pytest.fail(f"node failed:\n{result.stdout}\n{result.stderr}")
    return out, json.loads((out / "dynamics.json").read_text()), problem, manifest


def test_design_restores_the_whole_pose(dynamics_kernel):
    """A second run must start where `design()` says, not where the last one stopped."""
    _, result, problem, _ = dynamics_kernel
    assert np.allclose(np.array(result["redesign"]), problem.q0())


def test_the_kernel_accepts_its_own_default_geometry(dynamics_kernel):
    """`default_geometry` is what a page changes one number in, so it has to be usable as it is."""
    _, result, _, _ = dynamics_kernel
    assert result["defaultsStatus"] == 0


def test_the_kernel_writes_a_dynamics_solver(dynamics_kernel):
    _, _, _, manifest = dynamics_kernel
    assert manifest["solver"] == "dynamics"
    assert manifest["integration"]["steps"] == 101
    assert manifest["layout"]["sizes"]["traj"] == 101 * manifest["layout"]["sizes"]["q"]


def test_the_kernel_reproduces_pythons_acceleration(dynamics_kernel):
    from otkd.problem import dynamics

    _, result, problem, _ = dynamics_kernel
    reference = dynamics.accelerations(problem, problem.q0(), np.zeros(problem.n))
    assert result["status"] == 0
    scale = np.max(np.abs(reference.qddot))
    assert np.max(np.abs(np.array(result["acceleration"]) - reference.qddot)) < 1e-6 * scale


def test_the_kernel_reproduces_pythons_trajectory(dynamics_kernel):
    from otkd.problem import dynamics

    _, result, problem, manifest = dynamics_kernel
    trajectory = dynamics.integrate(
        problem,
        problem.q0(),
        np.zeros(problem.n),
        duration=manifest["integration"]["duration"],
        steps=manifest["integration"]["steps"],
    )
    states = np.array(result["trajectory"])
    assert result["constraint"] < 1e-9
    for step in (1, 10, 50, 100):
        assert np.max(np.abs(states[step] - trajectory.q[step])) < 1e-6, step


def test_the_kernel_trajectory_conserves_energy(dynamics_kernel):
    """The check that caught the missing velocity-product term, run on the kernel's own states."""
    from otkd.problem import dynamics

    _, result, problem, _ = dynamics_kernel
    states = np.array(result["trajectory"])
    speeds = np.array(result["velocities"])
    energies = np.array([dynamics.energy(problem, q, v, None)[2] for q, v in zip(states, speeds)])
    assert np.max(np.abs(energies - energies[0])) / abs(energies[0]) < 1e-6


LOADED_SCRIPT = """
import { readFile, writeFile } from 'node:fs/promises';
import kernel from './kernel.js';
const ref = JSON.parse(await readFile('./reference.json'));
await kernel.ready();
const out = {};
for (const [name, load] of Object.entries(ref.loads)) {
  kernel.setGeometry({ points: ref.points, axes: ref.axes, numbers: ref.numbers });
  // The kernel starts from the model's own loads — zero here — and this is the override a page
  // would use: three numbers per load, in the model's order.
  kernel.setLoads(load);
  kernel.design();
  kernel.setVelocity(new Array(kernel.manifest.layout.sizes.q).fill(0));
  const worst = kernel.integrate(ref.steps);
  const heave = kernel.trajectory(ref.steps).map((q) => q[ref.start + 2]);
  out[name] = { worst, peak: Math.min(...heave) };
}
await writeFile('./loaded.json', JSON.stringify(out));
"""


@pytest.fixture(scope="module")
def loaded_kernel(tmp_path_factory):
    """The shipped quarter car, whose study declares the corner's load at zero.

    So the model's own numbers are gravity's alone — and the kernel still carries the load slots,
    which is what lets a page put the corner under load.
    """
    from otkd.problem.compile import Problem

    root = tmp_path_factory.mktemp("wasm-loaded")
    model = otkd.load(build(root, "quarter-car", steps=41))
    problem = Problem(model)
    out = root / "kernel"
    written = emit_wasm(model, out)
    if written["wasm"] is None:
        pytest.skip(written["note"] or "no wasm toolchain")
    if not _node_available():
        pytest.skip("node is not installed: cannot run the module")
    manifest = json.loads((out / "kernel.json").read_text())
    assert len(manifest["loads"]) == 2
    assert all(force == [0.0, 0.0, 0.0] for force in (load["force"] for load in manifest["loads"]))
    (out / "reference.json").write_text(
        json.dumps(
            {
                "points": model.geometry.points,
                "axes": model.geometry.axes,
                "numbers": manifest["default_numbers"],
                "steps": manifest["integration"]["steps"],
                "start": problem._qstart["chassis"],
                # Zero is the model; a corner under load is the page's choice.
                # Three numbers per load, in the model's own load order: the model is zero.
                "loads": {
                    "default": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "3 kN sideways": [3000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "-3.5 kN down": [0.0, 0.0, -3500.0, 0.0, 0.0, 0.0],
                },
            }
        )
    )
    (out / "run.mjs").write_text(LOADED_SCRIPT)
    result = subprocess.run(["node", "run.mjs"], cwd=out, capture_output=True, text=True, timeout=600)
    if result.returncode != 0:
        pytest.fail(f"node failed:\n{result.stdout}\n{result.stderr}")
    return json.loads((out / "loaded.json").read_text()), problem, manifest


LOAD_CASES = {
    "default": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "3 kN sideways": [3000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "-3.5 kN down": [0.0, 0.0, -3500.0, 0.0, 0.0, 0.0],
}


@pytest.mark.parametrize("case", list(LOAD_CASES))
def test_the_kernel_takes_its_loads_at_run_time(loaded_kernel, case):
    """A load's direction is the model's; its magnitude is a number a page sets."""
    from otkd.problem import dynamics

    results, problem, manifest = loaded_kernel
    values = LOAD_CASES[case]
    loads = [
        (
            problem.resolved.point_body[problem.topology.sites[load.site].origin],
            problem.topology.sites[load.site].origin,
            np.array(values[3 * index : 3 * index + 3], dtype=float),
        )
        for index, load in enumerate(problem.model.study.loads)
    ]
    trajectory = dynamics.integrate(
        problem,
        problem.q0(),
        np.zeros(problem.n),
        duration=manifest["integration"]["duration"],
        steps=manifest["integration"]["steps"],
        loads=loads,
    )
    heave = trajectory.q[:, problem._qstart["chassis"] + 2]
    assert results[case]["worst"] < 1e-9
    assert abs(results[case]["peak"] - heave.min()) < 1e-6


@pytest.fixture(scope="module")
def ubj_kernel(tmp_path_factory):
    """A bush at the upper ball joint: three compliant components, three free directions.

    The kernel refused this model while its equilibrium path assumed one free direction, so it is
    the conformance test for the KKT version rather than an extra.
    """
    return _emit_and_run(tmp_path_factory, "double-wishbone-ubj")


def test_the_emitter_writes_a_buildable_kernel(kernel):
    out, result, _ = kernel
    for name in ("kernel.rs", "kernel.wasm", "kernel.js", "kernel.json"):
        assert (out / name).exists(), name
    manifest = json.loads((out / "kernel.json").read_text())
    assert manifest["abi"] == "otkd-kernel-2"
    assert manifest["layout"]["sizes"]["q"] == 21
    assert manifest["rows"][0]["name"] == "lca-ground.tx"


def test_packing_reproduces_the_resolved_geometry(kernel):
    _, result, _ = kernel
    assert result["status"] == 0


def test_the_kernel_reproduces_the_python_jacobian(kernel):
    _, result, reference = kernel
    expected = np.array(reference["design"]["jac"]).reshape(-1)
    assert np.max(np.abs(np.array(result["jac"]) - expected)) < 1e-12


@pytest.mark.parametrize("which", ["kernel", "redundant_kernel", "compliant_kernel", "ubj_kernel"])
def test_the_kernel_reproduces_python_poses(which, request):
    _, result, reference = request.getfixturevalue(which)
    assert len(result["frames"]) == len(reference["frames"])
    for mine, theirs in zip(result["frames"], reference["frames"]):
        assert mine["residual"] < 1e-12
        assert np.max(np.abs(np.array(mine["q"]) - np.array(theirs["q"]))) < 1e-9
        for name, expected in theirs["points"].items():
            assert np.allclose(mine["points"][name], expected, atol=1e-9), name


@pytest.mark.parametrize("which", ["kernel", "redundant_kernel", "compliant_kernel", "ubj_kernel"])
def test_the_kernel_reproduces_the_python_metrics(which, request):
    _, result, reference = request.getfixturevalue(which)
    for mine, theirs in zip(result["frames"], reference["frames"]):
        for name in ("camber", "toe", "track"):
            assert abs(mine["metrics"][name] - theirs["metrics"][name]) < 1e-9, (name, theirs["theta"])
        for name in ("contact", "ic", "rc"):
            ours, yours = mine["metrics"][name], theirs["metrics"][name]
            if ours is None or yours is None:
                assert ours is None and yours is None, (name, theirs["theta"])
            else:
                assert np.allclose(ours, yours, atol=1e-9), (name, theirs["theta"])


def test_the_kernel_is_fast_enough_to_interact_with(kernel):
    _, result, reference = kernel
    assert result["per_frame_ms"] < 5.0  # measured ~0.09 ms; the bound is for slow CI


def test_the_source_is_reproducible(tmp_path):
    model = otkd.load(build(tmp_path, "double-wishbone"))
    first, second = tmp_path / "a", tmp_path / "b"
    emit_wasm(model, first, build=False)
    emit_wasm(model, second, build=False)
    for name in ("kernel.rs", "kernel.json"):
        assert (first / name).read_text() == (second / name).read_text(), name
    assert "pub extern \"C\" fn solve" in (first / "kernel.rs").read_text()


def test_the_redundant_kernel_matches_least_squares(redundant_kernel):
    """A redundant model gets Levenberg–Marquardt, and it agrees with Python's least squares."""
    _, result, reference = redundant_kernel
    assert result["status"] == 0
    for mine, theirs in zip(result["frames"], reference["frames"]):
        assert mine["residual"] < 1e-12
        assert np.max(np.abs(np.array(mine["q"]) - np.array(theirs["q"]))) < 1e-9
        for name in ("camber", "toe", "track"):
            assert abs(mine["metrics"][name] - theirs["metrics"][name]) < 1e-9, name


def test_the_compliant_kernel_matches_the_equilibrium(compliant_kernel):
    """The equilibrium is solved in the kernel, with the load case baked in."""
    out, result, _ = compliant_kernel
    manifest = json.loads((out / "kernel.json").read_text())
    assert manifest["solver"] == "equilibrium"
    reference = json.loads((out / "reference.json").read_text())
    assert result["status"] == 0
    for mine, theirs in zip(result["frames"], reference["frames"]):
        assert mine["residual"] < 1e-8
        assert np.max(np.abs(np.array(mine["q"]) - np.array(theirs["q"]))) < 1e-8
        for name in ("camber", "toe", "track"):
            assert abs(mine["metrics"][name] - theirs["metrics"][name]) < 1e-7, name


def test_an_under_determined_model_is_refused(tmp_path):
    model = otkd.load(build(tmp_path, "double-wishbone"))
    model.topology.actuators = []
    with pytest.raises(ValueError, match="under-determined"):
        emit_wasm(model, tmp_path / "kernel", build=False)


def test_the_vendor_bundle_is_self_describing(tmp_path):
    model = otkd.load(build(tmp_path, "double-wishbone"))
    out = tmp_path / "kernel"
    emit_wasm(model, out, build=False)
    provenance = json.loads((out / "provenance.json").read_text())
    assert provenance["model"] == "dw-1dof"
    assert set(provenance["sources"]) == {
        "topology.yaml", "geometry.yaml", "study.yaml", "constraints.yaml"
    }
    for name in provenance["sources"]:
        assert (out / "source" / name).exists(), name


def test_the_release_bundle_is_reproducible(tmp_path):
    """The zip is how a second repository vendors this: it must not drift between builds."""
    import subprocess
    import sys

    script = Path(__file__).resolve().parents[1] / "scripts" / "build_release.py"
    version = otkd.__version__

    def build(out: Path) -> bytes:
        subprocess.run(
            [sys.executable, str(script), "--only", "double-wishbone", "--no-wasm",
             "--version", version, "--out", str(out)],
            check=True,
            capture_output=True,
        )
        return (out / f"otkd-{version}.zip").read_bytes()

    assert build(tmp_path / "a") == build(tmp_path / "b")
