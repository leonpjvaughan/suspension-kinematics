#!/usr/bin/env python3
"""Run a dynamics model's compiled kernel and print the trajectory.

The browser kernel is the engine: the same derivation as the Python one, factored into a
straight-line program, printed as Rust and compiled to WebAssembly. This script builds it for a
model, runs it in node, and prints what it found — the trajectory's extremes, the energy drift
recomputed with the Python engine's own energy, and the constraint violation the kernel reports.

    uv run python scripts/run_kernel.py examples/quarter-car

It doubles as the reproduction recipe for a page: the numbers it prints are the numbers a browser
gets, because it is the browser's module.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402

import otkd  # noqa: E402
from otkd.problem import dynamics  # noqa: E402
from otkd.problem.compile import Problem  # noqa: E402

HARNESS = """
import { readFile, writeFile } from 'node:fs/promises';
import kernel from './kernel.js';
const ref = JSON.parse(await readFile('./reference.json'));
await kernel.ready();
const status = kernel.setGeometry(ref);
kernel.design();
kernel.setDrive(0);
kernel.setVelocity(new Array(kernel.manifest.layout.sizes.q).fill(0));
const worst = kernel.integrate(ref.steps);
await writeFile('./out.json', JSON.stringify({
  status, worst,
  trajectory: kernel.trajectory(ref.steps).map((q) => Array.from(q)),
  velocities: kernel.velocities(ref.steps).map((v) => Array.from(v)),
}));
"""


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a dynamics model's compiled kernel.")
    parser.add_argument("model", nargs="?", default="examples/quarter-car")
    parser.add_argument("--json", help="write the trajectory here")
    args = parser.parse_args()

    model = otkd.load(args.model)
    if model.study is None or model.study.analysis != "dynamics":
        print(f"{args.model} is not a dynamics study", file=sys.stderr)
        return 1
    problem = Problem(model)

    with tempfile.TemporaryDirectory() as work:
        target = pathlib.Path(work) / "kernel"
        from otkd.emitters.wasm import emit_wasm

        written = emit_wasm(model, target)
        if written["wasm"] is None:
            print(f"no kernel: {written['note']}", file=sys.stderr)
            return 1
        manifest = json.loads((target / "kernel.json").read_text())
        steps = manifest["integration"]["steps"]
        (target / "reference.json").write_text(
            json.dumps(
                {
                    "points": model.geometry.points,
                    "axes": model.geometry.axes,
                    "numbers": manifest["default_numbers"],
                    "steps": steps,
                }
            )
        )
        (target / "run.mjs").write_text(HARNESS)
        result = subprocess.run(["node", "run.mjs"], cwd=target, capture_output=True, text=True)
        if result.returncode != 0:
            print(result.stdout, result.stderr, file=sys.stderr)
            return 1
        data = json.loads((target / "out.json").read_text())

    states = np.array(data["trajectory"])
    speeds = np.array(data["velocities"])
    energies = np.array([dynamics.energy(problem, q, v, None)[2] for q, v in zip(states, speeds)])
    reference = dynamics.integrate(
        problem, problem.q0(), np.zeros(problem.n),
        duration=manifest["integration"]["duration"], steps=steps,
    )
    heave = states[:, problem._qstart["chassis"] + 2]

    print(f"{model.topology.id}: {steps} steps over {manifest['integration']['duration']} s, "
          f"dt = {manifest['integration']['dt']:.4f} s")
    print(f"  kernel      status {data['status']}, constraint {data['worst']:.2e}")
    print(f"  energy      drift {np.ptp(energies) / abs(energies[0]):.2e} of {energies[0]:.3f} J")
    print(f"  heave       {heave.min() * 1000:+.1f} .. {heave.max() * 1000:+.1f} mm")
    print(f"  against Python's own integration: max |q| difference "
          f"{np.max(np.abs(states - reference.q)):.2e}")

    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(data, indent=2))
        print(f"  wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
