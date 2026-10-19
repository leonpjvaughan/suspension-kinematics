"""Build a model directory in a temporary tree, with a shorter study.

The committed examples run 81 frames because a figure wants them; tests want the same model
with nine, so every helper copies the example and rewrites only the study's step count —
which is the point of the study being a separate file.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import yaml

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"


def build(tmp_path: Path, name: str, *, steps: int = 9) -> Path:
    root = tmp_path / name
    root.mkdir(parents=True, exist_ok=True)
    shutil.copy(EXAMPLES / "constraints.yaml", tmp_path / "constraints.yaml")
    for file in ("topology.yaml", "geometry.yaml"):
        shutil.copy(EXAMPLES / name / file, root / file)
    data = yaml.safe_load((EXAMPLES / name / "study.yaml").read_text())
    for driver in data.get("drivers", []):
        law = driver.get("law", {})
        if law.get("kind") == "linear":
            law["steps"] = steps
    (root / "study.yaml").write_text(yaml.safe_dump(data, sort_keys=False))
    return root


#: Plausible mass properties for the corner, used by the dynamics tests and the CLI.
DYNAMICS_MASSES = {
    "lca": (3.2, [0.500, 0.0, 0.150], [0.021, 0.140, 0.131, 0.0, 0.002, 0.0]),
    "uca": (1.8, [0.530, 0.0, 0.500], [0.008, 0.043, 0.041, 0.0, 0.0, 0.0]),
    "up": (12.0, [0.730, 0.0, 0.310], [0.310, 0.180, 0.310, 0.0, 0.0, 0.0]),
}


def add_dynamics(root: Path, *, compliance: float = 1e-4) -> Path:
    """Give a compliant corner mass properties, which the kinematic examples do not carry.

    The compliance is softer than the kinematic example's: the energy test is about the
    integrator, and a 1 kN/mm spring oscillates at ~290 rad/s, which RK4 resolves only with a much
    smaller step than the test wants to take.
    """
    topology = yaml.safe_load((root / "topology.yaml").read_text())
    for body in DYNAMICS_MASSES:
        topology["bodies"][body].setdefault("points", []).append(f"com-{body}")
    (root / "topology.yaml").write_text(yaml.safe_dump(topology, sort_keys=False))

    geometry = yaml.safe_load((root / "geometry.yaml").read_text())
    geometry.setdefault("joints", {}).setdefault("tie-rod", {}).setdefault("compliance", {})
    geometry["joints"]["tie-rod"]["compliance"]["length"] = compliance
    for body, (_, com, _) in DYNAMICS_MASSES.items():
        geometry["points"][f"com-{body}"] = com
    geometry["dynamics"] = {
        "gravity": [0.0, 0.0, -9.81],
        "bodies": {
            body: {"mass": mass, "com": f"com-{body}", "inertia": inertia}
            for body, (mass, _, inertia) in DYNAMICS_MASSES.items()
        },
    }
    (root / "geometry.yaml").write_text(yaml.safe_dump(geometry, sort_keys=False))
    return root
