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
