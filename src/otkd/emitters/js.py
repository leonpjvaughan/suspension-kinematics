"""The ES module the book imports.

One serializer, two wrappers: `otkd emit` and `otkd solve --json` produce the same object, so
a terminal and a figure cannot disagree about a number. Everything is rounded *after* the
agreement is computed, and `meta.generator` comes from the installed version rather than the
clock, so a rerun is byte-identical and reviewable in a diff.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import TYPE_CHECKING

from .. import __version__
from ..analysis.outputs import Reference, compute, quantities
from ..solvers.sweep import Sweep

if TYPE_CHECKING:
    from ..problem.compile import Problem

LENGTH_DP = 6
ANGLE_DP = 4
#: The residual trace and the agreement are roundoff-level quantities: their low digits are a
#: property of the BLAS and the scalar library, not of the model. They are reported to
#: significant figures, and the agreement to *one* — it is a bound ("they agree to 1e-13"), not
#: a measurement whose second digit means anything.
RESIDUAL_FIGURES = 3
SINGULAR_FIGURES = 4
AGREEMENT_FIGURES = 1
#: A study axis sweeps a geometry number, and a compliance is 1e-8 while a length is 0.37:
#: decimal places cannot hold both, so an axis value is reported to significant figures.
AXIS_FIGURES = 6
#: A residual below this is the solver's own roundoff: the arithmetic that produced it differs
#: between machines in its last bits, so its digits are not the model's. Reporting it as zero is
#: both honest and the only way the artefact can be byte-identical.
RESIDUAL_FLOOR = 1e-9
#: Significant figures for the residual trace of an equilibrium solve. Every frame of a sweep is
#: seeded from its predecessor, whose last bits a different LAPACK moves, and a stiff model
#: amplifies that into a few per cent of the *first* iterate's residual — which is a digit at three
#: figures and none at one. A kinematic solve is a root-find from the design pose with no such
#: history, so it keeps three.
EQUILIBRIUM_RESIDUAL_FIGURES = 1
#: Decimal places per quantity: angles to four, lengths to six. One table, because a frame entry
#: and a trajectory sample are the same numbers and must round the same way.
QUANTITY_PLACES = {
    "camber": ANGLE_DP,
    "toe": ANGLE_DP,
    "ic": LENGTH_DP,
    "rc": LENGTH_DP,
    "track": LENGTH_DP,
    "contact": LENGTH_DP,
}


def _r(value: float, dp: int) -> float:
    rounded = round(float(value), dp)
    return 0.0 if rounded == 0 else rounded


def _sig(value: float, figures: int, *, floor: float = 0.0) -> float:
    """Round to significant figures, which is what a roundoff-level number deserves.

    `floor` reports anything below it as zero: a residual of 1e-20 m is zero, and printing it
    would only expose the platform's rounding.
    """
    number = float(value)
    if number == 0.0 or math.isnan(number) or math.isinf(number):
        return 0.0 if number == 0.0 else number
    if abs(number) < floor:
        return 0.0
    return float(f"{number:.{figures - 1}e}")


def _point(point, dp: int = LENGTH_DP) -> list[float]:
    return [_r(component, dp) for component in point]


def _rounded(values: dict[str, object]) -> dict[str, object]:
    """Round a quantity mapping to the places its unit deserves, keeping its keys and order."""
    rounded: dict[str, object] = {}
    for name, value in values.items():
        places = QUANTITY_PLACES[name]
        if value is None:
            rounded[name] = None
        elif isinstance(value, list):
            rounded[name] = [_r(component, places) for component in value]
        else:
            rounded[name] = _r(float(value), places)
    return rounded


def payload(sweep: Sweep) -> dict:
    study = sweep.model.study
    travel_start = _r(study.drivers[0].law.grid()[0], LENGTH_DP) if study and study.drivers else 0.0
    travel_stop = _r(study.drivers[0].law.grid()[-1], LENGTH_DP) if study and study.drivers else 0.0

    frames = []
    for frame in sweep.frames:
        out = frame.outputs
        entry = {
            "theta": _r(frame.values[sweep.primary], LENGTH_DP),
            "lbj": _xz(frame.points.get("lbj")),
            "ubj": _xz(frame.points.get("ubj")),
            "wc": _xz(frame.points.get("wc")),
            "tro": _xz(frame.points.get("tro")),
            "points": {name: _point(value) for name, value in frame.points.items()},
        }
        if sweep.axis_numbers:
            entry["axis"] = [_sig(value, AXIS_FIGURES) for value in frame.axis]
        entry.update(_rounded(quantities(out)))
        entry["branchChanged"] = bool(frame.branch_changed)
        frames.append(entry)

    representative = min(range(len(sweep.frames)), key=lambda i: abs(sweep.frames[i].values[sweep.primary]))
    iterations = _iterations(sweep, representative)

    agreement = None
    if sweep.analytic is not None:
        agreement = {
            "analytic": [_r(value, ANGLE_DP) for value in sweep.analytic],
            "numeric": [_r(frame.outputs.camber, ANGLE_DP) for frame in sweep.frames],
            "max_abs_diff": _sig(sweep.agreement, AGREEMENT_FIGURES) if sweep.agreement is not None else None,
        }

    return {
        "meta": {
            "model": sweep.model.topology.id,
            "units": {"length": "m", "angle": "deg"},
            "branch": sweep.branch,
            "frames": len(frames),
            "travel": {"start": travel_start, "stop": travel_stop, "unit": sweep.unit},
            # Deliberately without the version: the payload is data, and a version here would
            # change every committed `out.js` on every commit. The version is on the release,
            # in `MANIFEST.json` and `provenance.json`.
            "generator": "otkd",
            "representative_frame": representative,
        },
        "sweep": {
            "input": {
                "name": sweep.primary,
                "unit": sweep.unit,
                "values": [_r(frame.values[sweep.primary], LENGTH_DP) for frame in sweep.frames],
            },
            # Only when the study sweeps one: a payload that says "axes": [] in every model
            # would change every committed export to say nothing.
            **(
                {
                    "axes": [
                        {
                            "numbers": numbers,
                            "values": [_sig(value, AXIS_FIGURES) for value in values],
                        }
                        for numbers, values in zip(sweep.axis_numbers, sweep.axis_values)
                    ]
                }
                if sweep.axis_numbers
                else {}
            ),
            "frames": frames,
        },
        "iterations": iterations,
        "agreement": agreement,
    }


MAX_ITERATIONS = 60


def sample(
    problem: "Problem",
    q,
    reference: Reference,
    *,
    time: float,
    energy: float,
    constraint: float,
) -> dict:
    """One step of a trajectory, shaped exactly like a frame entry.

    A trajectory has no driven value (`theta`) and no branch history that can change, so those
    two sweep concepts are absent. Everything else is the same: the same quantity names, the same
    rounding, the same world points, computed by the same code that fills a frame — so a page that
    draws a sweep can draw a trajectory without a second reading of the model.
    """
    points = problem.world_points(q)
    entry: dict = {
        "t": _r(time, LENGTH_DP),
        "energy": _r(energy, LENGTH_DP),
        "constraint": float(constraint),
        "lbj": _xz(points.get("lbj")),
        "ubj": _xz(points.get("ubj")),
        "wc": _xz(points.get("wc")),
        "tro": _xz(points.get("tro")),
        "points": {name: _point(value) for name, value in points.items()},
    }
    entry.update(_rounded(quantities(compute(problem, q, reference))))
    return entry


def _iterations(sweep: Sweep, index: int) -> list[dict]:
    from ..problem.compile import DRIVER, HARD, NORM

    frame = sweep.frames[index]
    rows = [i for i, row in enumerate(sweep.problem.rows) if row.group in (HARD, NORM, DRIVER)]
    figures = EQUILIBRIUM_RESIDUAL_FIGURES if sweep.problem.has_springs else RESIDUAL_FIGURES
    history = frame.history
    if len(history) > MAX_ITERATIONS:
        stride = (len(history) - 1) / (MAX_ITERATIONS - 1)
        history = [history[min(len(history) - 1, round(k * stride))] for k in range(MAX_ITERATIONS)]
    out = []
    for k, (q, residual) in enumerate(history):
        try:
            singular = sweep.problem.jacobian(q, sweep.problem.numbers(), sweep.problem.theta0())
            values = [float(v) for v in __import__("numpy").linalg.svd(singular[rows], compute_uv=False)]
        except Exception:  # pragma: no cover - a history entry must never break the export
            values = []
        out.append(
            {
                "k": k,
                "residual": [_sig(v, figures, floor=RESIDUAL_FLOOR) for v in residual],
                "singular_values": [_sig(v, SINGULAR_FIGURES) for v in values],
            }
        )
    return out


def _xz(point) -> list[float] | None:
    return None if point is None else [_r(point[0], LENGTH_DP), _r(point[2], LENGTH_DP)]


def _object(name: str, head: dict, item: str, entries: list) -> str:
    """`export const <name> = {head..., item: [ one entry per line ]};`

    Frames and iterates go one per line. The file is generated and diffed for determinism,
    not read by a human, and indenting every number quadruples it."""
    lines = [f"export const {name} = {{"]
    for key, value in head.items():
        lines.append(f'  "{key}": {json.dumps(value, allow_nan=False)},')
    lines.append(f'  "{item}": [')
    lines.append(",\n".join("    " + json.dumps(entry, allow_nan=False) for entry in entries))
    lines.append("  ]")
    lines.append("};")
    return "\n".join(lines) + "\n"


def _array(name: str, entries: list) -> str:
    lines = [f"export const {name} = ["]
    lines.append(",\n".join("  " + json.dumps(entry, allow_nan=False) for entry in entries))
    lines.append("];")
    return "\n".join(lines) + "\n"


def render(payload: dict) -> str:
    parts = ["// Generated by otkd. Do not edit by hand.\n"]
    parts.append(f"export const meta = {json.dumps(payload['meta'], indent=2, allow_nan=False)};\n")
    head = {"input": payload["sweep"]["input"]}
    if "axes" in payload["sweep"]:
        head["axes"] = payload["sweep"]["axes"]
    parts.append(_object("sweep", head, "frames", payload["sweep"]["frames"]))
    parts.append(_array("iterations", payload["iterations"]))
    parts.append(
        f"export const agreement = {json.dumps(payload['agreement'], indent=2, allow_nan=False)};\n"
    )
    return "\n".join(parts)


def emit_js(model, path: str | Path) -> Path:
    from ..solvers.sweep import sweep

    result = sweep(model)
    text = render(payload(result))
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text)
    return target
