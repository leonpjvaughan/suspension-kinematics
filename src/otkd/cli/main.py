"""`otkd` — the command line. One verb per stage of the pipeline."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

from .. import __version__
from ..report import check
from ..config.model import Model
from ..errors import ConfigError, Fault, SolverError


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="otkd", description="Config files in, mathematical problems out.")
    parser.add_argument("--version", action="version", version=f"otkd {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("check", help="parse, resolve, verify the design position and the rank")
    p.add_argument("model")
    p.add_argument("--study")
    p.add_argument("--quiet", action="store_true")

    p = sub.add_parser("compile", help="print the derived problem")
    p.add_argument("model")
    p.add_argument("--study")
    p.add_argument("--format", choices=("text", "latex", "json"), default="text")

    p = sub.add_parser("solve", help="solve the study")
    p.add_argument("model")
    p.add_argument("--study")
    p.add_argument("--at", type=float, help="solve a single frame at this driven value")
    p.add_argument("--json", dest="json_path")
    p.add_argument("--csv", dest="csv_path")
    p.add_argument("--quiet", action="store_true")

    p = sub.add_parser("emit", help="write the artefact")
    p.add_argument("model")
    p.add_argument("out")
    p.add_argument("--study")
    p.add_argument("--format", choices=("js", "json", "wasm"), default="js")
    p.add_argument("--quiet", action="store_true")

    p = sub.add_parser("kinds", help="the kinds this model's library defines")
    p.add_argument("model")

    args = parser.parse_args(argv)
    try:
        return _dispatch(args)
    except ConfigError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except SolverError as exc:
        print(f"solver: {exc}", file=sys.stderr)
        return 2


def _dispatch(args) -> int:
    model = load_model(args) if args.command != "kinds" else load_model(args)
    if args.command == "check":
        report = check(model)
        if not args.quiet:
            print(report.summary())
        for warning in report.warnings:
            print(f"warning: {warning}", file=sys.stderr)
        return 0
    if args.command == "kinds":
        _print_kinds(model)
        return 0
    if args.command == "compile":
        _print_problem(model, args.format)
        return 0
    if args.command == "solve":
        return _solve(model, args)
    if args.command == "emit":
        if model.study is not None and model.study.analysis == "dynamics":
            print(
                "a dynamics study produces a trajectory, not a sweep; write it with `otkd solve --json`",
                file=sys.stderr,
            )
            return 1
        if args.format == "wasm":
            from ..emitters.wasm import emit_wasm

            try:
                written = emit_wasm(model, args.out)
            except ValueError as exc:
                print(str(exc), file=sys.stderr)
                return 1
            if not args.quiet:
                print(f"wrote {written['dir']} ({'kernel.wasm' if written['wasm'] else 'no wasm'})")
                if written["note"]:
                    print(written["note"], file=sys.stderr)
            return 0
        from ..emitters.js import payload, render
        from ..solvers.sweep import sweep

        result = sweep(model)
        text = render(payload(result)) if args.format == "js" else json.dumps(payload(result), indent=2)
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text)
        if not args.quiet:
            print(f"wrote {args.out} ({len(result.frames)} frames)")
        return 0
    raise AssertionError(args.command)


def load_model(args) -> Model:
    study = getattr(args, "study", None)
    if getattr(args, "at", None) is not None:
        model = Model.load(args.model, study=study)
        if model.study is None or model.study.analysis == "dynamics" or not model.study.drivers:
            raise ConfigError(
                [Fault(args.model, "--at", "a dynamics study has no driven coordinate to solve at")]
                if model.study is not None and model.study.analysis == "dynamics"
                else []
            )
        driver = model.study.drivers[0]
        driver.law.kind = "constant"
        driver.law.value = args.at
    else:
        model = Model.load(args.model, study=study)
    return model


def _solve(model: Model, args) -> int:
    from ..emitters.js import payload
    from ..solvers.sweep import sweep

    if model.study is not None and model.study.analysis == "dynamics":
        return _solve_dynamics(model, args)

    result = sweep(model)
    data = payload(result)
    if args.json_path:
        Path(args.json_path).write_text(json.dumps(data, indent=2))
    if args.csv_path:
        _write_csv(result, args.csv_path)
    if not args.quiet:
        print(_table(result))
    return 0


def _solve_dynamics(model: Model, args) -> int:
    import numpy as np

    from ..problem import dynamics
    from ..problem.compile import Problem
    from ..solvers.sweep import sweep  # noqa: F401 - keeps the kinematics import local

    problem = Problem(model)
    study = model.study
    assert study is not None
    loads = []
    for load in study.loads:
        site = problem.topology.sites[load.site]
        loads.append(
            (problem.resolved.point_body[site.origin], site.origin, np.asarray(load.force, dtype=float))
        )
    trajectory = dynamics.integrate(
        problem,
        problem.q0(),
        np.zeros(problem.n),
        duration=float(study.duration),
        steps=int(study.steps),
        loads=loads,
    )
    if args.json_path:
        Path(args.json_path).write_text(
            json.dumps(
                {
                    "meta": {
                        "model": model.topology.id,
                        "analysis": "dynamics",
                        "duration": study.duration,
                        "steps": study.steps,
                        "generator": f"otkd {__version__}",
                    },
                    "trajectory": [
                        {"t": _round(float(t), 6), "energy": _round(float(e), 6), "constraint": float(c)}
                        for t, e, c in zip(trajectory.t, trajectory.energy, trajectory.constraint)
                    ],
                },
                indent=2,
            )
        )
    if not args.quiet:
        stride = max(1, len(trajectory.t) // 12)
        print(f"{'t':>8s} {'energy':>12s} {'|phi|':>10s}")
        for t, e, c in zip(trajectory.t[::stride], trajectory.energy[::stride], trajectory.constraint[::stride]):
            print(f"{t:8.4f} {e:12.4f} {c:10.2e}")
    return 0


def _round(value: float, places: int) -> float:
    rounded = round(value, places)
    return 0.0 if rounded == 0 else rounded


def _table(result) -> str:
    lines = [f"{result.primary:10s} {'camber':>9s} {'toe':>9s} {'track':>9s}  ic                rc"]
    step = max(1, len(result.frames) // 12)
    for frame in result.frames[::step]:
        out = frame.outputs
        ic = "null" if out.ic is None else f"[{out.ic[0]:6.3f},{out.ic[1]:6.3f}]"
        rc = "null" if out.rc is None else f"[{out.rc[0]:6.3f},{out.rc[1]:6.3f}]"
        lines.append(
            f"{frame.values[result.primary]:+10.4f} {out.camber:+9.4f} {out.toe:+9.4f} "
            f"{out.track:+9.5f}  {ic:18s} {rc}"
        )
    return "\n".join(lines)


def _write_csv(result, path: str) -> None:
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["theta", "camber", "toe", "track", "ic_x", "ic_z", "rc_x", "rc_z"])
        for frame in result.frames:
            out = frame.outputs
            writer.writerow(
                [
                    frame.values[result.primary],
                    out.camber,
                    out.toe,
                    out.track,
                    *(out.ic or ("", "")),
                    *(out.rc or ("", "")),
                ]
            )


def _latex_names(problem) -> dict:
    """Readable names for the generated symbols.

    The printer would otherwise emit `g_{lcaf 0}`, which is a name for a debugger. These are
    names for a page: design points `P`, site frames `C`, joint lengths `L`, gaps `G`,
    compliances `c`, and the drive `theta`.
    """
    import sympy as sp

    mapping: dict = {}
    for body, start in problem._qstart.items():
        for index, axis in enumerate("xyz"):
            mapping[problem.q[start + index]] = sp.Symbol(rf"p_{{\text{{{body}}},{axis}}}")
        for index, component in enumerate("wxyz"):
            mapping[problem.q[start + 3 + index]] = sp.Symbol(rf"q_{{\text{{{body}}},{component}}}")
    axis_of = "xyz"
    for symbol, key in zip(problem.g, problem._gkey):
        kind = key[0]
        if kind == "point":
            mapping[symbol] = sp.Symbol(rf"P_{{\text{{{key[1]}}},{axis_of[key[2]]}}}")
        elif kind == "site":
            row, column = divmod(key[2], 3)
            mapping[symbol] = sp.Symbol(rf"C_{{\text{{{key[1]}}},{row + 1}{column + 1}}}")
        elif kind == "length":
            mapping[symbol] = sp.Symbol(rf"L_{{\text{{{key[1]}}}}}")
        elif kind == "gap":
            mapping[symbol] = sp.Symbol(rf"G_{{\text{{{key[1]}}}}}")
        elif kind == "compliance":
            mapping[symbol] = sp.Symbol(rf"c_{{\text{{{key[1]}}},\text{{{key[2]}}}}}")
    for drive in problem.drives:
        mapping[drive.theta] = sp.Symbol(rf"\theta_{{\text{{{drive.actuator}}}}}")
    return mapping


def _print_problem(model: Model, fmt: str) -> None:
    import sympy as sp

    from ..problem.compile import Problem

    problem = Problem(model)
    if fmt == "json":
        print(
            json.dumps(
                {
                    "id": model.topology.id,
                    "unknowns": problem.n,
                    "rows": [
                        {"name": row.name, "group": row.group, "unit": row.unit}
                        for row in problem.rows
                    ],
                    "actuators": [
                        {"name": drive.actuator, "joint": drive.joint, "dof": drive.dof, "unit": drive.unit}
                        for drive in problem.drives
                    ],
                },
                indent=2,
            )
        )
        return
    names = _latex_names(problem)
    for row, expr in zip(problem.rows, problem._exprs):
        if fmt == "latex":
            label = row.name.replace(".", r"\,")
            print(f"{label} &: {sp.latex(expr.xreplace(names))} = 0")
        else:
            print(f"{row.name:24s} {row.group:8s} {sp.sstr(expr)}")
    if fmt == "text":
        print(f"\n{problem.n} unknowns; {len(problem.rows)} rows; {len(problem.g)} geometry symbols")


def _print_kinds(model: Model) -> None:
    for name, kind in sorted(model.kinds.items()):
        modes = " ".join(f"{component}={mode}" for component, mode in kind.modes.items())
        print(f"{name:14s} {kind.family:6s} {modes}")


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
