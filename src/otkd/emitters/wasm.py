"""Generate a Rust/WASM kernel from the same derivation.

Nothing here re-derives anything. It takes the compiled `Problem`, runs common-subexpression
elimination over the residual and the Jacobian, and prints the resulting straight-line program
as Rust. The Rust is an *artefact* of the derivation, exactly as `out.js` is an artefact of a
sweep: if the two ever disagree, the derivation is right and the artefact is stale.

The module is deliberately small and fixed-size: one static `f64` buffer, no allocator, no
libc, no `wasm-bindgen`. See docs/wasm.md.
"""

from __future__ import annotations

import base64
import hashlib
import json
import math
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import sympy as sp
from sympy.printing import rust_code
from sympy.simplify.cse_main import cse

from ..config.model import Model
from ..problem.compile import Problem

TARGET = "wasm32-unknown-unknown"


@dataclass(frozen=True)
class Layout:
    sizes: dict[str, int]
    offsets: dict[str, int]
    total: int

    def as_dict(self) -> dict:
        return {"sizes": self.sizes, "offsets": self.offsets, "slots": self.total}


def emit_wasm(model: Model, out_dir: str | Path, *, build: bool = True) -> dict:
    """Write the kernel and everything needed to audit it; build and inline if asked.

    The output is a *vendor bundle*: `kernel.js` is all a page imports, `kernel.json` describes
    the ABI, and `provenance.json` plus `source/` record exactly which four files produced it.
    """
    problem = Problem(model)
    kind = _supported_kind(problem)
    target = Path(out_dir)
    target.mkdir(parents=True, exist_ok=True)

    points = sorted(model.geometry.points)
    axes = sorted(model.geometry.axes)
    numbers = [key for key in problem._gkey if key[0] in ("length", "gap", "compliance")]

    layout = Layout(
        sizes={
            "q": problem.n,
            "g": len(problem.g),
            "t": len(problem.drives),
            "out": len(problem.rows),
            "jac": problem.n * len(problem.rows),
            "met": 10,
            "world": 3 * len(points),
            "pts": 3 * len(points),
            "axs": 3 * len(axes),
            "num": len(numbers),
        },
        offsets={},
        total=0,
    )
    offset = 0
    offsets = {}
    for key, size in layout.sizes.items():
        offsets[key] = offset
        offset += size
    layout = Layout(sizes=layout.sizes, offsets=offsets, total=offset)

    source = _rust_source(problem, model, layout, points, axes, numbers, kind)
    (target / "kernel.rs").write_text(source)

    wasm_path = None
    note = ""
    if build:
        wasm_path, note = _build(target)

    manifest = {
        "model": model.topology.id,
        "abi": "otkd-kernel-2",
        "solver": kind,
        "layout": layout.as_dict(),
        "metrics": ["camber", "toe", "track", "contact", "ic", "rc"],
        "points": points,
        "axes": axes,
        "numbers": [_number_descriptor(key) for key in numbers],
        # The design values for those slots, so a page does not have to reconstruct them.
        # `null` means "derive it": a missing tie-rod length defaults to the design separation.
        "default_numbers": _default_numbers(problem, numbers),
        "rows": [{"name": row.name, "unit": row.unit, "group": row.group} for row in problem.rows],
        "drive": [drive.actuator for drive in problem.drives],
        "wasm": wasm_path.name if wasm_path else None,
        "note": note,
    }
    (target / "kernel.json").write_text(json.dumps(manifest, indent=2) + "\n")

    module = ""
    if wasm_path is not None:
        module = base64.b64encode(wasm_path.read_bytes()).decode("ascii")
    (target / "kernel.js").write_text(_js_glue(manifest, module))
    _write_provenance(target, model, layout, wasm_path)
    copy_sources(target, model)
    return {"dir": str(target), "wasm": str(wasm_path) if wasm_path else None, "note": note}


def _supported_kind(problem: Problem) -> str:
    """The solver the kernel will carry, or refuse.

    Two are compiled: damped Newton for a square system, and Levenberg–Marquardt for a redundant
    one. A compliant model needs the equilibrium root-find, which is not compiled yet; emitting a
    kernel that solved the wrong equations would be worse than refusing. See docs/wasm.md §7.
    """
    from ..solvers.system import System

    if problem.model.study is not None and problem.model.study.analysis == "dynamics":
        raise ValueError(
            "a dynamics study has no compiled solver: the kernel solves kinematics and equilibrium, "
            "not a trajectory"
        )
    kind = System(problem, problem.numbers(), problem.theta0()).classify()
    if kind in ("square", "redundant", "equilibrium"):
        return kind
    hint = {
        "equilibrium": "a compliant model needs the equilibrium root-find, which is not compiled yet",
        "under": "the model is under-determined, so there is nothing to solve",
    }[kind]
    raise ValueError(f"the WASM kernel cannot solve this model yet: it is {kind!r}; {hint}")


#: The config files a model is built from, in the order a bundle lists them.
SOURCE_KEYS = ("topology", "geometry", "study", "constraints")


def sources(model: Model) -> dict[str, str]:
    """The four config files, by name, with the digest of their bytes.

    The library is the one a consumer cannot infer: it lives outside the model directory, so a
    release that changed what `spring` means would change the model without changing a model file.
    """
    digests = {}
    for key in SOURCE_KEYS:
        path = model.files.get(key)
        if path and Path(path).exists():
            digests[Path(path).name] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    return digests


def copy_sources(target: Path, model: Model) -> dict[str, str]:
    """Copy the four config files into `target/source/` and return their digests.

    An artefact with no inputs cannot be audited. For a dynamics model the sources are the *whole*
    of its provenance — it ships no payload and no kernel — which is why this is not the kernel's
    private business.
    """
    source = target / "source"
    source.mkdir(parents=True, exist_ok=True)
    for key in SOURCE_KEYS:
        path = model.files.get(key)
        if path and Path(path).exists():
            shutil.copy(path, source / Path(path).name)
    return sources(model)


def _write_provenance(target: Path, model: Model, layout: Layout, wasm_path: Path | None) -> None:
    from .. import __version__

    rustc = shutil.which("rustc")
    version = subprocess.run([rustc, "--version"], capture_output=True, text=True).stdout.strip() if rustc else ""
    sources_ = sources(model)
    provenance = {
        "model": model.topology.id,
        "generator": f"otkd {__version__}",
        "rustc": version,
        "target": TARGET,
        "abi": "otkd-kernel-2",
        "layout": layout.as_dict(),
        "sources": sources_,
        "wasm_sha256": hashlib.sha256(wasm_path.read_bytes()).hexdigest() if wasm_path else None,
    }
    (target / "provenance.json").write_text(json.dumps(provenance, indent=2, sort_keys=True) + "\n")


# -- Rust source -----------------------------------------------------------------------


def _rust_source(problem: Problem, model: Model, layout: Layout, points, axes, numbers, kind: str) -> str:
    n = problem.n
    rows = len(problem.rows)
    size = layout.sizes
    off = layout.offsets

    lines: list[str] = [
        "// Generated by otkd from the model's own derivation. Do not edit by hand.",
        f"// model: {model.topology.id}",
        "#![allow(clippy::all)]",
        "",
        f"const N: usize = {n};",
        f"const M: usize = {rows};",
        f"const G: usize = {size['g']};",
        f"const A: usize = {size['t']};",
        f"const P: usize = {len(points)};",
        "",
    ]
    for key in ("q", "g", "t", "out", "jac", "met", "world", "pts", "axs", "num"):
        lines.append(f"const O_{key.upper()}: usize = {off[key]};")
    lines += [
        f"const TOTAL: usize = {layout.total};",
        "",
        "#[no_mangle]",
        "pub static mut MEM: [f64; TOTAL] = [0.0; TOTAL];",
        "",
        "#[inline] fn sub3(a: [f64;3], b: [f64;3]) -> [f64;3] { [a[0]-b[0], a[1]-b[1], a[2]-b[2]] }",
        "#[inline] fn dot3(a: [f64;3], b: [f64;3]) -> f64 { a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }",
        "#[inline] fn cross3(a: [f64;3], b: [f64;3]) -> [f64;3] {",
        "    [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]",
        "}",
        "#[inline] fn norm3(a: [f64;3]) -> f64 { dot3(a, a).sqrt() }",
        "#[inline] fn norm2(a: [f64;2]) -> f64 { (a[0]*a[0] + a[1]*a[1]).sqrt() }",
        "",
        "/// Intersection of two front-view lines; NaN when they are parallel.",
        "#[inline]",
        "fn intersect2(a: [f64;2], u: [f64;2], c: [f64;2], v: [f64;2]) -> [f64;2] {",
        "    let cross = u[0]*v[1] - u[1]*v[0];",
        "    let scale = norm2(u) * norm2(v);",
        "    if scale == 0.0 || cross.abs() < 1e-12 * scale { return [f64::NAN, f64::NAN]; }",
        "    let t = ((c[0]-a[0])*v[1] - (c[1]-a[1])*v[0]) / cross;",
        "    [a[0] + t*u[0], a[1] + t*u[1]]",
        "}",
        "",
        "/// Complete a site's declared axes into a right-handed orthonormal frame, z last.",
        "#[inline]",
        "fn frame(z: [f64;3], x: Option<[f64;3]>, y: Option<[f64;3]>) -> Option<[f64;9]> {",
        "    let zn = norm3(z);",
        "    if zn == 0.0 { return None; }",
        "    let zz = [z[0]/zn, z[1]/zn, z[2]/zn];",
        "    let xx = if let Some(v) = x {",
        "        let p = sub3(v, [dot3(v, zz)*zz[0], dot3(v, zz)*zz[1], dot3(v, zz)*zz[2]]);",
        "        let pn = norm3(p);",
        "        if pn == 0.0 { return None; }",
        "        [p[0]/pn, p[1]/pn, p[2]/pn]",
        "    } else if let Some(v) = y {",
        "        let p = sub3(v, [dot3(v, zz)*zz[0], dot3(v, zz)*zz[1], dot3(v, zz)*zz[2]]);",
        "        let pn = norm3(p);",
        "        if pn == 0.0 { return None; }",
        "        cross3([p[0]/pn, p[1]/pn, p[2]/pn], zz)",
        "    } else {",
        "        let helper = if zz[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };",
        "        let p = sub3(helper, [dot3(helper, zz)*zz[0], dot3(helper, zz)*zz[1], dot3(helper, zz)*zz[2]]);",
        "        let pn = norm3(p);",
        "        [p[0]/pn, p[1]/pn, p[2]/pn]",
        "    };",
        "    let yy = cross3(zz, xx);",
        "    Some([xx[0], yy[0], zz[0], xx[1], yy[1], zz[1], xx[2], yy[2], zz[2]])",
        "}",
        "",
        "#[inline]",
        "fn at(mem: &[f64], base: usize, index: usize, k: usize) -> f64 { mem[base + 3*index + k] }",
        "#[inline]",
        "fn point(mem: &[f64], index: usize) -> [f64;3] {",
        "    [mem[O_PTS + 3*index], mem[O_PTS + 3*index + 1], mem[O_PTS + 3*index + 2]]",
        "}",
        "",
    ]

    # design pose
    q0 = problem.q0()
    lines += ["#[no_mangle]", "pub extern \"C\" fn design() {", "    unsafe {"]
    for i, value in enumerate(q0):
        if value != 0.0:
            lines.append(f"        MEM[O_Q + {i}] = {value:.17e};")
    lines += ["    }", "}", ""]

    # packing: raw geometry -> the g vector
    lines += ["/// Build the derived geometry vector from the raw points, axes and numbers.",
              "#[no_mangle]", "pub extern \"C\" fn pack() -> i32 {", "    unsafe {",
              "        let mem = &*core::ptr::addr_of!(MEM);"]
    site_locals = _site_frames(problem, model, axes)
    lines += site_locals
    for k, key in enumerate(problem._gkey):
        lines.append(f"        MEM[O_G + {k}] = {_g_expression(key, problem, model, points, axes, numbers, size)};")
    lines += ["        0", "    }", "}", ""]

    # compute: residual and Jacobian
    cse_lines, out_lines = _cse_block(problem, n, rows)
    lines += [
        "#[inline]",
        "fn compute_inner(q: &[f64; N], g: &[f64; G], t: &[f64; A], out: &mut [f64; M], jac: &mut [f64; N*M]) {",
        *cse_lines,
        *out_lines,
        "}",
        "",
        "#[no_mangle]",
        "pub extern \"C\" fn compute() {",
        "    let mut q = [0.0f64; N];",
        "    let mut g = [0.0f64; G];",
        "    let mut t = [0.0f64; A];",
        "    let mut out = [0.0f64; M];",
        "    let mut jac = [0.0f64; N*M];",
        "    unsafe {",
        "        for i in 0..N { q[i] = MEM[O_Q + i]; }",
        "        for i in 0..G { g[i] = MEM[O_G + i]; }",
        "        for i in 0..A { t[i] = MEM[O_T + i]; }",
        "    }","        compute_inner(&q, &g, &t, &mut out, &mut jac);",
        "    unsafe {",
        "        for i in 0..M { MEM[O_OUT + i] = out[i]; }",
        "        for i in 0..N*M { MEM[O_JAC + i] = jac[i]; }",
        "    }",
        "}",
        "",
    ]

    # world points
    world_lines = _world_block(problem, model, points)
    lines += [
        "#[no_mangle]",
        "pub extern \"C\" fn world() {",
        "    let mut q = [0.0f64; N];",
        "    let mut g = [0.0f64; G];",
        "    unsafe {",
        "        for i in 0..N { q[i] = MEM[O_Q + i]; }",
        "        for i in 0..G { g[i] = MEM[O_G + i]; }",
        "    }",
        *world_lines,
        "}",
        "",
    ]

    lines += _metrics_block(problem, model)

    # solvers
    lines += [
        "#[inline]",
        "fn solve_linear(a: &mut [f64; N*N], b: &mut [f64; N]) -> bool {",
        "    for col in 0..N {",
        "        let mut pivot = col;",
        "        let mut best = a[col*N + col].abs();",
        "        for row in col+1..N {",
        "            let value = a[row*N + col].abs();",
        "            if value > best { best = value; pivot = row; }",
        "        }",
        "        if best < 1e-14 { return false; }",
        "        if pivot != col {",
        "            for k in 0..N { a.swap(col*N + k, pivot*N + k); }",
        "            b.swap(col, pivot);",
        "        }",
        "        let d = a[col*N + col];",
        "        for row in col+1..N {",
        "            let f = a[row*N + col] / d;",
        "            if f != 0.0 {",
        "                for k in col..N { a[row*N + k] -= f * a[col*N + k]; }",
        "                b[row] -= f * b[col];",
        "            }",
        "        }",
        "    }",
        "    for row in (0..N).rev() {",
        "        let mut s = b[row];",
        "        for k in row+1..N { s -= a[row*N + k] * b[k]; }",
        "        b[row] = s / a[row*N + row];",
        "    }",
        "    true",
        "}",
        "",
        "#[inline]",
        "fn residual(q: &[f64; N], g: &[f64; G], t: &[f64; A], out: &mut [f64; M], jac: &mut [f64; N*M]) -> f64 {",
        "    compute_inner(q, g, t, out, jac);",
        "    let mut worst = 0.0f64;",
        "    for i in 0..M { let v = out[i].abs(); if v > worst { worst = v; } }",
        "    worst",
        "}",
        "",
    ]
    if kind == "square":
        lines += _newton_solver()
    elif kind == "redundant":
        lines += _least_squares_solver()
    else:
        lines += _equilibrium_solver(problem, model)
    return "\n".join(lines)


def _site_frames(problem: Problem, model: Model, axes: list[str]) -> list[str]:
    lines = []
    for index, (site, _matrix) in enumerate(sorted(problem._site_c.items())):
        declaration = model.topology.sites[site]
        names = {"x": declaration.x, "y": declaration.y, "z": declaration.z}
        args = []
        for component in ("x", "y"):
            name = names[component]
            args.append(
                f"Some([{_axis_expr(axes, name, 0)}, {_axis_expr(axes, name, 1)}, {_axis_expr(axes, name, 2)}])"
                if name
                else "None"
            )
        z_name = names["z"]
        z = [f"{_axis_expr(axes, z_name, k)}" for k in range(3)] if z_name else ["0.0", "0.0", "1.0"]
        lines.append(
            f"        let f{index} = match frame([{z[0]}, {z[1]}, {z[2]}], {args[0]}, {args[1]}) "
            f"{{ Some(v) => v, None => return 1 }};"
        )
    return lines


def _axis_expr(axes: list[str], name: str | None, k: int) -> str:
    if name is None:
        return ["0.0", "0.0", "1.0"][k]
    return f"MEM[O_AXS + {3 * axes.index(name) + k}]"


def _g_expression(key, problem, model, points, axes, numbers, size) -> str:
    kind = key[0]
    if kind == "point":
        return f"MEM[O_PTS + {3 * points.index(key[1]) + key[2]}]"
    if kind == "site":
        index = list(sorted(problem._site_c)).index(key[1])
        return f"f{index}[{key[2]}]"
    slot = numbers.index(key)
    if kind == "gap":
        return f"MEM[O_NUM + {slot}]"
    if kind == "compliance":
        return f"MEM[O_NUM + {slot}]"
    # length: a NaN slot means "the design separation"
    joint = problem.resolved.joints[key[1]]
    a = points.index(model.topology.sites[joint.from_].origin)
    b = points.index(model.topology.sites[joint.to].origin)
    return (
        f"if MEM[O_NUM + {slot}].is_nan() "
        f"{{ norm3(sub3(point(mem, {b}), point(mem, {a}))) }} else {{ MEM[O_NUM + {slot}] }}"
    )


def _solver_prelude() -> list[str]:
    return [
        "    let mut q = [0.0f64; N];",
        "    let mut g = [0.0f64; G];",
        "    let mut t = [0.0f64; A];",
        "    unsafe {",
        "        for i in 0..N { q[i] = MEM[O_Q + i]; }",
        "        for i in 0..G { g[i] = MEM[O_G + i]; }",
        "        for i in 0..A { t[i] = MEM[O_T + i]; }",
        "    }",
    ]


def _newton_solver() -> list[str]:
    return [
        "/// Damped Newton from the current Q. Returns the infinity norm of the residual; a",
        "/// negative value means a singular Jacobian (-1) or no improving step (-2).",
        "#[no_mangle]",
        'pub extern "C" fn solve(iters: i32) -> f64 {',
        *_solver_prelude(),
        "    let mut out = [0.0f64; M];",
        "    let mut jac = [0.0f64; N*M];",
        "    let mut worst = residual(&q, &g, &t, &mut out, &mut jac);",
        "    let mut status = -2.0;",
        "    for _ in 0..iters {",
        "        if worst < 1e-12 { status = worst; break; }",
        "        let mut a = [0.0f64; N*N];",
        "        let mut b = [0.0f64; N];",
        "        for k in 0..N*N { a[k] = jac[k]; }",
        "        for i in 0..M { b[i] = -out[i]; }",
        "        if !solve_linear(&mut a, &mut b) { status = -1.0; break; }",
        "        let base = worst;",
        "        let mut alpha = 1.0;",
        "        let mut moved = false;",
        "        for _ in 0..12 {",
        "            let mut trial = q;",
        "            for i in 0..N { trial[i] += alpha * b[i]; }",
        "            let mut out_trial = [0.0f64; M];",
        "            let mut jac_trial = [0.0f64; N*M];",
        "            let value = residual(&trial, &g, &t, &mut out_trial, &mut jac_trial);",
        "            if value < base {",
        "                q = trial; worst = value; out = out_trial; jac = jac_trial;",
        "                moved = true; status = worst; break;",
        "            }",
        "            alpha *= 0.5;",
        "        }",
        "        if !moved { status = -2.0; break; }",
        "    }",
        "    unsafe { for i in 0..N { MEM[O_Q + i] = q[i]; } }",
        "    status",
        "}",
        "",
    ]


def _least_squares_solver() -> list[str]:
    """Levenberg–Marquardt on the normal equations: `(JᵀJ + λI) δ = −Jᵀr`."""
    return [
        "/// Levenberg–Marquardt from the current Q, for a redundant model. Returns the infinity",
        "/// norm of the residual; -1 means the normal equations were singular.",
        "#[no_mangle]",
        'pub extern "C" fn solve(iters: i32) -> f64 {',
        *_solver_prelude(),
        "    let mut out = [0.0f64; M];",
        "    let mut jac = [0.0f64; N*M];",
        "    let mut worst = residual(&q, &g, &t, &mut out, &mut jac);",
        "    let mut lambda = 1e-6f64;",
        "    for _ in 0..iters {",
        "        if worst < 1e-12 { break; }",
        "        let mut a = [0.0f64; N*N];",
        "        let mut b = [0.0f64; N];",
        "        for i in 0..M {",
        "            for p in 0..N { b[p] -= jac[i*N + p] * out[i]; }",
        "        }",
        "        for p in 0..N {",
        "            for r in 0..N {",
        "                let mut s = 0.0f64;",
        "                for i in 0..M { s += jac[i*N + p] * jac[i*N + r]; }",
        "                a[p*N + r] = s;",
        "            }",
        "        }",
        "        for d in 0..N { a[d*N + d] += lambda * (a[d*N + d] + 1e-12); }",
        "        if !solve_linear(&mut a, &mut b) { return -1.0; }",
        "        let mut trial = q;",
        "        for i in 0..N { trial[i] += b[i]; }",
        "        let mut out_trial = [0.0f64; M];",
        "        let mut jac_trial = [0.0f64; N*M];",
        "        let value = residual(&trial, &g, &t, &mut out_trial, &mut jac_trial);",
        "        if value < worst {",
        "            q = trial; out = out_trial; jac = jac_trial; worst = value;",
        "            lambda = (lambda * 0.3).max(1e-12);",
        "        } else {",
        "            lambda = (lambda * 10.0).min(1e9);",
        "        }",
        "    }",
        "    unsafe { for i in 0..N { MEM[O_Q + i] = q[i]; } }",
        "    worst",
        "}",
        "",
    ]


def _equilibrium_solver(problem: Problem, model: Model) -> list[str]:
    """Newton on the equilibrium residual, for a compliant model.

    `r = [constrained rows ; Ẑ·∇Π / f_ref]`, where `Ẑ` spans the null space of the constrained
    Jacobian. That null space is one-dimensional and is found by one linear solve rather than an
    SVD: appending the *spring row's* gradient as an extra row makes the matrix square and
    non-singular, and the solution of `[J_c ; ∇g_spring] u = e_last` is a vector with `J_c u = 0`.

    The Jacobian of `r` is differentiated numerically: the projected potential's analytic
    derivative contains `(∂Z/∂q)ᵀ∇Π`, which is the term that stalled the project's Python
    solver, and the same decision applies here.
    """
    springs = [i for i, row in enumerate(problem.rows) if row.group == "spring"]
    constrained = [i for i, row in enumerate(problem.rows) if row.group != "spring"]
    if not springs:
        raise ValueError("an equilibrium kernel needs at least one compliant component")
    if len(constrained) != problem.n - 1:
        raise ValueError(
            f"an equilibrium kernel expects one free direction; this model has {problem.n - len(constrained)}"
        )
    compliance_index = []
    for index in springs:
        joint, component = problem.rows[index].name.split(".", 1)
        compliance_index.append(problem._gkey.index(("compliance", joint, component, 0)))

    loads = []
    for load in (model.study.loads if model.study else []):
        if load.law.kind != "constant":
            raise ValueError("the kernel takes constant loads only")
        site = problem.topology.sites[load.site]
        body = problem.resolved.point_body[site.origin]
        loads.append((body, site.origin, list(load.force)))
    force_scale = max(1.0, sum(sum(abs(v) for v in force) for _, _, force in loads))

    mapping = _symbol_map(problem)
    n = problem.n

    def jacobian_rows(body: str, point: str) -> list[str]:
        p_, r_ = problem._pose[body]
        expr = p_ + r_ * problem._point_sym[point]
        jac = expr.jacobian(problem.q)
        return [_rust(jac[k, j].xreplace(mapping)) for k in range(3) for j in range(n)]

    rows = ",".join(str(i) for i in constrained)
    spring_rows = ",".join(str(i) for i in springs)
    spring_c = ",".join(str(i) for i in compliance_index)

    lines = [
        f"const ROWS: usize = {len(constrained)};",
        f"const ROW_INDEX: [usize; ROWS] = [{rows}];",
        f"const SPRING_ROWS: [usize; {len(springs)}] = [{spring_rows}];",
        f"const SPRING_C: [usize; {len(springs)}] = [{spring_c}];",
        f"const FORCE_SCALE: f64 = {force_scale:.17e};",
        "",
        "#[inline]",
        "fn maxabs(v: &[f64; N]) -> f64 {",
        "    let mut worst = 0.0f64;",
        "    for i in 0..N { let a = v[i].abs(); if a > worst { worst = a; } }",
        "    worst",
        "}",
        "",
        "#[inline]",
        "fn equilibrium_rows(q: &[f64; N], g: &[f64; G], t: &[f64; A]) -> [f64; N] {",
        "    let mut out = [0.0f64; M];",
        "    let mut jac = [0.0f64; N*M];",
        "    compute_inner(q, g, t, &mut out, &mut jac);",
        "    let mut grad = [0.0f64; N];",
        "    for s in 0..SPRING_ROWS.len() {",
        "        let scale = out[SPRING_ROWS[s]] / g[SPRING_C[s]];",
        "        for j in 0..N { grad[j] += scale * jac[SPRING_ROWS[s]*N + j]; }",
        "    }",
    ]
    for body, point, force in loads:
        values = jacobian_rows(body, point)
        lines.append("    let jx = [")
        for k in range(3):
            chunk = ", ".join(values[k * n : (k + 1) * n])
            lines.append(f"        {chunk},")
        lines.append("    ];")
        for k, name in enumerate("xyz"):
            if force[k]:
                lines.append(
                    f"    for j in 0..N {{ grad[j] -= jx[{k}*N + j] * {float(force[k]):.17e}; }}"
                )
    lines += [
        "    let mut rows = [0.0f64; ROWS];",
        "    for i in 0..ROWS { rows[i] = out[ROW_INDEX[i]]; }",
        "    // one linear solve gives the free direction: J_c u = 0, with the spring gradient",
        "    // appended so the matrix is square and the result is not the zero vector.",
        "    let mut a = [0.0f64; N*N];",
        "    let mut b = [0.0f64; N];",
        "    for i in 0..ROWS {",
        "        for j in 0..N { a[i*N + j] = jac[ROW_INDEX[i]*N + j]; }",
        "    }",
        "    for j in 0..N { a[ROWS*N + j] = jac[SPRING_ROWS[0]*N + j]; }",
        "    for j in 0..N { b[j] = 0.0; }",
        "    b[N - 1] = 1.0;",
        "    let mut residual = [0.0f64; N];",
        "    for i in 0..ROWS { residual[i] = rows[i]; }",
        "    residual[N - 1] = 0.0;",
        "    if solve_linear(&mut a, &mut b) {",
        "        let mut norm = 0.0f64;",
        "        for j in 0..N { norm += b[j] * b[j]; }",
        "        norm = norm.sqrt();",
        "        let mut projected = 0.0f64;",
        "        for j in 0..N { projected += b[j] * grad[j]; }",
        "        residual[N - 1] = projected / (norm * FORCE_SCALE);",
        "    } else {",
        "        residual[N - 1] = f64::NAN;",
        "    }",
        "    residual",
        "}",
        "",
        "/// Newton on the equilibrium residual. Returns its infinity norm, or -1 if the",
        "/// constrained Jacobian was singular.",
        "#[no_mangle]",
        'pub extern "C" fn solve(iters: i32) -> f64 {',
        *_solver_prelude(),
        "    let mut r = equilibrium_rows(&q, &g, &t);",
        "    let mut worst = maxabs(&r);",
        "    for _ in 0..iters {",
        "        if worst < 1e-10 { break; }",
        "        let mut a = [0.0f64; N*N];",
        "        for j in 0..N {",
        "            let h = 1e-7 * (1.0 + q[j].abs());",
        "            let mut qp = q;",
        "            let mut qm = q;",
        "            qp[j] += h;",
        "            qm[j] -= h;",
        "            let rp = equilibrium_rows(&qp, &g, &t);",
        "            let rm = equilibrium_rows(&qm, &g, &t);",
        "            for i in 0..N { a[i*N + j] = (rp[i] - rm[i]) / (2.0 * h); }",
        "        }",
        "        let mut step = r;",
        "        for i in 0..N { step[i] = -step[i]; }",
        "        if !solve_linear(&mut a, &mut step) { return -1.0; }",
        "        let base = worst;",
        "        let mut alpha = 1.0;",
        "        let mut moved = false;",
        "        for _ in 0..12 {",
        "            let mut trial = q;",
        "            for i in 0..N { trial[i] += alpha * step[i]; }",
        "            let candidate = equilibrium_rows(&trial, &g, &t);",
        "            let value = maxabs(&candidate);",
        "            if value < base { q = trial; r = candidate; worst = value; moved = true; break; }",
        "            alpha *= 0.5;",
        "        }",
        "        if !moved { break; }",
        "    }",
        "    unsafe { for i in 0..N { MEM[O_Q + i] = q[i]; } }",
        "    worst",
        "}",
        "",
    ]
    return lines


def _metrics_block(problem: Problem, model: Model) -> list[str]:
    """The quantities a figure draws, computed in the kernel from the same construction Python uses.

    These are formulas, not derivations: camber and toe from the axle, the contact patch, the
    front-view instant centre through the foot of the perpendicular, and the roll centre on the
    line from the patch. `tests/test_wasm.py` requires them to agree with `analysis/outputs.py`
    to 1e-9, which is what keeps two implementations honest.
    """
    from ..analysis import outputs

    structure = outputs._front_view(problem)
    if structure is None:
        return ["// metrics: this model has no front-view structure to report", ""]
    lbj, foot_lower, ubj, foot_upper = structure
    reference = outputs.make_reference(problem)
    axle_a, axle_b = problem.resolved.axle
    mapping = _symbol_map(problem)

    def point_expressions(name: str) -> list[str]:
        body = problem.resolved.point_body[name]
        p, rotation = problem._pose[body]
        world = p + rotation * problem._point_sym[name]
        return [_rust(world[i].xreplace(mapping)) for i in range(3)]

    def constant(value: float) -> str:
        return f"{float(value):.17e}"

    radius = float(reference.tyre_radius)
    heading = float(reference.heading)
    contact_x0 = float(reference.contact[0])

    lines = [
        "/// Camber, toe, track, contact patch, instant centre and roll centre at the current pose.",
        "/// Parallel or degenerate lines report NaN, which the glue turns into null.",
        "#[no_mangle]",
        'pub extern "C" fn metrics() {',
        "    let mut q = [0.0f64; N];",
        "    let mut g = [0.0f64; G];",
        "    unsafe {",
        "        for i in 0..N { q[i] = MEM[O_Q + i]; }",
        "        for i in 0..G { g[i] = MEM[O_G + i]; }",
        "    }",
    ]
    for name in dict.fromkeys([axle_a, axle_b, lbj, ubj]):
        values = point_expressions(name)
        lines.append(f"    let p_{_ident(name)} = [{values[0]}, {values[1]}, {values[2]}];")

    lines += [
        f"    let axle = sub3(p_{_ident(axle_b)}, p_{_ident(axle_a)});",
        "    let an = norm3(axle);",
        "    let n = [axle[0] / an, axle[1] / an, axle[2] / an];",
        f"    let centre = [(p_{_ident(axle_a)}[0] + p_{_ident(axle_b)}[0]) * 0.5,",
        f"                  (p_{_ident(axle_a)}[1] + p_{_ident(axle_b)}[1]) * 0.5,",
        f"                  (p_{_ident(axle_a)}[2] + p_{_ident(axle_b)}[2]) * 0.5];",
    ]

    road = problem.resolved.road
    if road is None:
        lines += [
            "    // the lowest point of the wheel disc: down, projected into the wheel plane",
            "    let in_plane = [n[2] * n[0], n[2] * n[1], n[2] * n[2] - 1.0];",
            "    let scale = norm3(in_plane);",
            "    let contact3 = [centre[0] + " + constant(radius) + " * in_plane[0] / scale,",
            "                    centre[1] + " + constant(radius) + " * in_plane[1] / scale,",
            "                    centre[2] + " + constant(radius) + " * in_plane[2] / scale];",
        ]
    else:
        plane = next(j for j in problem.resolved.joints.values() if j.family == "plane" and j.a == road)
        p_road, rotation_road = problem._pose[road]
        normal = (rotation_road * problem._site_c[plane.from_] * sp.Matrix([0, 0, 1])).xreplace(mapping)
        origin = problem._point_sym[problem.topology.sites[plane.from_].origin]
        patch = (p_road + rotation_road * origin).xreplace(mapping)
        lines += [
            "    let road_n = [%s, %s, %s];" % tuple(_rust(normal[i]) for i in range(3)),
            "    let patch = [%s, %s, %s];" % tuple(_rust(patch[i]) for i in range(3)),
            "    let rn = norm3(road_n);",
            "    let road_n = [road_n[0] / rn, road_n[1] / rn, road_n[2] / rn];",
            "    let gap = dot3(road_n, sub3(centre, patch));",
            "    let contact3 = [centre[0] - gap * road_n[0],",
            "                    centre[1] - gap * road_n[1],",
            "                    centre[2] - gap * road_n[2]];",
        ]

    lines += [
        "    let contact = [contact3[0], contact3[2]];",
        "    let camber = (-n[2]).asin().to_degrees();",
        f"    let mut toe = n[1].atan2(n[0]).to_degrees() - {constant(math.degrees(heading))};",
        "    if toe > 180.0 { toe -= 360.0; } else if toe < -180.0 { toe += 360.0; }",
        f"    let track = contact[0] - {constant(contact_x0)};",
        f"    let foot_l = [{constant(foot_lower[0])}, {constant(foot_lower[1])}, {constant(foot_lower[2])}];",
        f"    let foot_u = [{constant(foot_upper[0])}, {constant(foot_upper[1])}, {constant(foot_upper[2])}];",
        f"    let d_l = sub3(foot_l, p_{_ident(lbj)});",
        f"    let d_u = sub3(foot_u, p_{_ident(ubj)});",
        f"    let ic = intersect2([p_{_ident(lbj)}[0], p_{_ident(lbj)}[2]], [d_l[0], d_l[2]],",
        f"                        [p_{_ident(ubj)}[0], p_{_ident(ubj)}[2]], [d_u[0], d_u[2]]);",
        "    let rc = if (ic[0] - contact[0]).abs() < 1e-12 {",
        "        [f64::NAN, f64::NAN]",
        "    } else {",
        "        let t = (0.0 - contact[0]) / (ic[0] - contact[0]);",
        "        [0.0, contact[1] + t * (ic[1] - contact[1])]",
        "    };",
        "    unsafe {",
        "        MEM[O_MET + 0] = camber;",
        "        MEM[O_MET + 1] = toe;",
        "        MEM[O_MET + 2] = track;",
        "        MEM[O_MET + 3] = contact[0];",
        "        MEM[O_MET + 4] = contact[1];",
        "        MEM[O_MET + 5] = ic[0];",
        "        MEM[O_MET + 6] = ic[1];",
        "        MEM[O_MET + 7] = rc[0];",
        "        MEM[O_MET + 8] = rc[1];",
        "        MEM[O_MET + 9] = 0.0;",
        "    }",
        "}",
        "",
    ]
    return lines


def _ident(name: str) -> str:
    return name.replace("-", "_").replace(".", "_")


def _symbol_map(problem: Problem) -> dict:
    mapping = {}
    for i, symbol in enumerate(problem.q):
        mapping[symbol] = sp.Symbol(f"q[{i}]")
    for i, symbol in enumerate(problem.g):
        mapping[symbol] = sp.Symbol(f"g[{i}]")
    for i, drive in enumerate(problem.drives):
        mapping[drive.theta] = sp.Symbol(f"t[{i}]")
    return mapping


def _cse_block(problem: Problem, n: int, rows: int) -> tuple[list[str], list[str]]:
    mapping = _symbol_map(problem)
    # Row-major: jac[row*N + col]. The two-leg model has more rows than unknowns, so the
    # iteration order matters and the square case must use the same layout.
    flat = [problem.jacobian_expr[i, j] for i in range(rows) for j in range(n)]
    exprs = [expression.xreplace(mapping) for expression in [*problem._exprs, *flat]]
    replacements, reduced = cse(exprs, optimizations="basic")
    body = [f"    let {symbol} = {_rust(expression)};" for symbol, expression in replacements]
    out = [f"    out[{i}] = {_rust(reduced[i])};" for i in range(rows)]
    out += [f"    jac[{k}] = {_rust(reduced[rows + k])};" for k in range(n * rows)]
    return body, out


def _world_block(problem: Problem, model: Model, points: list[str]) -> list[str]:
    mapping = _symbol_map(problem)
    exprs = []
    for point in points:
        body = problem.resolved.point_body[point]
        p, rotation = problem._pose[body]
        world = p + rotation * problem._point_sym[point]
        exprs.extend(world[i].xreplace(mapping) for i in range(3))
    replacements, reduced = cse(exprs, optimizations="basic")
    lines = [f"    let {symbol} = {_rust(expression)};" for symbol, expression in replacements]
    lines += [f"    unsafe {{ MEM[O_WORLD + {i}] = {_rust(reduced[i])}; }}" for i in range(len(exprs))]
    return lines


def _default_numbers(problem: Problem, numbers: list[tuple]) -> list:
    resolved = problem.resolved
    values: list = []
    for key in numbers:
        if key[0] == "length":
            values.append(resolved.length.get(key[1]))
        elif key[0] == "gap":
            values.append(resolved.gap.get(key[1]))
        else:
            values.append(resolved.compliance.get((key[1], key[2])))
    return values


def _number_descriptor(key: tuple) -> dict:
    if key[0] == "compliance":
        return {"kind": "compliance", "joint": key[1], "component": key[2]}
    return {"kind": key[0], "joint": key[1]}


# -- Rust text -------------------------------------------------------------------------


def _rust(expression) -> str:
    """Print an expression as Rust, with integer literals promoted to floats.

    sympy prints `2*x`, which is a type error in Rust: integer literals are not coerced to
    `f64`. Promotion must skip the two places an integer is required — `.powi(n)` and array
    indices — so this walks the string rather than rewriting it with a regex.
    """
    text = rust_code(expression)
    out: list[str] = []
    i = 0
    brackets = 0
    while i < len(text):
        if text.startswith(".powi(", i):
            depth = 0
            j = i
            while j < len(text):
                if text[j] == "(":
                    depth += 1
                elif text[j] == ")":
                    depth -= 1
                    if depth == 0:
                        j += 1
                        break
                j += 1
            out.append(text[i:j])
            i = j
            continue
        ch = text[i]
        if ch == "[":
            brackets += 1
        elif ch == "]":
            brackets -= 1
        if ch.isdigit() and (i == 0 or not (text[i - 1].isalnum() or text[i - 1] in "._")):
            j = i
            while j < len(text) and (text[j].isdigit() or text[j] == "."):
                j += 1
            token = text[i:j]
            if j < len(text) and text[j] in "eE":
                out.append(token)
                i = j
                continue
            out.append(token if (brackets or "." in token) else token + ".0")
            i = j
            continue
        out.append(ch)
        i += 1
    return "".join(out)


# -- building and glue -----------------------------------------------------------------


def _build(target: Path) -> tuple[Path | None, str]:
    rustc = shutil.which("rustc")
    if rustc is None:
        return None, "rustc not found: kernel.rs written, kernel.wasm not built"
    command = [
        rustc, "--target", TARGET, "--crate-type", "cdylib", "--edition", "2021",
        "-O", "-C", "panic=abort", "-C", "strip=symbols",
        "-C", "lto=fat", "-C", "codegen-units=1",
        "kernel.rs", "-o", "kernel.wasm",
    ]
    result = subprocess.run(command, cwd=target, capture_output=True, text=True)
    if result.returncode != 0:
        detail = (result.stderr or "").strip().splitlines()
        return None, f"rustc failed ({TARGET}): " + " / ".join(detail[:3])
    return target / "kernel.wasm", ""


def _js_glue(manifest: dict, module_b64: str) -> str:
    layout = manifest["layout"]
    return f"""// Generated by otkd. Do not edit by hand.
// model: {manifest['model']}
//
// A fixed-size kernel: {layout['slots']} f64 slots, no allocator, no imports. Write the raw
// geometry into the input blocks, call pack(), then solve() as often as you like.
export const manifest = {json.dumps(manifest, indent=2)};

const BASE64 = "{module_b64}";

let instance = null;
let view = null;
/** First f64 index of the module's static buffer; the offsets are relative to it. */
let base = 0;

const O = manifest.layout.offsets;
const S = manifest.layout.sizes;
const at = (key) => base + O[key];

function memory() {{
  if (view === null || view.buffer !== instance.exports.memory.buffer) {{
    view = new Float64Array(instance.exports.memory.buffer);
  }}
  return view;
}}

/** Instantiate the module. Call once; the returned promise is cached. */
export async function ready() {{
  if (instance) return api;
  const binary = Uint8Array.from(atob(BASE64), (c) => c.charCodeAt(0));
  const {{ instance: created }} = await WebAssembly.instantiate(binary, {{}});
  instance = created;
  base = created.exports.MEM.value / 8;
  return api;
}}

/** Reset the pose to the design position. */
export function design() {{
  instance.exports.design();
}}

/**
 * Write raw geometry and derive everything else inside the kernel.
 * `geometry` is {{ points: {{name: [x,y,z]}}, axes: {{name: [x,y,z]}}, numbers: [..] }}.
 * Returns the kernel's status: 0 ok, 1 a degenerate axis frame, 2 a missing number.
 */
export function setGeometry(geometry) {{
  const mem = memory();
  manifest.points.forEach((name, i) => {{
    const p = geometry.points[name];
    if (!p) throw new Error(`geometry is missing point ${{name}}`);
    mem.set(p, at('pts') + 3 * i);
  }});
  manifest.axes.forEach((name, i) => {{
    const a = geometry.axes[name];
    if (!a) throw new Error(`geometry is missing axis ${{name}}`);
    mem.set(a, at('axs') + 3 * i);
  }});
  (geometry.numbers ?? []).forEach((value, i) => {{ mem[at('num') + i] = value ?? NaN; }});
  return instance.exports.pack();
}}

/** Set the driven coordinate(s). */
export function setDrive(value) {{
  const mem = memory();
  if (Array.isArray(value)) value.forEach((v, i) => {{ mem[at('t') + i] = v; }});
  else mem[at('t')] = value;
}}

/** Solve from the current pose. Returns the residual norm (negative on failure). */
export function solve(iterations = 40) {{
  return instance.exports.solve(iterations);
}}

/** The solved pose, {{S['q']}} numbers. */
export function pose() {{
  return Array.from(memory().subarray(at('q'), at('q') + S.q));
}}

/** The residual and Jacobian at the current pose. */
export function evaluate() {{
  instance.exports.compute();
  const mem = memory();
  return {{
    rows: Array.from(mem.subarray(at('out'), at('out') + S.out)),
    jac: Array.from(mem.subarray(at('jac'), at('jac') + S.jac)),
  }};
}}

/**
 * The quantities a figure draws, at the current pose. `ic` and `rc` are null when the lines
 * are parallel — the kernel reports NaN and this turns it into null.
 */
export function metrics() {{
  instance.exports.metrics();
  const mem = memory();
  const m = Array.from(mem.subarray(at('met'), at('met') + S.met));
  const pair = (x, y) => (Number.isFinite(x) && Number.isFinite(y) ? [x, y] : null);
  return {{
    camber: m[0],
    toe: m[1],
    track: m[2],
    contact: pair(m[3], m[4]),
    ic: pair(m[5], m[6]),
    rc: pair(m[7], m[8]),
  }};
}}

/** World positions of every geometry point, in `manifest.points` order, [x, y, z] each. */
export function world() {{
  instance.exports.world();
  const mem = memory();
  const out = {{}};
  manifest.points.forEach((name, i) => {{
    out[name] = Array.from(mem.subarray(at('world') + 3 * i, at('world') + 3 * i + 3));
  }});
  return out;
}}

/** Pack, solve one frame, return the pose, points and a status. One call per interaction. */
export async function frame(geometry, drive) {{
  await ready();
  const status = setGeometry(geometry);
  design();
  setDrive(drive);
  const residual = solve();
  return {{ status, residual, q: pose(), points: world(), metrics: metrics() }};
}}

const api = {{
  manifest, ready, design, setGeometry, setDrive, solve, pose, evaluate, metrics, world, frame,
}};
export default api;
"""
