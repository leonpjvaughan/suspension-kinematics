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

    sizes = {
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
    }
    study = model.study
    if kind == "dynamics":
        assert study is not None and study.steps is not None
        sizes["v"] = problem.n
        sizes["acc"] = problem.n
        sizes["traj"] = study.steps * problem.n
        sizes["trajv"] = study.steps * problem.n
    layout = Layout(sizes=sizes, offsets={}, total=0)
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
        **(
            {
                "integration": {
                    "duration": study.duration,
                    "steps": study.steps,
                    "dt": (study.duration or 0.0) / (study.steps - 1),
                }
            }
            if kind == "dynamics"
            else {}
        ),
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
        # A trajectory is a different program from the three pose solvers, and it is generated
        # only for a study that asks for one: the buffers it needs are `steps × n` numbers.
        return "dynamics"
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
    offsets = ["q", "g", "t", "out", "jac", "met", "world", "pts", "axs", "num"]
    if kind == "dynamics":
        offsets += ["v", "acc", "traj", "trajv"]
    for key in offsets:
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
        "fn solve_linear(n: usize, a: &mut [f64], b: &mut [f64]) -> bool {",
        "    for col in 0..n {",
        "        let mut pivot = col;",
        "        let mut best = a[col*n + col].abs();",
        "        for row in col+1..n {",
        "            let value = a[row*n + col].abs();",
        "            if value > best { best = value; pivot = row; }",
        "        }",
        "        if best < 1e-14 { return false; }",
        "        if pivot != col {",
        "            for k in 0..n { a.swap(col*n + k, pivot*n + k); }",
        "            b.swap(col, pivot);",
        "        }",
        "        let d = a[col*n + col];",
        "        for row in col+1..n {",
        "            let f = a[row*n + col] / d;",
        "            if f != 0.0 {",
        "                for k in col..n { a[row*n + k] -= f * a[col*n + k]; }",
        "                b[row] -= f * b[col];",
        "            }",
        "        }",
        "    }",
        "    for row in (0..n).rev() {",
        "        let mut s = b[row];",
        "        for k in row+1..n { s -= a[row*n + k] * b[k]; }",
        "        b[row] = s / a[row*n + row];",
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
    if kind == "dynamics":
        # The pose solvers still matter — a page places the mechanism by driving it — and the
        # integrator is generated on top of the same rows and Jacobian.
        lines += _newton_solver()
        lines += _dynamics_solver(problem, model)
    elif kind == "square":
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
        "        if !solve_linear(N, &mut a, &mut b) { status = -1.0; break; }",
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
        "        if !solve_linear(N, &mut a, &mut b) { return -1.0; }",
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
    """Newton on the KKT system, for a compliant model.

    The equilibrium is the saddle point of `Π` subject to the constrained rows, so this generates
    the system `[∇Π − Φᵀλ ; Φ] = 0` in `(q, λ)`: square, smooth, and needing no null-space basis.
    `λ` comes out as the reaction in every constrained row, which is the number a suspension
    engineer wants.

    The first version projected the potential onto a one-dimensional null space found by a single
    linear solve. That was right for the tie-rod corner and wrong for a bush at a ball joint, where
    three compliant components make the free space three-dimensional and an SVD basis that rotates
    with `q` makes the residual non-smooth — the Python solver failed the same way, and
    [`decisions.md`](../docs/decisions.md) round 23 has the measurements.

    The Jacobian is differentiated numerically, as in Python: the analytic one is what stalled.
    """
    springs = [i for i, row in enumerate(problem.rows) if row.group == "spring"]
    constrained = [i for i, row in enumerate(problem.rows) if row.group != "spring"]
    if not springs:
        raise ValueError("an equilibrium kernel needs at least one compliant component")
    free = problem.n - len(constrained)
    if free < 1:
        raise ValueError(f"an equilibrium kernel needs at least one free direction; this model has {free}")

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
        "/// The KKT system's dimension: the pose unknowns, then one multiplier per constrained row.",
        "const K: usize = N + ROWS;",
        "",
        "#[inline]",
        "fn maxabs(v: &[f64]) -> f64 {",
        "    let mut worst = 0.0f64;",
        "    for i in 0..v.len() { let a = v[i].abs(); if a > worst { worst = a; } }",
        "    worst",
        "}",
        "",
        "#[inline]",
        "fn kkt(x: &[f64], g: &[f64; G], t: &[f64; A], out: &mut [f64]) {",
        "    let mut q = [0.0f64; N];",
        "    for i in 0..N { q[i] = x[i]; }",
        "    let mut rows = [0.0f64; M];",
        "    let mut jac = [0.0f64; N*M];",
        "    compute_inner(&q, g, t, &mut rows, &mut jac);",
        "    let mut grad = [0.0f64; N];",
        "    for s in 0..SPRING_ROWS.len() {",
        "        let scale = rows[SPRING_ROWS[s]] / g[SPRING_C[s]];",
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
        "    for r in 0..ROWS {",
        "        let lam = x[N + r];",
        "        for j in 0..N { grad[j] -= jac[ROW_INDEX[r]*N + j] * lam; }",
        "    }",
        "    for j in 0..N { out[j] = grad[j] / FORCE_SCALE; }",
        "    for r in 0..ROWS { out[N + r] = rows[ROW_INDEX[r]]; }",
        "}",
        "",
        "/// Newton on the KKT residual. Returns its infinity norm, or -1 if the system was",
        "/// singular.",
        "#[no_mangle]",
        'pub extern "C" fn solve(iters: i32) -> f64 {',
        *_solver_prelude(),
        "    let mut x = [0.0f64; K];",
        "    for i in 0..N { x[i] = q[i]; }",
        "    let mut r = [0.0f64; K];",
        "    kkt(&x, &g, &t, &mut r);",
        "    let mut worst = maxabs(&r);",
        "    for _ in 0..iters {",
        "        // Tight, because the finite-difference Newton approaches the root from one side and",
        "        // stops wherever the threshold is; a browser pays for two more iterations of a",
        "        // 40-unknown system and gets a residual the conformance tests can read.",
        "        if worst < 1e-13 { break; }",
        "        let mut a = [0.0f64; K*K];",
        "        for j in 0..K {",
        "            let h = 1e-7 * (1.0 + x[j].abs());",
        "            let mut xp = x;",
        "            let mut xm = x;",
        "            xp[j] += h;",
        "            xm[j] -= h;",
        "            let mut rp = [0.0f64; K];",
        "            let mut rm = [0.0f64; K];",
        "            kkt(&xp, &g, &t, &mut rp);",
        "            kkt(&xm, &g, &t, &mut rm);",
        "            for i in 0..K { a[i*K + j] = (rp[i] - rm[i]) / (2.0 * h); }",
        "        }",
        "        let mut step = [0.0f64; K];",
        "        for i in 0..K { step[i] = -r[i]; }",
        "        if !solve_linear(K, &mut a, &mut step) { return -1.0; }",
        "        let base = worst;",
        "        let mut alpha = 1.0;",
        "        let mut moved = false;",
        "        for _ in 0..16 {",
        "            let mut trial = x;",
        "            for i in 0..K { trial[i] += alpha * step[i]; }",
        "            let mut cand = [0.0f64; K];",
        "            kkt(&trial, &g, &t, &mut cand);",
        "            let value = maxabs(&cand);",
        "            if value < base { x = trial; r = cand; worst = value; moved = true; break; }",
        "            alpha *= 0.5;",
        "        }",
        "        if !moved { break; }",
        "    }",
        "    unsafe { for i in 0..N { MEM[O_Q + i] = x[i]; } }",
        "    worst",
        "}",
        "",
    ]
    return lines



def _dynamics_solver(problem: Problem, model: Model) -> list[str]:
    """RK4 with projection: the same equations the Python engine solves, as Rust.

    `M q̈ + C = f − Φᵀλ` with `Φ q̈ = γ`, solved as one saddle-point system; `C` differentiated per
    body, which is the form round 26 settled on. After every accepted step the state is projected
    back onto `Φ = 0` and `Φ_q q̇ = 0`: RK4 drifts off a rigid link by `O(h⁴)` a step, and a
    trajectory that leaves the mechanism is not a trajectory of the mechanism.

    Two regularisations, both stated where they happen: the saddle system has a free direction (a
    dynamics model is *supposed* to), and a left-null vector makes it singular, so the constraint
    block gets `−εI`; and the projection solves `J Jᵀ dλ = −Φ`, which is singular for the same
    reason, so it gets `+εI` scaled to the matrix.
    """
    dynamics = model.geometry.dynamics
    if dynamics is None:
        raise ValueError("a dynamics kernel needs a `dynamics:` block")
    study = model.study
    assert study is not None and study.duration is not None and study.steps is not None
    n = problem.n
    rows = [i for i, row in enumerate(problem.rows) if row.group != "spring"]
    springs = [i for i, row in enumerate(problem.rows) if row.group == "spring"]
    if not rows:
        raise ValueError("a dynamics kernel needs at least one constrained row")

    bodies = [body for body in problem.moving]
    starts, masses, inertias, coms, dampings = [], [], [], [], []
    compliance_slots = []
    for body in bodies:
        numbers = dynamics.bodies.get(body)
        if numbers is None:
            raise ValueError(f"body {body!r} has no mass, so the model has no dynamics")
        starts.append(problem._qstart[body])
        masses.append(float(numbers.mass))
        inertia = [float(v) for v in numbers.inertia]
        inertias.append(inertia)
        com = problem.resolved.points[numbers.com] if isinstance(numbers.com, str) else numbers.com
        coms.append([float(v) for v in com])
    for index in springs:
        joint, component = problem.rows[index].name.split(".", 1)
        compliance_slots.append(problem._gkey.index(("compliance", joint, component, 0)))
        dampings.append(float(model.geometry.joints[joint].damping.get(component, 0.0)) if joint in model.geometry.joints else 0.0)

    loads = []
    for load in study.loads:
        if load.law.kind != "constant":
            raise ValueError("the kernel takes constant loads only")
        site = problem.topology.sites[load.site]
        body = problem.resolved.point_body[site.origin]
        loads.append((body, site.origin, [float(v) for v in load.force]))

    gravity = [float(v) for v in dynamics.gravity]
    mapping = _symbol_map(problem)

    def jacobian_rows(body: str, point: str) -> list[str]:
        p_, r_ = problem._pose[body]
        expr = p_ + r_ * problem._point_sym[point]
        jac = expr.jacobian(problem.q)
        return [_rust(jac[k, j].xreplace(mapping)) for k in range(3) for j in range(n)]

    def array(name: str, values: list[float], fmt: str = "{:.17e}") -> str:
        return f"const {name}: [f64; {len(values)}] = [" + ", ".join(fmt.format(v) for v in values) + "];"

    lines = [
        "// ---- dynamics: M qddot + C = f - Phi^T lambda, with Phi qddot = gamma ----",
        f"const NB: usize = {len(bodies)};",
        f"const R: usize = {len(rows)};",
        f"const S: usize = {len(springs)};",
        "const K: usize = N + R;",
        f"const DT: f64 = {study.duration / (study.steps - 1):.17e};",
        f"const ALLOC: usize = {study.steps};",
        "/// The saddle system's constraint block is regularised: a dynamics model has a free",
        "/// direction, so the constraint rows have a left null vector and the system is singular.",
        "const EPS: f64 = 1e-10;",
        "",
        "const BODY_START: [usize; NB] = [" + ", ".join(str(v) for v in starts) + "];",
        array("BODY_MASS", masses),
        "const BODY_INERTIA: [[f64; 6]; NB] = ["
        + ", ".join("[" + ", ".join(f"{v:.17e}" for v in inertia) + "]" for inertia in inertias)
        + "];",
        "const BODY_COM: [[f64; 3]; NB] = ["
        + ", ".join("[" + ", ".join(f"{v:.17e}" for v in com) + "]" for com in coms)
        + "];",
        "const GRAVITY: [f64; 3] = [" + ", ".join(f"{v:.17e}" for v in gravity) + "];",
        "const CONSTRAINED: [usize; R] = [" + ", ".join(str(i) for i in rows) + "];",
        "const SPRING_ROWS: [usize; S] = [" + ", ".join(str(i) for i in springs) + "];",
        "const SPRING_G: [usize; S] = [" + ", ".join(str(i) for i in compliance_slots) + "];",
        "const SPRING_DAMPING: [f64; S] = [" + ", ".join(f"{v:.17e}" for v in dampings) + "];",
        "",
        "#[inline]",
        "fn dyn_maxabs(v: &[f64; N]) -> f64 {",
        "    let mut worst = 0.0f64;",
        "    for i in 0..N { let a = v[i].abs(); if a > worst { worst = a; } }",
        "    worst",
        "}",
        "",
        "/// `d(p + R o)/dq` for the seven pose parameters, at a body's centre of mass.",
        "#[inline]",
        "fn pjac(o: [f64; 3], w: f64, x: f64, y: f64, z: f64) -> [[f64; 7]; 3] {",
        "    let v = [x, y, z];",
        "    let mut j = [[0.0f64; 7]; 3];",
        "    for k in 0..3 { j[k][k] = 1.0; }",
        "    let c = cross3(v, o);",
        "    for k in 0..3 { j[k][3] = 2.0 * (w * o[k] + c[k]); }",
        "    let vo = dot3(v, o);",
        "    for k in 0..3 {",
        "        let mut axis = [0.0f64; 3];",
        "        axis[k] = 1.0;",
        "        let a = cross3(axis, o);",
        "        for r in 0..3 {",
        "            j[r][4 + k] = 2.0 * (-v[k] * o[r] + axis[r] * vo + v[r] * o[k] + w * a[r]);",
        "        }",
        "    }",
        "    j",
        "}",
        "",
        "/// `omega = W(q) qdot`: the world-frame angular velocity from the quaternion rate.",
        "#[inline]",
        "fn omega_map(w: f64, x: f64, y: f64, z: f64) -> [[f64; 4]; 3] {",
        "    let mut m = [[0.0f64; 4]; 3];",
        "    m[0] = [2.0 * -x, 2.0 * w, 2.0 * -z, 2.0 * y];",
        "    m[1] = [2.0 * -y, 2.0 * z, 2.0 * w, 2.0 * -x];",
        "    m[2] = [2.0 * -z, 2.0 * -y, 2.0 * x, 2.0 * w];",
        "    m",
        "}",
        "",
        "/// One body's 7x7 block of M.",
        "#[inline]",
        "fn body_block(b: usize, q: &[f64; N]) -> [[f64; 7]; 7] {",
        "    let s = BODY_START[b];",
        "    let (w, x, y, z) = (q[s + 3], q[s + 4], q[s + 5], q[s + 6]);",
        "    let jv = pjac(BODY_COM[b], w, x, y, z);",
        "    let om = omega_map(w, x, y, z);",
        "    let i = BODY_INERTIA[b];",
        "    let inertia = [[i[0], i[3], i[4]], [i[3], i[1], i[5]], [i[4], i[5], i[2]]];",
        "    let mut out = [[0.0f64; 7]; 7];",
        "    for r in 0..7 {",
        "        for c in 0..7 {",
        "            let mut acc = 0.0;",
        "            for k in 0..3 { acc += jv[k][r] * jv[k][c]; }",
        "            out[r][c] = BODY_MASS[b] * acc;",
        "        }",
        "    }",
        "    for r in 3..7 {",
        "        for c in 3..7 {",
        "            let mut acc = 0.0;",
        "            for k in 0..3 {",
        "                for l in 0..3 { acc += om[k][r - 3] * inertia[k][l] * om[l][c - 3]; }",
        "            }",
        "            out[r][c] += acc;",
        "        }",
        "    }",
        "    out",
        "}",
        "",
        "#[inline]",
        "fn kinetic_body(b: usize, q: &[f64; N], v: &[f64; N]) -> f64 {",
        "    let s = BODY_START[b];",
        "    let block = body_block(b, q);",
        "    let mut sum = 0.0;",
        "    for r in 0..7 {",
        "        for c in 0..7 { sum += v[s + r] * block[r][c] * v[s + c]; }",
        "    }",
        "    0.5 * sum",
        "}",
        "",
        "/// `C = Mdot qdot - d(half qdot^T M qdot)/dq`, per body, as round 26 measured in Python.",
        "fn velocity_products(q: &[f64; N], v: &[f64; N], out: &mut [f64; N]) {",
        "    let mut speed = 0.0;",
        "    for i in 0..N { speed += v[i] * v[i]; }",
        "    let speed = speed.sqrt();",
        "    let step = if speed > 0.0 { 1e-7 / speed } else { 0.0 };",
        "    for i in 0..N { out[i] = 0.0; }",
        "    for b in 0..NB {",
        "        let s = BODY_START[b];",
        "        let mut grad = [0.0f64; 7];",
        "        for j in 0..7 {",
        "            let h = 1e-7 * (1.0 + q[s + j].abs());",
        "            let mut qp = *q;",
        "            let mut qm = *q;",
        "            qp[s + j] += h;",
        "            qm[s + j] -= h;",
        "            grad[j] = (kinetic_body(b, &qp, v) - kinetic_body(b, &qm, v)) / (2.0 * h);",
        "        }",
        "        if step > 0.0 {",
        "            let mut qp = *q;",
        "            let mut qm = *q;",
        "            for i in 0..N { qp[i] += step * v[i]; qm[i] -= step * v[i]; }",
        "            let bp = body_block(b, &qp);",
        "            let bm = body_block(b, &qm);",
        "            for r in 0..7 {",
        "                let mut fp = 0.0;",
        "                let mut fm = 0.0;",
        "                for c in 0..7 { fp += bp[r][c] * v[s + c]; fm += bm[r][c] * v[s + c]; }",
        "                out[s + r] = (fp - fm) / (2.0 * step) - grad[r];",
        "            }",
        "        } else {",
        "            for r in 0..7 { out[s + r] = -grad[r]; }",
        "        }",
        "    }",
        "}",
        "",
        "/// Gravity, the study's loads, and the springs' and dampers' internal forces.",
        "fn forces(q: &[f64; N], v: &[f64; N], g: &[f64; G], t: &[f64; A], out: &mut [f64; N]) {",
        "    for i in 0..N { out[i] = 0.0; }",
        "    for b in 0..NB {",
        "        let s = BODY_START[b];",
        "        let (w, x, y, z) = (q[s + 3], q[s + 4], q[s + 5], q[s + 6]);",
        "        let jv = pjac(BODY_COM[b], w, x, y, z);",
        "        for r in 0..7 {",
        "            let mut acc = 0.0;",
        "            for k in 0..3 { acc += jv[k][r] * BODY_MASS[b] * GRAVITY[k]; }",
        "            out[s + r] += acc;",
        "        }",
        "    }",
    ]
    for body, point, force in loads:
        values = jacobian_rows(body, point)
        lines.append("    {")
        lines.append("        let jx = [")
        for k in range(3):
            chunk = ", ".join(values[k * n : (k + 1) * n])
            lines.append(f"            [{chunk}],")
        lines.append("        ];")
        for k in range(3):
            if force[k]:
                lines.append(
                    f"        for j in 0..N {{ out[j] += jx[{k}][j] * {force[k]:.17e}; }}"
                )
        lines.append("    }")
    lines += [
        "    let mut values = [0.0f64; M];",
        "    let mut jac = [0.0f64; N*M];",
        "    compute_inner(q, g, t, &mut values, &mut jac);",
        "    for s in 0..S {",
        "        let row = SPRING_ROWS[s];",
        "        let compliance = g[SPRING_G[s]];",
        "        let tension = if compliance != 0.0 { values[row] / compliance } else { 0.0 };",
        "        let mut rate = 0.0;",
        "        for i in 0..N { rate += jac[row * N + i] * v[i]; }",
        "        let force = tension + SPRING_DAMPING[s] * rate;",
        "        for i in 0..N { out[i] -= force * jac[row * N + i]; }",
        "    }",
        "}",
        "",
        "/// `gamma = -d(Phi_q qdot)/dt`, at constant q.",
        "fn gamma(q: &[f64; N], v: &[f64; N], g: &[f64; G], t: &[f64; A], out: &mut [f64; R]) {",
        "    let mut speed = 0.0;",
        "    for i in 0..N { speed += v[i] * v[i]; }",
        "    let speed = speed.sqrt();",
        "    if speed == 0.0 {",
        "        for r in 0..R { out[r] = 0.0; }",
        "        return;",
        "    }",
        "    let step = 1e-6 / speed;",
        "    let mut qp = *q;",
        "    let mut qm = *q;",
        "    for i in 0..N { qp[i] += step * v[i]; qm[i] -= step * v[i]; }",
        "    let mut values = [0.0f64; M];",
        "    let mut jacp = [0.0f64; N*M];",
        "    let mut jacm = [0.0f64; N*M];",
        "    compute_inner(&qp, g, t, &mut values, &mut jacp);",
        "    compute_inner(&qm, g, t, &mut values, &mut jacm);",
        "    for r in 0..R {",
        "        let row = CONSTRAINED[r];",
        "        let mut acc = 0.0;",
        "        for i in 0..N { acc += (jacp[row * N + i] - jacm[row * N + i]) / (2.0 * step) * v[i]; }",
        "        out[r] = -acc;",
        "    }",
        "}",
        "",
        "/// One acceleration, by the saddle-point system.",
        "fn accel(q: &[f64; N], v: &[f64; N], g: &[f64; G], t: &[f64; A], out: &mut [f64; N], lam: &mut [f64; R]) {",
        "    let mut m = [0.0f64; N*N];",
        "    for b in 0..NB {",
        "        let block = body_block(b, q);",
        "        let s = BODY_START[b];",
        "        for r in 0..7 { for c in 0..7 { m[(s + r) * N + (s + c)] = block[r][c]; } }",
        "    }",
        "    let mut f = [0.0f64; N];",
        "    forces(q, v, g, t, &mut f);",
        "    let mut c = [0.0f64; N];",
        "    velocity_products(q, v, &mut c);",
        "    for i in 0..N { f[i] -= c[i]; }",
        "    let mut values = [0.0f64; M];",
        "    let mut jac = [0.0f64; N*M];",
        "    compute_inner(q, g, t, &mut values, &mut jac);",
        "    let mut gam = [0.0f64; R];",
        "    gamma(q, v, g, t, &mut gam);",
        "    let mut a = [0.0f64; K*K];",
        "    for i in 0..N { for j in 0..N { a[i * K + j] = m[i * N + j]; } }",
        "    for r in 0..R {",
        "        let row = CONSTRAINED[r];",
        "        for j in 0..N {",
        "            let value = jac[row * N + j];",
        "            a[j * K + (N + r)] = value;",
        "            a[(N + r) * K + j] = value;",
        "        }",
        "        a[(N + r) * K + (N + r)] = -EPS;",
        "    }",
        "    let mut rhs = [0.0f64; K];",
        "    for i in 0..N { rhs[i] = f[i]; }",
        "    for r in 0..R { rhs[N + r] = gam[r]; }",
        "    if !solve_linear(K, &mut a, &mut rhs) {",
        "        for i in 0..N { out[i] = f64::NAN; }",
        "        for r in 0..R { lam[r] = f64::NAN; }",
        "        return;",
        "    }",
        "    for i in 0..N { out[i] = rhs[i]; }",
        "    for r in 0..R { lam[r] = rhs[N + r]; }",
        "}",
        "",
        "/// Put the state back on the manifold, by the smallest correction that does it.",
        "fn project(q: &mut [f64; N], v: &mut [f64; N], g: &[f64; G], t: &[f64; A], out: &mut [f64; N]) {",
        "    for _ in 0..4 {",
        "        let mut values = [0.0f64; M];",
        "        let mut jac = [0.0f64; N*M];",
        "        compute_inner(q, g, t, &mut values, &mut jac);",
        "        let mut worst = 0.0f64;",
        "        for r in 0..R { let a = values[CONSTRAINED[r]].abs(); if a > worst { worst = a; } }",
        "        if worst < 1e-13 { break; }",
        "        let mut gram = [0.0f64; R*R];",
        "        let mut rhs = [0.0f64; R];",
        "        let mut scale = 0.0f64;",
        "        for r in 0..R {",
        "            for c in 0..R {",
        "                let mut acc = 0.0;",
        "                for i in 0..N { acc += jac[CONSTRAINED[r] * N + i] * jac[CONSTRAINED[c] * N + i]; }",
        "                gram[r * R + c] = acc;",
        "                if acc > scale { scale = acc; }",
        "            }",
        "            rhs[r] = -values[CONSTRAINED[r]];",
        "        }",
        "        let eps = 1e-12 * scale;",
        "        for r in 0..R { gram[r * R + r] += eps; }",
        "        if !solve_linear(R, &mut gram, &mut rhs) { break; }",
        "        for i in 0..N {",
        "            let mut step = 0.0;",
        "            for r in 0..R { step += jac[CONSTRAINED[r] * N + i] * rhs[r]; }",
        "            q[i] += step;",
        "        }",
        "    }",
        "    for _ in 0..2 {",
        "        let mut values = [0.0f64; M];",
        "        let mut jac = [0.0f64; N*M];",
        "        compute_inner(q, g, t, &mut values, &mut jac);",
        "        let mut drift = [0.0f64; R];",
        "        let mut worst = 0.0f64;",
        "        for r in 0..R {",
        "            let mut acc = 0.0;",
        "            for i in 0..N { acc += jac[CONSTRAINED[r] * N + i] * v[i]; }",
        "            drift[r] = acc;",
        "            if acc.abs() > worst { worst = acc.abs(); }",
        "        }",
        "        if worst < 1e-13 { break; }",
        "        let mut gram = [0.0f64; R*R];",
        "        let mut rhs = [0.0f64; R];",
        "        let mut scale = 0.0f64;",
        "        for r in 0..R {",
        "            for c in 0..R {",
        "                let mut acc = 0.0;",
        "                for i in 0..N { acc += jac[CONSTRAINED[r] * N + i] * jac[CONSTRAINED[c] * N + i]; }",
        "                gram[r * R + c] = acc;",
        "                if acc > scale { scale = acc; }",
        "            }",
        "            rhs[r] = -drift[r];",
        "        }",
        "        let eps = 1e-12 * scale;",
        "        for r in 0..R { gram[r * R + r] += eps; }",
        "        if !solve_linear(R, &mut gram, &mut rhs) { break; }",
        "        for i in 0..N {",
        "            let mut step = 0.0;",
        "            for r in 0..R { step += jac[CONSTRAINED[r] * N + i] * rhs[r]; }",
        "            v[i] += step;",
        "        }",
        "    }",
        "    for i in 0..N { out[i] = q[i]; }",
        "}",
        "",
        "/// Renormalise every quaternion: a linear combination of unit quaternions is not one, and",
        "/// the RK4 stages are linear combinations.",
        "#[inline]",
        "fn gauge(q: &mut [f64; N]) {",
        "    for b in 0..NB {",
        "        let s = BODY_START[b] + 3;",
        "        let mut norm = 0.0;",
        "        for k in 0..4 { norm += q[s + k] * q[s + k]; }",
        "        let norm = norm.sqrt();",
        "        if norm != 0.0 { for k in 0..4 { q[s + k] /= norm; } }",
        "    }",
        "}",
        "",
        "fn derivative(q: &[f64; N], v: &[f64; N], g: &[f64; G], t: &[f64; A], dq: &mut [f64; N], dv: &mut [f64; N]) {",
        "    let mut stage = *q;",
        "    gauge(&mut stage);",
        "    let mut lam = [0.0f64; R];",
        "    accel(&stage, v, g, t, dv, &mut lam);",
        "    for i in 0..N { dq[i] = v[i]; }",
        "}",
        "",
        "/// The acceleration at the current state. Returns its largest component.",
        "#[no_mangle]",
        'pub extern "C" fn accelerations() -> f64 {',
        "    let mut q = [0.0f64; N];",
        "    let mut v = [0.0f64; N];",
        "    let mut g = [0.0f64; G];",
        "    let mut t = [0.0f64; A];",
        "    unsafe {",
        "        for i in 0..N { q[i] = MEM[O_Q + i]; }",
        "        for i in 0..N { v[i] = MEM[O_V + i]; }",
        "        for i in 0..G { g[i] = MEM[O_G + i]; }",
        "        for i in 0..A { t[i] = MEM[O_T + i]; }",
        "    }",
        "    let mut out = [0.0f64; N];",
        "    let mut lam = [0.0f64; R];",
        "    accel(&q, &v, &g, &t, &mut out, &mut lam);",
        "    unsafe { for i in 0..N { MEM[O_ACC + i] = out[i]; } }",
        "    dyn_maxabs(&out)",
        "}",
        "",
        "/// Integrate from the current state: RK4 with a projection after every step.",
        "/// Returns the worst constraint violation seen, or -1 if the state left the numbers.",
        "#[no_mangle]",
        'pub extern "C" fn integrate(steps: i32) -> f64 {',
        "    let steps = if steps as usize > ALLOC { ALLOC } else { steps as usize };",
        "    if steps < 2 { return -1.0; }",
        "    let mut q = [0.0f64; N];",
        "    let mut v = [0.0f64; N];",
        "    let mut g = [0.0f64; G];",
        "    let mut t = [0.0f64; A];",
        "    unsafe {",
        "        for i in 0..N { q[i] = MEM[O_Q + i]; }",
        "        for i in 0..N { v[i] = MEM[O_V + i]; }",
        "        for i in 0..G { g[i] = MEM[O_G + i]; }",
        "        for i in 0..A { t[i] = MEM[O_T + i]; }",
        "    }",
        "    gauge(&mut q);",
        "    let mut worst = 0.0f64;",
        "    for step in 0..steps {",
        "        if step > 0 {",
        "            let mut k1q = [0.0f64; N];",
        "            let mut k1v = [0.0f64; N];",
        "            derivative(&q, &v, &g, &t, &mut k1q, &mut k1v);",
        "            let mut q2 = q;",
        "            let mut v2 = v;",
        "            for i in 0..N { q2[i] += 0.5 * DT * k1q[i]; v2[i] += 0.5 * DT * k1v[i]; }",
        "            let mut k2q = [0.0f64; N];",
        "            let mut k2v = [0.0f64; N];",
        "            derivative(&q2, &v2, &g, &t, &mut k2q, &mut k2v);",
        "            let mut q3 = q;",
        "            let mut v3 = v;",
        "            for i in 0..N { q3[i] += 0.5 * DT * k2q[i]; v3[i] += 0.5 * DT * k2v[i]; }",
        "            let mut k3q = [0.0f64; N];",
        "            let mut k3v = [0.0f64; N];",
        "            derivative(&q3, &v3, &g, &t, &mut k3q, &mut k3v);",
        "            let mut q4 = q;",
        "            let mut v4 = v;",
        "            for i in 0..N { q4[i] += DT * k3q[i]; v4[i] += DT * k3v[i]; }",
        "            let mut k4q = [0.0f64; N];",
        "            let mut k4v = [0.0f64; N];",
        "            derivative(&q4, &v4, &g, &t, &mut k4q, &mut k4v);",
        "            for i in 0..N {",
        "                q[i] += DT / 6.0 * (k1q[i] + 2.0 * k2q[i] + 2.0 * k3q[i] + k4q[i]);",
        "                v[i] += DT / 6.0 * (k1v[i] + 2.0 * k2v[i] + 2.0 * k3v[i] + k4v[i]);",
        "            }",
        "            let mut scratch = [0.0f64; N];",
        "            project(&mut q, &mut v, &g, &t, &mut scratch);",
        "            gauge(&mut q);",
        "        }",
        "        let mut values = [0.0f64; M];",
        "        let mut jac = [0.0f64; N*M];",
        "        compute_inner(&q, &g, &t, &mut values, &mut jac);",
        "        for r in 0..R {",
        "            let a = values[CONSTRAINED[r]].abs();",
        "            if a > worst { worst = a; }",
        "        }",
        "        unsafe {",
        "            for i in 0..N {",
        "                MEM[O_TRAJ + step * N + i] = q[i];",
        "                MEM[O_TRAJV + step * N + i] = v[i];",
        "            }",
        "        }",
        "    }",
        "    unsafe {",
        "        for i in 0..N { MEM[O_Q + i] = q[i]; }",
        "        for i in 0..N { MEM[O_V + i] = v[i]; }",
        "    }",
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
    lbj = ubj = None
    foot_lower = foot_upper = None
    if structure is not None:
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
    for name in dict.fromkeys([axle_a, axle_b, *([lbj, ubj] if lbj else [])]):
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
    ]
    if structure is None:
        # No rigid front view to construct: the upper joint is compliant, so the arm's hinge line
        # does not locate a ball joint and the instant centre is undefined rather than wrong.
        # NaN is the honest value, and the glue turns it into null the way Python's outputs do.
        lines += [
            "    let ic = [f64::NAN, f64::NAN];",
            "    let rc = [f64::NAN, f64::NAN];",
        ]
    else:
        lines += [
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
        ]
    lines += [
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


DYNAMICS_GLUE = """
/** Set the velocity state, one number per pose unknown. */
export function setVelocity(values) {
  const mem = memory();
  values.forEach((value, i) => { mem[at('v') + i] = value; });
}

/** The velocity state. */
export function velocity() {
  return Array.from(memory().subarray(at('v'), at('v') + S.q));
}

/** The acceleration at the current state, and its largest component. */
export function accelerations() {
  const residual = instance.exports.accelerations();
  return { residual, acceleration: Array.from(memory().subarray(at('acc'), at('acc') + S.q)) };
}

/** Integrate from the current state. Returns the worst constraint violation. */
export function integrate(steps) {
  return instance.exports.integrate(steps ?? manifest.integration.steps);
}

/** The poses the last integration wrote, one per step. */
export function trajectory(steps) {
  const mem = memory();
  const count = steps ?? manifest.integration.steps;
  const out = [];
  for (let k = 0; k < count; k += 1) {
    out.push(Array.from(mem.subarray(at('traj') + k * S.q, at('traj') + (k + 1) * S.q)));
  }
  return out;
}

/** The velocities the last integration wrote, one per step. */
export function velocities(steps) {
  const mem = memory();
  const count = steps ?? manifest.integration.steps;
  const out = [];
  for (let k = 0; k < count; k += 1) {
    out.push(Array.from(mem.subarray(at('trajv') + k * S.q, at('trajv') + (k + 1) * S.q)));
  }
  return out;
}

"""


def _js_glue(manifest: dict, module_b64: str) -> str:
    layout = manifest["layout"]
    # The dynamics helpers read buffers only a dynamics kernel has, so they are emitted only for
    # one: a kinematic module asked for `at('v')` would read someone else's memory.
    dynamics = manifest.get("solver") == "dynamics"
    dynamics_glue = DYNAMICS_GLUE if dynamics else ""
    dynamics_api = (
        "  setVelocity, velocity, accelerations, integrate, trajectory, velocities,\n"
        if dynamics
        else ""
    )
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

{dynamics_glue}const api = {{
  manifest, ready, design, setGeometry, setDrive, solve, pose, evaluate, metrics, world, frame,
{dynamics_api}}};
export default api;
"""
