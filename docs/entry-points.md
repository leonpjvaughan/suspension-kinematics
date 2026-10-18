# Entry points: your config, your files

The package has no model, no library and no state of its own. It reads four files that you
supply, derives the problem from them, and writes one artefact where you ask it to. The
`examples/` directory is documentation: a working library and four models you can copy.

This document is the surface a user meets: what you provide, the commands, the Python API,
and what happens when a file is wrong. The names themselves — and why they are those names —
are in [`naming.md`](naming.md); the interactive front end is in [`ui.md`](ui.md).

---

## 1. What you provide

```
my-corner/
  topology.yaml       # bodies, sites, joints, actuators — no numbers
  geometry.yaml       # every number
  study.yaml          # which actuator is driven, with what law; loads; what is dropped
  constraints.yaml    # what `spherical`, `revolute`, `distance` … mean
  out.js              # generated, if you want the ES module
```

`constraints.yaml` may be shared between models; it is a library, not a model file. The
examples do exactly that — one library, four models:

```
examples/
  constraints.yaml               # shared by everything below
  double-wishbone/               topology.yaml geometry.yaml study.yaml out.js
  double-wishbone-two-leg/       …
  double-wishbone-compliant/     …
  ride-height/                   topology.yaml geometry.yaml study.yaml
```

**Resolution.** `topology.constraints` defaults to `constraints.yaml` beside the topology
file and may name any path; the path is resolved **relative to the topology file**, never
the working directory, so a model behaves the same however it is invoked. A missing library
is an error that names both the path it tried and the key that changes it. There is no
fallback: a model without its library is not a model.

**Version.** Each file may carry `version: 1`. Omitted means 1; a version this build does
not know is refused by name. That is the one piece of future-proofing here, and it exists
because your files outlive a package upgrade.

---

## 2. Command line

Installed as `otkd` (`[project.scripts]`). The verbs are the pipeline, one per stage:

| command | does |
|---|---|
| `otkd check <model> [--study PATH]` | parse, resolve names, print every row's residual at the design position, then rank, mobility and redundancy |
| `otkd compile <model> [--format text\|latex\|json]` | print the problem itself: residual rows, units, Jacobian, sparsity |
| `otkd solve <model> [--study PATH] [--at VALUE] [--json PATH] [--csv PATH]` | run the study, or one frame with `--at` |
| `otkd emit <model> <out> [--format js\|json\|wasm] [--study PATH]` | write the artefact. `js` is the book's contract (`HANDOFF.md` §7); `wasm` writes a directory containing a generated Rust/WASM kernel ([`wasm.md`](wasm.md)) |
| `otkd kinds <model>` | the kinds *your* library defines, with each component's mode and row count |
| `otkd ui <model>` | the config UI ([`ui.md`](ui.md)) |

`--study` defaults to `<model>/study.yaml`, which is what makes "one model, several
questions" work: a second study file in the same directory is a second sweep, a load case or
a stiffness ladder, and no command has to learn about it. `--at` exists because the UI needs
one frame per drag, not eighty-one.

```console
$ otkd check examples/double-wishbone
examples/double-wishbone
  constraints   ../constraints.yaml (10 kinds)
  bodies        4 (3 moving; fixed: ground)
  rows
    lca-ground.tx  locked  0.000e+00  m
    lca-ground.ty  locked  0.000e+00  m
    …
    tie-rod.length locked  0.000e+00  m
    lca-ground.rz  driven  0.000e+00  rad
  jacobian      18 x 18   rank 18   redundancy 0
  mobility      1          actuators 1   ok
```

```console
$ otkd solve examples/double-wishbone
theta      camber     toe      track    ic          rc
-0.3500    -2.1134    0.0412  -0.0123  [0.81,0.44] [0.00,0.06]
-0.3413    -2.0531    0.0401  -0.0117  [0.81,0.45] [0.00,0.06]
…
```

`solve --json` emits the same object `emit --format js` serialises into an ES module. One
producer, two wrappers: a terminal and a figure cannot disagree about a number.

**Exit codes:** `0` success, `1` a configuration error (the report is printed and nothing is
written), `2` a solver failure (the frame, the last residual and the worst row by name). The
codes exist so `solve` can sit in a script or a CI job without parsing prose.

---

## 3. Python API

Five names at the top level, plus the compiled problem for the cases that need it.

| name | takes | gives |
|---|---|---|
| `load(path, *, study=None)` | a model directory (or a topology path) | `Model` — topology, geometry, study, and the resolved kinds |
| `check(model)` | `Model` | `Report`; raises `ConfigError` on anything that would make the solve meaningless |
| `compile(model)` | `Model` | `Problem` — the symbolic rows and Jacobian, with **geometry left as arguments**, cached by topology and library |
| `solve(model)` | `Model` | `Sweep` — frames, outputs, branch labels |
| `emit(model, path)` | `Model`, a path | writes the artefact — `solve` plus a formatter |
| `Problem.solve(geometry, study=None)` | numbers | `Sweep`, reusing the same derived functions |
| `Model.with_geometry(geometry)` | a `Geometry` | the same model against different numbers |

```python
import otkd

model = otkd.load("examples/double-wishbone")
report = otkd.check(model)
print(report.mobility, report.redundancy)

frames = otkd.solve(model)
print(frames[-1].camber, frames[-1].ic)

otkd.emit(model, "examples/double-wishbone/out.js")
```

**Why `compile` is separate from `load`.** The symbolic derivation contains the topology,
not the numbers: points, axes, lengths, gaps and compliances are *parameters* of the
lambdified functions. So a design study compiles once and then solves many geometries:

```python
base = otkd.load("examples/double-wishbone")
problem = otkd.compile(base)                # 10.5 s, once

for design in designs:                      # Geometry objects, not files
    frames = problem.solve(design)          # 1 ms of derivation, then the sweep
    print(design.id, frames[-1].camber)
```

`Model.with_geometry(design)` is the same thing spelled the other way round, for code that
already thinks in models.

That is the same promise the config split makes — one topology, many geometries — kept
literally rather than approximately. It is also the reason `solve` is cheap enough to sit
behind a CLI: the slow step is per topology, the fast step is per sweep.

The advanced surface (`Problem.rows`, `.jac`, `.diagnose`) is a function of `(q, geometry)`
and is what `newton`, `least_squares` and `svd_diagnose` consume
([`problem-mapping.md`](problem-mapping.md) §4–§6). It is public because a reader of the
book may want to step the solver by hand, not because the package needs it.

---

## 4. When a file is wrong

The loader refuses rather than repairs, and every message names the file, the key and what
it expected:

```
geometry.yaml: joints.tie-rod.length = 0.371000
  topology.yaml: joint `tie-rod` (distance) is at 0.370673 in the design position
  fix: change `length` in geometry.yaml, or delete it to have it default
```

A configuration error is a list of such lines, not the first one found, so one run fixes a
draft instead of revealing a typo per attempt. `ConfigError` carries them as structured
records (`file`, `key`, `message`), which is what `check` prints and what a notebook can
iterate. Solver failures are a different type (`SolverError`) and a different exit code,
because "your model is malformed" and "your model has no solution here" are different
problems.

The four rules that catch the expensive mistakes, stated once: the design position must
satisfy every hard and compliant row; the rank of the Jacobian must equal the unknowns minus
the actuators; a dropped row must have been redundant; and an actuator must target a
free, non-compliant component. Everything else is name resolution
([`config-layer.md`](config-layer.md) §9).

---

## 5. Deliberately not entry points

- **No plugin system.** A new constraint *family* is code (three expressions); a new *kind*
  is data, and data is what `constraints.yaml` is for.
- **No `--set` overrides in v1.** A study file is cheap to copy and reviewable in a diff; a
  command-line override is a configuration that exists only in a shell history. The Python
  API is the answer for programmatic sweeps.
- **No server in the CLI sense, and no notebook magic.** `ui` starts a loopback server for
  the length of one session and nothing else ([`ui.md`](ui.md) §4).
- **No Pyodide build.** The seam for it is one comment in `emit/js.py`
  (`HANDOFF.md` §1), not an entry point.
- **No scaffolding command in v1.** Copying `examples/<name>/` is the scaffolding story,
  and it has the advantage of showing a model that already validates. `ui`'s *Save as* is
  the eventual replacement.
