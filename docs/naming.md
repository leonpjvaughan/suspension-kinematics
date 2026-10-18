# Naming, layout and the command line

**Status:** design, not yet implemented. This document decides what the thing is called, how
its modules are arranged, and what its verbs are. The substance is in
[`config-layer.md`](config-layer.md), [`problem-mapping.md`](problem-mapping.md) and
[`entry-points.md`](entry-points.md); this is the wrapper.

## 1. What it is, in one line

**Config files in, mathematical problems out.**

It reads a mechanism's structure and numbers, derives the residual and its Jacobian
symbolically, and hands them to a solver or an emitter. Suspension geometry is the first
consumer, not the subject: nothing in the config layer knows what a wishbone is, and the
`plane-gap` kind is as happy with a train pantograph as with a tyre.

## 2. The name

**Recommendation: `otkd` everywhere** — distribution, import, and command. Prose expands it
as *open textbook kinematics and dynamics* on first use, and then never again.

The virtue of one name in three places is that every other option produces three spellings
to keep in sync, and the two failures people actually hit are `pip install` versus `import`
and the command versus the import. `otkd` is short, distinct, and already yours.

| candidate | distribution / import / command | for | against |
|---|---|---|---|
| **`otkd`** | `otkd` / `otkd` / `otkd` | one name; short; expands cleanly | opaque until expanded; ties the name to a book |
| `ot-kin-dyn` | `ot-kin-dyn` / `ot_kin_dyn` / `otkd` | the expansion is in the name | three spellings, a long import, and a hyphen/underscore trap |
| `otk` | `otk` / `otk` / `otk` | shortest; leaves room for a sibling `otk-dynamics` | drops *dynamics* from the name, which is half the ambition; three-letter names collide |
| a neutral noun (`rig`, `bench`, `linkage`) | — | outlives the book; reads as a tool | loses the tie to the book, is vaguer about what it does, and is the kind of name that is already taken |

The honest tension: `otkd` says *textbook* in a package that is deliberately general. Two
things make it acceptable. The import name and the book tie are not the same commitment — a
distribution can be renamed later while the import and command stay `otkd`, or vice versa —
and the first consumer really is a textbook whose argument is that the model is derived
rather than transcribed. If the general use ever dominates, rename the *project* and keep the
import. Do not rename the import twice.

**What the rename touches that a rename should not:**

- `HANDOFF.md` predates it and keeps its own wording; where it names the package, read
  `otkd`. The repository name is the owner's to change.
- The export's `meta.generator` becomes `otkd <version>` (`HANDOFF.md` §7). That is a change
  to a **value** in a frozen contract, so it is recorded here: a consumer that prints the
  generator string will print a different string, and should be reading the version rather
  than the name.
- Nothing is published to PyPI in v1 (`HANDOFF.md` §2), so availability is not a blocker and
  the name is not yet a claim on a namespace.

## 3. One distribution, packages underneath

A namespace of separately versioned distributions (`otkd-core`, `otkd-ui`, …) buys
independent release cadence, which is not a problem anyone has yet. It also costs a
compatibility matrix and a decision per change about which distribution it belongs to. So:
**one distribution now, with the module layout already divided along the seams a split would
use.** Splitting later is then a packaging change, not a refactor.

```
src/otkd/
  __init__.py          # the five public names: load, check, compile, solve, emit
  errors.py            # ConfigError (a list of faults), SolverError
  report.py            # `check`: the design position, the rank, the redundancy
  config/              # what a model is
    kinds.py           #   families, ConstraintKind, library loading
    topology.py        #   BodySpec, Site, Joint, Actuator, Roles, Topology
    geometry.py        #   points, axes, joint numbers, scalars, dynamics block
    study.py           #   analysis, drivers, laws, loads, drop, seed
    model.py           #   the four loaded files, resolution, the validation rules
  problem/             # config -> mathematics  (the first half of the job)
    rows (in compile.py)
    compile.py         #   symbols, rows, Jacobian, potential, the cache
  solvers/             # mathematics -> numbers  (the second half)
    system.py          #   the row partitions for one frame
    newton.py  least_squares.py  equilibrium.py  svd.py  sweep.py
  analysis/            # numbers -> quantities a figure draws
    outputs.py         #   camber, toe, instant centre, roll centre, track, contact
    analytic.py        #   closed forms, the independent cross-check
  emitters/            # numbers -> artefacts
    js.py              #   the payload and the ES module
    wasm.py            #   the generated Rust/WASM kernel
  cli/
    main.py            # check, compile, solve, emit, kinds
```

**Module names may not collide with the verbs.** `otkd/solve.py` would be shadowed by the
`otkd.solve()` function the moment it was imported, and vice versa, because a package attribute
wins over a function of the same name. Hence `solvers/`, `emitters/`, `report.py`. The public
names are the five verbs and nothing else.

The two halves are the point: `problem/` is where a config becomes mathematics and is the
part that must be right; `solve/` is where numbers happen and is the part that may be
replaced. A future Rust engine would sit *behind* `solve/`'s interface and would have to
reproduce `emit`'s bytes, not just plausible numbers.

If a split is ever needed, the natural seam is `otkd-ui` (an optional wheel that depends on
`otkd`) — not core/CLI, which ship together or not at all.

## 4. The verbs

The command line is the pipeline, one verb per stage:

| verb | stage | what it does |
|---|---|---|
| `otkd check <model>` | config | parse, resolve names, verify the design position, print rank / mobility / redundancy |
| `otkd compile <model>` | → mathematics | print the problem: residual rows, units, Jacobian, sparsity; `--format text, latex, json` |
| `otkd solve <model>` | → numbers | run the study; a table, or `--json`; `--at VALUE` for one frame |
| `otkd emit <model> <out>` | → artefact | write it; `--format js, json` (default `js`, the book's contract) |
| `otkd kinds <model>` | — | the library as your model resolves it |
| `otkd ui <model>` | — | the config UI ([`ui.md`](ui.md)) |

Renamed from the first draft, and why:

| was | now | reason |
|---|---|---|
| `validate` | `check` | it is a pass/fail gate with a report, not a ceremony; `check` fits a script |
| `run` | `solve` | the package's job is to *solve* the problem it compiled; `run` says nothing |
| `export` | `emit` | an emitter produces one artefact from a compiled problem; what the artefact *is* becomes a flag |
| `kinds` | `kinds` | unchanged: short and precise |
| — | `compile` | the half of the job that was invisible, and the one a textbook wants to show |
| — | `ui` | the interactive front end |

`compile` is worth the extra verb for a reason beyond symmetry: it is the command that lets a
reader see the residual the solver actually differentiates, and `--format latex` prints it as
the book's equations. Generated equations, not transcribed ones, is the book's claim.

## 5. The engine boundary

Every front end — the CLI, the UI, a notebook, anything later — speaks the same contract:
**files in, JSON out.**

```
check(files)              -> report
compile(files)            -> problem      (rows, units, Jacobian, ranks)
solve(files, study)       -> payload      (frames, outputs, history, agreement)
emit(payload, format)     -> artefact     (js, json, latex)
```

Python is the reference implementation of the first three; `emit` is a formatter over
`solve`'s payload. That is what makes `otkd solve --json`, `otkd emit out.js` and the UI
unable to disagree: one producer, three wrappers. It is also the boundary a Rust or
JavaScript engine would have to meet, if one is ever written
([`ui.md`](ui.md) §6).

## 6. Deliberately not decided here

- **A `version` in the CLI** — there is one source of truth, the installed distribution's
  version, and `otkd --version` reads it.
- **Short aliases** (`otkd s`, `otkd c`). One name per verb until someone complains.
- **A REPL, a daemon, or a language server.** The API is the answer for programmatic use.
