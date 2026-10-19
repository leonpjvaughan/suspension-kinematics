# otkd

**Open textbook kinematics and dynamics.** Config files in, mathematical problems out.

A **mechanism** is described by three files and a library. `topology.yaml` says which bodies
exist, how they are joined and driven — no numbers at all. `geometry.yaml` holds every
number. `study.yaml` says which actuator moves, over what, under what load. A
`constraints.yaml` of your own says what `spherical`, `revolute` or `plane-gap` *mean*.

The package reads those four files, derives the residual and its Jacobian symbolically, and
hands them to a solver or an emitter. One topology can be swept over many geometries, so a
design study is a loop over numbers rather than a rewrite of constraints.

It exists to serve a book about *modelling*, whose first subject is a suspension corner: one
rigid triangle, then the same wishbone as two legs so the model is redundant, then the SVD
that diagnoses the redundancy, then a bush where a rigid constraint was. The numbers those
chapters draw are produced here as a static ES module.

## Status

Implemented: the config layer, the symbolic derivation, Newton, least squares, SVD
diagnosis, the compliant equilibrium, the sweep, the outputs, the ES-module emitter, a
generated Rust/WASM kernel, the CLI, four examples and the acceptance tests.
`uv run pytest -q` passes.

Versioning and releases: Conventional Commits decide the version, a merge to `main` releases
it, and the compiled bundle is attached to the GitHub release for the textbook site to vendor.
See [`docs/releasing.md`](docs/releasing.md).

Designed but not built: dynamics ([`docs/dynamics.md`](docs/dynamics.md)) and the
interactive config UI ([`docs/ui.md`](docs/ui.md), held back deliberately). The compiled
kernel that the browser UI and the book's interactive figures would share is built —
[`docs/wasm.md`](docs/wasm.md).

The handoff predates the rename and keeps its own wording; the repository name is still
`suspension-kinematics`.

## Using it

```console
$ otkd check examples/double-wishbone          # names, design position, rank, mobility
$ otkd compile examples/double-wishbone --format latex
$ otkd solve examples/double-wishbone --json out.json
$ otkd emit examples/double-wishbone out.js          # the static payload
$ otkd emit examples/double-wishbone assets/kernel --format wasm  # a solver for a page
$ otkd kinds examples/double-wishbone         # what your library means, component by component
```

```python
import otkd

model = otkd.load("examples/double-wishbone")
otkd.check(model)
frames = otkd.solve(model)
otkd.emit(model, "examples/double-wishbone/out.js")
```

`compile` is separate from `load` because the geometry is a parameter of the derived
functions, not part of them: a design study pays for the derivation once.

```python
base = otkd.load("examples/double-wishbone")
problem = otkd.compile(base)
for design in designs:
    frames = problem.solve(design)
```

## The examples

| directory | what it demonstrates |
|---|---|
| `double-wishbone` | §1: one rigid body per arm, one degree of freedom, square Newton, agreed with a closed form to 1e-13 |
| `double-wishbone-two-leg` | §2 and §3: the upper arm as two legs, redundant by one, least squares, and the SVD that says why |
| `double-wishbone-compliant` | §4: the tie rod as a compliance under load; the stiffness ladder converges to rigid |
| `ride-height` | the same corner with the chassis inertial and the road driven, so the sweep is ride height |
| `quarter-car` | a dynamics model: a heaving chassis, a coil-over, and the tyre held on a road plane |

`study-square.yaml` in the two-leg example is the same model with the dependent row dropped.

## Sign conventions

* **camber**, degrees: positive when the top of the wheel leans outboard, `asin(-n_z)` from
  the axle `n`.
* **toe**, degrees: change in the axle's plan-view heading from the design position, positive
  for toe-in.
* **track**, metres: lateral movement of the contact patch, outboard positive.
* **instant centre**: front-view intersection of the lines through each ball joint and its
  arm's inner pivot; `null` when they are parallel.
* **roll centre**: where the line from the contact patch through the instant centre crosses
  the centreline; `null` when undefined.

Coordinates are metres, x lateral (positive outboard for the corner modelled), y forward,
z up, at the design position. Every body's local frame *is* the world frame at design.

## Limits, stated up front

Rigid bodies and small deformations: no friction, no tyre deflection, no dynamics, no
compliance that is not a linear spring. It answers questions about where a linkage *is*, and
about how a chosen model earns its place — not about how a car behaves. That is what it is
for, and it is not an apology.

## Design record

[`docs/`](docs/README.md) holds the specification of the config layer, the mapping from a
model to each chapter's problem, the dynamics design, the entry points, and
[`decisions.md`](docs/decisions.md) with the reasoning for every choice — including the ones
the implementation changed.

## Licence

MIT. See [`LICENSE`](LICENSE).
