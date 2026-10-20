# Accepted follow-ups

Work that has been agreed and deliberately not done yet. Each entry says what it is, why it is
deferred, and what would make it done — so picking one up does not require reconstructing the
argument.

## Compiled kernels, stage 2

The kernel currently compiles the square Newton path. The emitter *refuses* a redundant or
compliant model with a message rather than emitting a solver for the wrong equations, so the
gap is loud. Three increments close it, in this order.

## Dynamics

v2.0 (accelerations and reactions) and v2.1 (time stepping with projection, the energy test, and
a `dynamics` study shape with a CLI path) are implemented and tested. Remaining, per
[`dynamics.md`](dynamics.md) §7: v2.2 — four corners, a chassis body and a contact-patch load.

## A compliant upper joint, and a stiffness axis

Requested by `maths-site` ([`handoff-from-maths-site.md`](handoff-from-maths-site.md) §1) and
accepted in the package's shape, not the book's. **Measured, and it is not the config change it
looked like.**

**A compliant spherical joint works, and the ladder is real.** No new kind is needed: a per-joint
`modes:` override says `{tx: spring, ty: spring, tz: spring}` on `uca-up`, and `geometry.yaml` gives
the three compliances. The equilibrium was the wall — the projected formulation could not converge
for three compliant components at one frame joint, and the KKT formulation can
([`decisions.md`](decisions.md) round 23). Measured on the book's own question, with the loads the
compliant example carries:

| stiffness | camber over the travel | at full bump |
|---|---|---|
| 0.2 MN/m | 2.0828° | −0.2378° |
| 1 MN/m | 2.0397° | −0.9531° |
| 10 MN/m | 2.0291° | −1.1112° |
| 100 MN/m | 2.0280° | −1.1270° |

Monotone, converging to the rigid answer, and the stiffness that matters is the ball joint's — which
is the ladder the book could not produce from a compliant *tie rod*.

What is left is the export shape: a model in `examples/` with that joint, and a stiffness axis, so
that one run yields the ladder rather than four.

**A stiffness axis is a study change, not an example.** A number iterated *outside* the travel
driver, so one run produces the ladder; each inner sweep seeded from the **design pose**, because a
soft joint and a stiff joint are different branches and `solvers/sweep.py`'s predecessor seeding is
for continuity within a sweep. `Sweep.frames` carries one `primary` value today. The classifier is
unaffected (rank is a property of the model) and the kernel needs no ABI change — it already takes
the compliance as a number slot.

Done means: an example sweeps stiffness and its export reports camber at rest and over the travel
for each value. The equilibrium converging, and camber responding, are done and measured above. If it does not, the answer is no —
an approximation built from a compliant *distance* joint would give a ladder of the right shape and
the wrong physics, which is worse than no ladder.

## The UI

Deferred, and possibly never built. [`ui.md`](ui.md) is the design if it is revived; the
condition for starting is that configuring a corner by hand becomes slower than the vertical
slice (load a model, drag a point, front view, check badge). Nothing in the package depends on
it.

## Polish, accepted

| item | why it matters |
|---|---|
| Display names in `otkd compile --format latex` | the symbols are `g_{lcaf 0}`; the command exists to typeset the book's equations, so the names should read as grammar does |
| An end-to-end CLI test | the library is covered; `check`, `solve` and `emit` are exercised by hand and by CI shell steps, not by a test |
| A sanity check on the toe curve | the steering arm was lengthened until bump steer was believable (±0.6°); it wants an engineer's eye, not a test |

## Done

- **The book's geometry** is reconciled on its side at 0.4.0: `maths-site` adopted
  `tyre-radius = 0.200` and the ground plane it implies, and its roll centre agrees with the export
  to 1 mm. Their words: the entry can be closed from where they sit.
- **A trajectory the book can draw** and **a bundle that witnesses a dynamics model**: `0.5.0`
  writes frame-shaped samples from `solve --json`, and gives every model `source/` plus a digest map
  in the manifest (`otkd-bundle-2`). [`decisions.md`](decisions.md) round 21.
- **Kernel equilibrium**: a compliant model now gets Newton on the equilibrium residual with the
  load case baked in, and agrees with Python's solver to `1e-12` on the pose. The free direction
  is found by one linear solve rather than an SVD, which is why this arrived without a singular
  value decomposition. The SVD *diagnosis* is deliberately still Python-only — §3's figure shows
  its result, and a browser has no use for an interactive one.
- **Kernel least squares** (`solver: "redundant"` in `kernel.json`): Levenberg–Marquardt on the
  normal equations, selected by the model's own classification. It agrees with Python's least
  squares to `1e-15` on the pose and `1e-13` on the metrics.
- **Kernel metrics** (`abi: otkd-kernel-2`): camber, toe, track, contact patch, instant centre
  and roll centre are computed in the kernel and agree with `analysis/outputs.py` to `1e-9` at
  every tested drive value. A page no longer computes any kinematics itself.
- **LaTeX display names**: `otkd compile --format latex` prints `P_{\text{lcaf},x}` and
  `C_{\text{lca-hinge},11}` instead of `g_{lcaf 0}`.
- **An end-to-end CLI test**: `check`, `solve`, `emit`, `kinds`, `compile` and the WASM refusal
  are exercised through the command line, including their exit codes.

## Done in the round that created this list

- **Instant centre**: the pivot is now the foot of the perpendicular from the ball joint onto
  the hinge line, not an arbitrary bushing origin, and a test pins the exact case (hinge axes
  perpendicular to the front view, where the construction is exact) and the two-bush arm.
- **Compliant `iterations[]`**: the equilibrium solve reports accepted iterates (3–4 per frame)
  instead of every residual evaluation (130+), so the export key means the same thing in §4 as
  in §1.
- **Dynamics config validation**: mass, centre of mass and inertia are checked at load, with the
  same fault style as everything else.
- **Wheel travel pinned by a test**: the example sweep is ±42 mm, not the handoff's implied
  ±40 mm from a different arm length, and a test fails if it drifts.
- **`contact` is always exported**: the solved patch where there is a road, the lowest point of
  the wheel otherwise.
