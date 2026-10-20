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

**A compliant spherical joint is already expressible, and does not solve.** No new kind is
needed: a per-joint `modes:` override says `{tx: spring, ty: spring, tz: spring}` on `uca-up`, and
`geometry.yaml` gives the three compliances. The model classifies as `equilibrium` with three free
directions and checks clean — and then the equilibrium does not converge, at any stiffness from
0.2 to 100 MN/m, from any of six starting poses, with a residual between 0.056 and 16.

Five explanations were tested and **four are ruled out**:

| hypothesis | measurement | verdict |
|---|---|---|
| the model is redundant | hard rows rank 18 of 18; with the three spring rows, 21 of 21, smallest singular value 0.086 | ruled out |
| the springs cannot reach the free directions | `J_spring Z` is a well-conditioned 3×3 (singular values 0.74, 0.39, 0.25) | ruled out |
| it is a force-balance problem | the residual is **the same at 3.5 kN and at 3.5 N** (0.056 in both) | ruled out |
| the null-basis sign anchoring flickers | disabling the anchoring entirely leaves it at 0.098 | ruled out |
| the projected potential is not smooth in `q` | consistent with everything above: the residual does not move with the load, and `Z(q)` comes from an SVD whose basis rotates — and whose columns can reorder — as `q` moves, so the finite-difference Jacobian is noise and the trust region stops on `xtol` | **the live hypothesis** |

The route that follows is to stop projecting: solve the **saddle-point KKT system** for `(q, λ)`,

```
[ ∇Π(q) − Φ_q(q)ᵀλ − J(q)ᵀF ]   = 0
[ Φ(q)                        ]
```

which is square, smooth, and needs no null-space basis at all. The package already solves that
shape for the dynamics accelerations (`docs/dynamics.md`), so the machinery and the reasoning exist.

**A stiffness axis is a study change, not an example.** A number iterated *outside* the travel
driver, so one run produces the ladder; each inner sweep seeded from the **design pose**, because a
soft joint and a stiff joint are different branches and `solvers/sweep.py`'s predecessor seeding is
for continuity within a sweep. `Sweep.frames` carries one `primary` value today. The classifier is
unaffected (rank is a property of the model) and the kernel needs no ABI change — it already takes
the compliance as a number slot.

Done means: the equilibrium converges for a ball joint that is compliant in three components; an
example sweeps stiffness and its export reports camber at rest and over the travel for each value;
and a **measurement** shows camber responds to that compliance. If it does not, the answer is no —
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
