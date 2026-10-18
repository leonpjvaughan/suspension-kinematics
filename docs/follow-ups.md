# Accepted follow-ups

Work that has been agreed and deliberately not done yet. Each entry says what it is, why it is
deferred, and what would make it done — so picking one up does not require reconstructing the
argument.

## Compiled kernels, stage 2

The kernel currently compiles the square Newton path. The emitter *refuses* a redundant or
compliant model with a message rather than emitting a solver for the wrong equations, so the
gap is loud. Three increments close it, in this order.

### 1. Least squares for redundant models

**What.** Emit a redundancy-aware solver: `(JᵀJ + λI) δ = −Jᵀr` with a trust-region-style
adaptation of `λ`, using the existing LU. This is what §2 of the book needs in a browser.

**Why deferred.** The chapter's §2/§3 figures step through *iterates* and show an SVD; a
precomputed payload is the better artefact for that, so this is a capability rather than a need.

**Done when.** The two-leg model's kernel matches the Python least-squares solution to `1e-9`
on every frame, and the residual floor is reported the same way.

### 2. SVD and the compliant equilibrium

**What.** A Jacobi or Householder SVD for singular values and the null space, then the
equilibrium root find of §4 in the kernel. This is the interactive stiffness slider.

**Why deferred.** It is the largest of the three and the only one whose Python side is still
changing shape (the solver moved from `lm` to `trf` this round).

**Done when.** The stiffness ladder in the browser matches the Python ladder to `1e-6` on
camber across the compliance range, and a singular Jacobian returns the same status code the
Python solver raises on.

## Dynamics

Accepted, planned in [`dynamics.md`](dynamics.md) §7: v2.0 accelerations and reactions, v2.1
time stepping with projection and the energy test, v2.2 four corners and a load case. Config
for it already parses and validates.

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
| Reconcile the book's geometry with the examples' | `maths-site`'s `mechanism.js` uses different numbers (upper-arm height 0.454, tyre radius 0.32); the chapter should teach one set |
| A sanity check on the toe curve | the steering arm was lengthened until bump steer was believable (±0.6°); it wants an engineer's eye, not a test |

## Done

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
