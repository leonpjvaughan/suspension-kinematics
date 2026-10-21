# Accepted follow-ups

Work that has been agreed and deliberately not done yet. Each entry says what it is, why it is
deferred, and what would make it done — so picking one up does not require reconstructing the
argument.

## Compiled kernels: a dynamics model

Every kinematic example now compiles, including the compliant and the bushed ones — square Newton,
least squares, and the KKT equilibrium ([`decisions.md`](decisions.md) round 25). What has no
kernel is a **dynamics** study: `emit --format wasm` refuses it by name, and the release manifest
records `none (dynamics)` for the quarter car. A page that wants a trajectory in the browser needs
a kernel that integrates, which is a different program from the three solvers that exist.

Done means: `emit --format wasm` on `examples/quarter-car` writes a kernel whose trajectory agrees
with Python's to `1e-9` over the study, the way the kinematic kernels agree on a pose.

## Dynamics v2.3: four corners, and a load that arrives through the tyre

v2.0 (accelerations and reactions), v2.1 (time stepping, the energy test, the CLI path) and v2.2
(`examples/quarter-car`: a heaving chassis, a coil-over, the tyre on a road plane) are implemented
and tested. Measured for the next step: a four-corner model compiles — 13 moving bodies, 91
unknowns, 167 s to derive, 11 ms per Jacobian — so it is usable for a figure and not for real time.

Remaining, per [`dynamics.md`](dynamics.md) §7: the chassis as a full body with its own compliance,
and a load that arrives through the contact patch rather than at the wheel centre. The quarter car's
`coil` is a spring with no damper, which is what the book's §1.6 quotes.

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

- **The velocity products are per body**: `37.0 ms → 10.3 ms` per acceleration on the quarter car,
  with the same energy drift ([`decisions.md`](decisions.md) round 26).
- **The kernel's equilibrium path** is the KKT system, so a bush at a ball joint compiles and agrees
  with Python to `1e-9` ([`decisions.md`](decisions.md) round 25).
- **A compliant upper joint and a stiffness axis**: `examples/double-wishbone-ubj` sweeps the bush
  at the upper ball joint over 0.2, 1, 10 and 100 MN/m in one run, and camber responds to it
  monotonically (2.0828° of travel at 0.2 MN/m, 2.0280° at 100 MN/m, converging to the rigid
  answer). The equilibrium had to be solved as a KKT system first
  ([`decisions.md`](decisions.md) rounds 23–24).
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
