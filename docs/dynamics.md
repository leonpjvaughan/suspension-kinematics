# Dynamics: what the config needs, and what it reuses

**Status:** design only. Nothing in this document is implemented in v1, and nothing in it
may be implemented before the four acceptance tests of `HANDOFF.md` §8 pass. It exists so
that the kinematics config is built with the right seams, not so that the solver grows a
time integrator early.

The question this answers: *if the same package had to produce motion under load, which
parts of the config layer would change?* The answer is the useful one — the topology does
not change at all, the geometry gains one block, and the study changes character.

---

## 1. What kinematics has, that dynamics needs

Kinematics asks where the mechanism *is*. Dynamics asks where it *goes*, which needs five
things the kinematic config deliberately omits:

| need | why | configuration |
|---|---|---|
| mass and centre of mass | inertia of each body | `geometry.dynamics.bodies.<name>.mass`, `.com` |
| inertia tensor | resistance to rotation | `.inertia` (six independent components, about the COM, in the design frame) |
| gravity | the default load | `geometry.dynamics.gravity` |
| elasticity and damping | the only forces inside the mechanism | already there: `compliance` per component; adds `damping` per component |
| applied loads over time | the reason it moves | `study.loads` |

Two more that are about the *study* rather than the model: an initial state
(`q₀`, `q̇₀`) and a time grid. Both belong to the study, and neither is a property of the
mechanism. An initial pose may be written as Euler XYZ degrees and is converted to the
internal rotation vector once at load (`config-layer.md` §12); a body's design pose never
has to be written at all.

---

## 2. What is reused, unchanged

Everything structural. A dynamics model is the *same file* as a kinematics model:

- **Bodies, sites, joints, constraint kinds, modes.** A revolute is a revolute. The hard
  rows are the same rows, and they become the constraint equations `Φ(q) = 0` of the DAE.
- **The `spring` mode.** A compliance is a potential in kinematics (a stationarity
  condition) and a potential in dynamics (a force). Same number, same units, same limit
  `c → 0`.
- **Actuators.** Same record, same target (one free component of one joint). `effort:
  position` prescribes the coordinate (a rheonomic constraint, with a reaction the solver
  reports); `effort: force`/`torque` applies an effort. In kinematics only the first is
  meaningful; in dynamics both are, and the effort is simply time-varying.
- **Sites and their frames.** `Z`, the free-direction basis of
  [`problem-mapping.md`](problem-mapping.md) §5, is the same object; in dynamics it is the
  basis in which the free equations of motion are written.
- **The symbolic Jacobian.** `Φ_q` is already derived, exactly, by sympy.

That reuse is the return on the config split: a chapter that shows a mechanism at rest and
a chapter that shows it moving are two studies of one topology.

---

## 3. What is extended

### 3.1 Geometry

One optional block, ignored by the kinematic solver and read by the dynamic one. It is
*not* a second file: a body's mass is a property of the body, and splitting it from its
geometry would create a model that can be loaded with half its numbers missing.

```yaml
dynamics:
  gravity: [0, 0, -9.81]
  bodies:
    lca: {mass: 3.2,  com: lca-com, inertia: [0.021, 0.140, 0.131, 0.0, 0.002, 0.0]}
    uca: {mass: 1.8,  com: uca-com, inertia: [0.008, 0.043, 0.041, 0.0, 0.0, 0.0]}
    up:  {mass: 12.0, com: up-com,  inertia: [0.310, 0.180, 0.310, 0.0, 0.0, 0.0]}

joints:
  tie-rod:
    compliance: 1.0e-6
    damping: 4.0e-4          # N·s/m for an axial component, N·m·s/rad for a rotational one
```

`com` names a **site** (or is a raw triple), so the centre of mass is a point a reader can
find in the drawing rather than three numbers that only exist here. Inertia is about the
COM, in the body's design frame — which, by the identity-design convention, is the world
frame at design, so an inertia tensor from CAD transfers without a rotation.

Validation for these fields is the same discipline as everywhere else: a body named in
`dynamics` that does not exist is an error; a moving body with no mass is an error *when a
dynamics study is loaded* and not before, because a kinematics model has no business
demanding masses.

### 3.2 Study

```yaml
analysis: dynamics
duration: 1.0                 # s
steps: 1001                   # output grid, inclusive
integrator: {method: radau, rtol: 1.0e-9, atol: 1.0e-11}
initial: {q: design, qdot: zero}      # or explicit per-body overrides
drivers: []                   # optional: position actuators become prescribed motion
loads:
  - {site: wco, force: [0, 0, -3500], law: {kind: constant}}
  - {site: wco, force: [0, 1800, 0],  law: {kind: linear, from: 0, to: 1800, steps: 101}}
```

A *load* is a wrench applied at a site, in world axes; in a dynamics study the law runs
over `duration`, and in a kinematics study over the driven coordinate, which is why the
same `law` vocabulary serves both. Keeping it exogenous is a deliberate boundary: the
moment a force depends on the state (a tyre, a damper with a velocity law, a contact) the
config stops describing a mechanism and starts describing a vehicle, which is the thing
the README says this is not.

---

## 4. The equations, and what is already built

With `q` the body-pose unknowns of [`problem-mapping.md`](problem-mapping.md) §2:

```
M(q) q̈  =  f(q, q̇)  −  Φ_q(q)ᵀ λ
Φ(q) = 0                                    (hard rows + any position actuator)
```

- **`M` is block diagonal, 6×6 per body**, because the coordinates are the bodies' own
  poses rather than generalised joint coordinates. Per body, `M_b = diag(m I₃,
  J_ω(φ)ᵀ I J_ω(φ))` with `J_ω` the right-Jacobian of the rotation vector. That is a small
  symbolic expression, derived once, and it is the reason this formulation is affordable
  in sympy at all. A joint-coordinate formulation would give a dense `M` and a much larger
  expression for no benefit here.
- **`f`** is gravity, the applied loads projected through `∂x/∂q`, the spring forces
  `−g_i ∇g_i / c_i`, the damping `−d_i (ġ_i) ∇g_i`, and any `effort` actuator.
- **`Φ_qᵀ λ`** is the joint reactions. `Φ_q` already exists; `λ` is new, and it is also
  the number a suspension engineer wants (bush and ball-joint loads), which is a good
  reason to expose it rather than eliminate it.

Solution strategy, for whoever builds it: index-1 reduction — solve the acceleration-level
constraint `Φ_q q̈ = γ` together with the equations of motion for `(q̈, λ)`, integrate, and
project `q, q̇` back onto `Φ = 0` at each output step. `scipy.integrate.solve_ivp` has no
DAE solver, so the reduction plus projection is the honest route; a full DAE integrator is
a dependency the handoff's dependency list does not allow.

**What already exists before any of this is written:** `Φ`, `Φ_q`, `∇Π` and the compliance
scaling from §5 of `problem-mapping.md`. The dynamics work adds `M`, `f`, `λ` and a time
loop. It does not add a second derivation of the constraints, and it must not.

---

### The per-step budget, measured

The constraint rows and Jacobian already exist and cost 0.4 ms and 0.8 ms per evaluation on
the wishbone. An index-1 step needs, per step:

| piece | cost | note |
|---|---|---|
| `Φ(q)`, `Φ_q(q)` | 1.2 ms | already exists |
| `γ = −q̇ᵀ ∇²Φ q̇` | ~1.6 ms | two Jacobian evaluations along `q̇`, rather than 21 lambdified Hessians |
| `M(q)`, `f(q, q̇)` | < 0.1 ms | block diagonal; `M_rot = 4GᵀIG` with `G` the quaternion's 3×4 map |
| solve `[M Φ_qᵀ; Φ_q 0]` | < 0.1 ms | ~25×25 dense |
| **total** | **~3 ms per step** | one corner |

Three milliseconds per step is 300 Hz for a single corner, and perhaps 100–200 Hz for four
corners with a chassis body. So Python is fast enough for **traces** — which is what the book
needs, and what the export pipeline already produces — and for a single corner at interactive
rates. It is not fast enough for a four-corner model at 1 kHz in real time, and it does not
need to be: the same split as the kinematics, where the page consumes a payload.

If real-time four-corner dynamics is ever required, the speed path is **code generation**, not
a rewrite: every row is polynomial, so the lambdified expressions can be printed to C, Rust or
WASM and the derivation stays the spec. The conformance test is the emitted bytes, which is the
same rule `ui.md` applies to a Rust engine.

## 5. Staged plan, and what v1 owes

| stage | adds | test that earns it |
|---|---|---|
| v1 | kinematics: square, redundant, SVD, and `spring` components with a load case | the four acceptance tests of `HANDOFF.md` §8 |
| v1.1 | the road-driven example: `plane-gap`, `roles`, ride height as the input | the road-driven and arm-driven models agree to `1e-9` |
| v1.2 | `effort: force` actuators and the equilibrium solve for a force-driven coordinate | the reaction a position actuator reports equals the effort a force actuator needs to hold the same pose |
| v2 | `M`, `f`, `λ`, prescribed motion, no integration | `q̈` and `λ` at `t = 0` match a hand calculation for the design pose |
| v2.1 | time integration | energy conserved to `1e-9` relative over a frictionless swing; a static equilibrium stays static |
| — | tyre, contact, friction, aero | not in scope, ever, per the README |

The load case in v1 is a *kinematics* addition, not a dynamics one: it is what §4 of the
book needs for compliance to be visible at all (`decisions.md` §3). Nothing in this
document is required for it; only the study's `loads` block, which is defined there and
not here.

v1 accepts a `dynamics:` block, checks its referential integrity, and **refuses
`analysis: dynamics` with a message naming the document that defines it.** Accepting and
ignoring it would be worse: a file that parses is a file a reader believes.

---

## 6. Invariants that must survive into dynamics

- **Rigid bodies, linear compliance.** A bush is a linear spring; no hyperelasticity, no
  contact.
- **No friction.** Joint reactions are normal forces; nothing dissipates except the
  damping a bush declares.
- **No tyre.** The wheel is a rigid disc or a point; a contact patch force is an input.
  A `plane-gap` joint can hold or resist the wheel's height above the road, but **rolling
  without slipping is a velocity relation**, not a position constraint, so it belongs here
  and not to the kinematic model (`problem-mapping.md` §3.4).
- **The topology is the same object.** A dynamics study may not need a topology it could
  not have loaded for kinematics. If it does, the config split is wrong and that is a bug
  in this document, not a feature.
- **Determinism.** The export contract (`HANDOFF.md` §7) says byte-identical regeneration.
  Adaptive integration threatens that across scipy versions, so an exported dynamics sweep
  fixes the grid and pins the integrator's tolerances in the study file, and CI regenerates
  it exactly as it regenerates a kinematic one.

---

## 7. Build plan

Written so the work is bounded and testable, in the order that keeps each stage verifiable.
Nothing here is implemented yet; the config (`geometry.dynamics`) already parses and validates.

### v2.0 — accelerations and reactions, no integration — **implemented**

| task | detail |
|---|---|
| `Problem.mass(q, g)` | block-diagonal per body: `diag(m I₃, 4 Gᵀ I G)` with `G(q)` the 3×4 map `ω = 2G(q)q̇` for a unit quaternion. Small, polynomial. |
| `Problem.forces(q, q̇, g, t)` | gravity, loads (through the existing point Jacobians), springs `−g_i/c_i ∇g_i`, damping `−d_i ġ_i ∇g_i`, actuator efforts. |
| `Problem.gamma(q, q̇, g, t)` | `−q̇ᵀ∇²Φ q̇`. Compute it as two Jacobian evaluations along `q̇`, not by lambdifying 21 Hessians: the same trick the equilibrium solve already relies on. |
| KKT solve | `[[M, Φ_qᵀ], [Φ_q, 0]] [q̈; λ] = [f; γ]` solved dense. `λ` is the per-row reaction — the number a suspension engineer wants. |
| outputs | `q̈` and `λ` per row, via `otkd.problem.dynamics.accelerations(...)`. |

As built, in `src/otkd/problem/dynamics.py`:

```python
from otkd.problem import dynamics

state = dynamics.accelerations(problem, q, qdot,
                               loads=[("up", "wc", np.array([0, 0, -3500.0]))],
                               gravity=np.array([0, 0, -9.81]))
state.qddot            # (n,)
state.reactions        # {"lca-ground.tx": ..., "tie-rod.length": ..., ...}
state.force            # the generalised force before the constraints react
```

`M` is assembled per body from `Jv = [I, −[r]× W]` and `Jω = [0, W]`, where `ω = W(q)q̇` and `r`
is the centre of mass: the translation and rotation of one body couple through the offset, so
the block is 7×7 and dense. Springs and dampers are forces, not constraints, so `Φ` here is the
hard rows, the normalisation and the drivers — the same set the kinematic solver uses minus the
springs.

Acceptance, and what the tests in `tests/test_dynamics.py` actually assert:

1. **The equilibrium has no acceleration.** A compliant corner solved for equilibrium under a
   load, with gravity switched off, gives `‖q̈‖∞ < 1e-6` and finite reactions in every row.
2. **The acceleration satisfies the velocity constraint to third order.** With
   `v = dq/dθ` from finite differences of solved poses, `Φ(q + h v + ½h²q̈; θ + h)` is smaller
   than `Φ(q + h v; θ + h)` by about `h` — the independent check that `γ` is the true second
   derivative. The drive must move with the state, or the driver row leaves an `O(h)` term.
3. **The mass matrix is symmetric, positive semi-definite, and positive on the motion**; a
   translation-only velocity gives `½ m v²` summed over the bodies.
4. Energy conservation is v2.1's test, once there is a trajectory to conserve it over.

### v2.1 — time stepping — **implemented**

RK4 with projection: after each step, project `q` onto `Φ = 0` and `|q| = 1`, and `q̇` onto
`Φ_q q̇ = 0`. `scipy` has no DAE solver, so this is 60 lines, not a dependency.

As built: `dynamics.integrate(problem, q, qdot, duration=..., steps=..., loads=...)` returns a
`Trajectory` of times, states, velocities, energies and the constraint residual at each step;
`dynamics.energy(...)` includes the applied loads' potential (`−F·x`), without which a loaded
mechanism's "energy" is not conserved at all. The tests take a frictionless swing from a
kinematic equilibrium with a kick along the free direction and require the energy to hold to
`1e-6` relative and the constraint to `1e-9`.

A **dynamics study** is now a file shape:

```yaml
analysis: dynamics
duration: 0.4
steps: 401
loads:
  - {site: wc, force: [1800, 0, -3500], law: {kind: constant}}
```

`otkd solve` integrates it from the design pose with zero velocity and prints the energy and the
constraint residual; `--json` writes the trajectory. `otkd emit` refuses a dynamics study: its
payload is a trajectory, not a sweep, and the book's export contract is about sweeps.

### v2.2 — the corner becomes a car

Damping in the config, a contact-patch load, and a four-corner topology with the chassis body.
The config already supports it; the mobility check will demand three road drivers or a load
case, which is the right conversation to have.

### What the WASM kernel changes

A dynamics step is the same three evaluations (`Φ_q`, `γ`, `M`,`f`) plus a linear solve:
about 3 ms in Python, and 0.1–0.2 ms in a generated kernel. So four-corner real time in a page
is feasible once the kernel grows the mass and force blocks, and the "traces only" assumption in
§1 of this document stops being a limitation.
