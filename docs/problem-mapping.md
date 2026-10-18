# From a model to a problem, and from a problem to a solver

**Status:** design, not yet implemented. Companion to
[`config-layer.md`](config-layer.md), which defines what a model *is*. This document
defines what is *built* from one, and how each of the four chapters' problems is selected
and solved. It refines handoff §5 and does not change the export contract in §7.

Read the first document first. The vocabulary here — body, site, component, mode, row,
actuator, driver — is defined there.

---

## 1. The pipeline

```
topology.yaml ─┐
geometry.yaml ─┼─► load + validate ─► resolve modes ─► symbolic rows ─► J = ∂rows/∂q
study.yaml    ─┘                        (§9 rules)       (sympy)         (sympy)
                                                                          │
                                          ┌───────────────────────────────┘
                                          ▼
                                  lambdify(cse=True) ─► Problem
                                                         │  rows, jacobian, row names,
                                                         │  units, driven columns, ranks
                                                         ▼
                                        newton / least_squares / svd_diagnose
```

There is exactly one `Problem` per (topology, constraints library). The geometry is **not**
baked into it: points, axes, lengths, gaps and compliances are symbols in the residual, so a
compiled function is `rows(q, g)` and `g` is supplied at solve time. Consequences worth
stating, because they are what make the entry points cheap:

- a **design study** compiles once and solves many geometries — no re-derivation, no
  re-lambdify, which is the config split's promise kept literally
  ([`entry-points.md`](entry-points.md) §3);
- a **stiffness ladder** is a change of one number in `g`, not a rebuild;
- the slow step is per topology, the fast step is per frame, so `sweep` is cheap enough to
  sit behind a CLI.

No solver knows a joint kind by name; no joint knows about numpy. The frame only moves the
driver row's right-hand side, which is why the symbolic build is amortised over the sweep.
Compiled functions are cached on a hash of the resolved symbolic rows, so a second model
sharing the same topology and library reuses the build.

---

## 2. Unknowns

One block of **seven** per moving body: three for translation, four for a unit quaternion
(scalar first), plus one normalisation row.

```
q_body = (p_x, p_y, p_z, q_w, q_x, q_y, q_z)      R = R(q),  |q| = 1
```

**Why a quaternion and not a rotation vector.** The handoff's design said a rotation vector
(three parameters, no extra row). It is smaller and it is what a textbook would write, but the
exponential map's derivative is `0/0` at zero rotation — which is exactly the design position
that every model is assembled, validated and first solved at. Making that safe needs custom
series-guarded helper functions with hand-supplied derivatives, which is precisely the kind of
hand-differentiation this package exists to avoid. With a quaternion every row is polynomial,
the Jacobian is exact everywhere, and the cost is one extra unknown and one extra row per
body. The rows stay written in terms of `R`, so this is a change to one file if it is ever
revisited.

Properties this buys:

- **`q = 0` is the design position, exactly**, because every body's design frame is the world
  frame, so the seed is a vector of zeros and ones that is *known* to satisfy the model.
- **No trigonometry anywhere except the driven rotation row**, so the Jacobian is polynomial
  and fast to evaluate.
- **No singularity anywhere.** A quaternion represents every rotation, `R` is polynomial, and
  the normalisation row keeps it on the unit sphere.
- One normalisation row per body, `q·q − 1 = 0`, which is a hard equality like any other.

Bodies marked `fixed` (always including `ground` or `chassis`) contribute no unknowns. **The
unknown count is `7 × (#moving bodies)`** — not "DOF minus driven", which is the mobility, a
derived and checked quantity.

A body with no constraint at all is legal to *parse* and is rejected by the rank check with
the free direction named, because a floating body is a modelling mistake nine times in ten and
an under-determined solve the tenth.

---

## 3. Rows

Rows are built by family and component. `C` is the site orientation (constant, world at
design), `o` the site origin (world at design, hence also the body-local coordinate),
`V = Cᵀ v(q_A⁻¹ q_B)` (the vector part of the relative quaternion, in the site frame),
`d = Cᵀ R_Aᵀ (x_B − x_A)`.

### 3.1 Hard rows (`locked`)

| component | expression | unit |
|---|---|---|
| `tx`, `ty`, `tz` | `d_i` | m |
| `rx`, `ry`, `rz` | `V_i` | dimensionless (`sin(θ/2) · axis_i`) |

Rotation rows use the relative quaternion's vector part because it is polynomial in the
unknowns — no `log`, no `atan2`, no branch choice — and it is zero exactly when the two site
frames are aligned. (`vex` of the rotation matrix would do the same job with a `sin θ`
scaling; the quaternion form avoids forming the matrix at all.) Locking `rx, ry` on a revolute is therefore two rows
that are zero if and only if the only rotation left is about the site's z.

### 3.2 Driver rows (`position` actuator)

| actuated component | expression | unit |
|---|---|---|
| `tx`/`ty`/`tz` | `d_i − s` | m |
| a rotation component | `θ_e(Q) − θ` | rad |

`θ_e = 2·atan2(v·e, w)` is the signed angle of the relative rotation about the unit axis `e`
(the site axis of the driven component), from the scalar part `w` and vector part `v` of the
relative quaternion, right-handed, in `(−π, π]`. For a
pure rotation about `e` it is exactly that angle; near the design position it is `≈ V·e`,
so it is smooth through zero. The sign convention is stated once and pinned by a test: **a
positive driven value rotates body `b` positively about the site axis relative to `a`.**
Which is why `a` and `b` are ordered in a joint.

`θ_e` is a coordinate only when the joint has exactly one free rotation component; a joint
free to rotate about two axes has no such scalar, and validation rejects an actuator there
by name (`config-layer.md` §6).

### 3.3 Axial rows

| component | expression | unit |
|---|---|---|
| `length` | `|x_B − x_A| − L` | m |

where `L` comes from `geometry.joints.<joint>.length`, defaulting to the design separation
and erroring on a disagreement (`config-layer.md` §9 rule 4).

### 3.4 Plane rows

| component | expression | unit |
|---|---|---|
| `gap` | `n · (x_B − x_A) − g` | m |

`n` is the `from` site's z axis in world (the plane normal), `x_A` a point on the plane, and
`g` the gap from `geometry.joints.<joint>.gap`. This is a wheel: the normal is the road's z,
the point is any point on the road, `x_B` is the wheel centre, and `g` is the radius. One
row, so a wheel on a road costs exactly one equation — and the row is signed, which is why
`g` can be a radius rather than a distance.

A `spring` gap is a compliant tyre: the same row moves into the equilibrium set of §5, with
`c` in m/N. Rolling without slipping is **not** expressible here — it is a relation between
velocities, not positions — so the contact point slides freely in a kinematic model and
belongs to the dynamics work (`dynamics.md` §6).

### 3.5 Compliance rows

> **A `spring` component does not contribute a row in the sense above.** It changes the
> family of problem being solved, and pretending otherwise produces a model in which
> compliance has no effect. This is the one place where the handoff's sketch is not
> implementable as written, so it is set out separately in §5.

### 3.6 Names, order, units

- Row name is `<joint>.<component>`, e.g. `lca-ground.rz`, `tie-rod.length`. It is the
  handle used by `study.drop`, by `validate` output, and by the export's
  `iterations[].residual` (a positional array — the order must be documented, so it is:
  joints in file order, components in `tx ty tz rx ry rz length` order).
- Rows carry their unit. `validate` prints it, because "residual 1e-3" means a millimetre
  for one row and 0.06° for another, and a reader has to be told which.
- Translation rows are metres, `V` rows are dimensionless, `length` rows are metres; the
  least-squares residual mixes both. Nothing in the solver compensates; §6 says why the
  tests compare solutions rather than costs.

---

## 4. Assembling the numeric problem

```python
@dataclass(frozen=True)
class Problem:
    names: tuple[str, ...]            # row names, stable order
    units: tuple[str, ...]
    rows: Callable[[q, g], np.ndarray]   # q = pose unknowns, g = geometry numbers
    jac: Callable[[q, g], np.ndarray]
    droppable: frozenset[str]         # rows whose removal is meaningful
    formula: str                      # the residual vector, for docs and debugging
    solve: Callable[..., Sweep]       # the public per-geometry entry point
```

`Problem` is keyed by (topology, library) and is geometry-free; `solve(geometry, study)` is
where the numbers enter. The numeric functions below omit the `g` argument in their
signatures for readability — every one of them takes it.

Three derived facts decide everything downstream. The first is structural; the other two are
computed at the design position for the geometry in hand, and re-checked at each solution:

- `n = 7 × (#moving bodies)` — the unknown count, a property of the topology alone.
- `rank(J)`, `redundancy = m − rank(J)`, `null_space_right(J)` — evaluated at `q = 0` with
  the supplied `g`.
- the design-position residual of every hard and compliant row — evaluated the same way.

### 4.1 Choosing the solve, from the numbers rather than a flag

| condition | problem | solver |
|---|---|---|
| `m == n` and `rank == n`, no springs | square, determined | `newton` |
| `m > n` and `rank == n`, no springs | redundant, consistent | `least_squares`, plus `svd_diagnose` |
| `rank < n` | under-determined | refuse; name the right-null-space directions |
| any `spring` component | stationary potential (KKT) | `newton` on the KKT residual (§5) |

The decision is made from the Jacobian, not from a `solver:` key in the study. A model
that says "solve me by least squares" and is not redundant should not be trusted; a model
that is redundant and says "Newton" cannot be. The study can only *remove* rows
(`drop:`), and validation checks that a dropped row was redundant — dropping an
independent row would silently remove a real constraint, which is the single most
expensive kind of quiet mistake available here.

The handoff's `mobility == number of driven coordinates` becomes `n − rank(J_hard) ==
#position actuators`, which is the same statement for §1 and the correct one for §2 (see
`config-layer.md` §9 rule 6 for why the Grübler count cannot express it).

---

## 5. Compliance is a different problem family

A `spring` component means the joint's coordinate is free but resisted. The residual for
that coordinate is not `g_i(q) = 0`; it is the statement that the *generalised effort*
along `g_i` equals the deflection divided by the compliance:

```
g_i(q) = c_i · f_i        f_i = generalised effort on component i from applied loads
```

`f_i` is an internal reaction, not an input: it depends on every other joint and on the
applied loads, and it is exactly what the rigid model computes in its Lagrange
multipliers. Writing it as a row would require knowing the load path before solving for
it. Two formulations avoid that, and only one of them is worth building:

**Rejected — reaction-substitution.** Solve rigidly, read the multiplier, set
`δ = c λ`, re-solve. Cheap, one iteration, and wrong whenever the deflection changes the
load path — which is the whole reason §4 exists. It would pass a monotonicity test and
teach the wrong lesson.

**Chosen — stationarity of the total potential.** The compliant model is

```
minimise   Π(q) = ½ Σ_i g_i(q)² / c_i  −  Σ_loads F·x(q)
subject to g_hard(q) = 0  and the driver rows
```

and its solution is the KKT point: the hard constraints hold, and the potential is
stationary in every remaining free direction. Newton solves the KKT residual

```
r(q) = [ g_hard(q) ; g_driver(q) ; Z(q)ᵀ ∇Π(q) ]
```

where `Z(q)` is an orthonormal basis of the null space of the hard-plus-driver Jacobian,
recomputed at each iterate. The row count is `rank(hard) + driven + dim Z = n`, so the
system stays square and `newton` is reused unchanged: **§4 is Newton on a KKT residual,
not a second solver.**

Three properties fall out, and they are the chapter:

- **`c = 0` reproduces the rigid model exactly.** The equilibrium rows scaled by `c_min`
  tend to the rigid rows as `c → 0`, and at `c = 0` the code takes the rigid path.
  Compliance is a *deformation* of the constraint, not an approximation of it.
- **The ladder is monotone and well conditioned** because the stiff limit is `c → 0` in
  the denominator, not `k → ∞` in the numerator (`config-layer.md` §3).
- **It is the same object as the dynamics.** `Π` plus kinetic energy is the Lagrangian;
  a bush is the same term in both. Building §4 this way means the dynamics work starts
  from a formulation that already exists, rather than from a kinematic special case that
  has to be discarded.

**This makes a load mandatory for §4, and that is a finding about the handoff, not an
implementation detail.** With no applied load the only term in `∇Π` is the spring itself,
`−δ/c`, so stationarity gives `δ = 0` at every stiffness: the compliant model is
indistinguishable from the rigid one, the stiffness ladder is flat, and acceptance test 4
passes vacuously. §4 only
means something if the wheel is loaded — a vertical wheel load and a lateral force at the
contact patch are the honest minimum — and it is then the *deflection under load* that
shrinks as the bushes stiffen. `decisions.md` §3 records the decision.

Row scaling is part of this. `∇Π` carries a factor `1/c_i` per contributing spring, so each
equilibrium row is multiplied by the **smallest compliance** among the springs that
contribute to it — that is, divided by the largest `1/c`. The Jacobian then stays `O(1)`
across a ladder spanning six decades, and "monotone convergence" is a statement about the
physics rather than about floating point.

---

## 6. The three solvers

### `newton(problem, q0, tol=1e-12, max_steps=60)`

Square system, damped Newton with a backtracking line search on `‖r‖∞`; solves
`J Δq = −r`, `q ← q + α Δq`. Returns `(q, iterations, residuals)` where `residuals` is the
full residual vector at each accepted iterate — that is what the export's `iterations[]`
needs, so the history is a return value, not an afterthought. Non-convergence raises
`SolverError` naming the frame (`theta`, index), the last residual and the worst row by
name. A step limit is a bug report, not a result.

The damping matters for exactly one reason: most frames start one step from the answer and
never use it, but the first frame of a sweep starts at the design position — where the
mechanism may be badly conditioned in the direction it is about to move — and any frame
after a dropped row or a change of branch may start on the wrong side of a fold.

### `least_squares(problem, q0)`

`scipy.optimize.least_squares(..., method="lm")` on the redundant residual. `lm` requires
`m ≥ n`, which is what redundancy means, so the choice is forced. Returns `q`, `cost`,
`optimality`, `iterations`, and the singular values of `J` at the solution. The iterate
history comes from the solver's `callback`, which receives each accepted `xk` — it has to
be captured there, because `least_squares` reports counts rather than the sequence the
export's `iterations[]` needs.

**`cost` is not a measure of correctness here.** The extra row of a redundant-but-
consistent model is satisfied to roundoff, so `cost ≈ 0` whether the solution is right or
not; the *solution* is what the acceptance tests compare. `cost` is exported because the
book's §2 figure shows the residual history, and a reader is entitled to see that the
floor is machine epsilon rather than a physical number. If the book wants a physical
floor, the model has to be *inconsistent* by a small amount, which is a different model and
a different claim (`decisions.md` §7).

### `svd_diagnose(problem, q)`

Computed at a solution, not at the design position. Returns:

- `singular_values`: all `m` of them, descending.
- `null_space`: a basis **over constraint rows** — the left null space, dimension
  `redundancy`. Each vector says which *rows* combine to zero, which is the thing a reader
  can check by hand.
- `dependent_rows`: row **names** whose coefficient in some left-null vector exceeds
  `1e-9` (a printed, relative threshold; the count is asserted, not the threshold).
- `rank`, `redundancy`, `condition_number`.

The book's §3 shows exactly this, so it is returned in the form a figure draws: names, not
indices into an array the figure would then have to re-index.
`svd_diagnose` never computes a solution itself — it is given one, so a figure cannot
disagree with the solver it is describing.

---

## 7. Branch continuity

Every frame is seeded from the previous frame's solution. A sweep records, per frame,
`branchChanged`, and never raises: a branch flip is information about the model and
silently hiding it destroys a camber curve in a way that looks like a physics bug.

Two independent detectors, because each alone can be fooled:

1. **Seed displacement.** A solution more than `Δ_max` (documented, per-frame, relative to
   the driven increment) from the seed means the solver found a different assembly.
2. **Closed-form label.** `analysis/analytic.py`'s circle-circle / circle-sphere solve returns both
   intersections; the frame's solution is labelled by which one it is near. A label that
   changes between frames is a branch change even if the step was small.

When the two disagree, the frame is flagged and `validate` reports it. For the
`double-wishbone` examples the two branches differ by the sign of the upright's lateral
offset; the design position sits unambiguously on one.

---

## 8. The problems, concretely

Every moving body contributes one normalisation row, so the counts below are constraint rows,
driver rows and norm rows. `n = 7 × moving bodies`.

### §1 `double-wishbone` — square Newton

| row group | rows |
|---|---|
| `lca-ground` revolute (`tx ty tz rx ry`, `rz` free) | 5 |
| `uca-ground` revolute | 5 |
| `lca-up` spherical | 3 |
| `uca-up` spherical | 3 |
| `tie-rod` distance | 1 |
| driver `lca-ground.rz` | 1 |
| unit quaternions (`lca`, `uca`, `up`) | 3 |
| **total** | **21** |

`n = 21`; `rank` of the constraint rows `20`; mobility `21 − 20 = 1`; prescribed `1`;
redundancy `0`. Newton. The independent cross-check is `analysis/analytic.py`: the upright is
the intersection of a circle (lower arm, about its hinge) and a sphere (upper arm, about
its), both in closed form, and the tie rod fixes the upright's spin about the kingpin. Test 1
compares camber *and* the solved positions over the whole travel range.

### §2 `double-wishbone-two-leg` — redundant least squares

`uca-ground`'s revolute is replaced by two sphericals, `uca-front` at `ucaf` and `uca-rear`
at `ucar`. The driven joint is §1's, unchanged, so the two camber curves are directly
comparable:

| row group | rows |
|---|---|
| `lca-ground` revolute | 5 |
| `uca-front` spherical | 3 |
| `uca-rear` spherical | 3 |
| `lca-up` spherical | 3 |
| `uca-up` spherical | 3 |
| `tie-rod` distance | 1 |
| driver | 1 |
| unit quaternions | 3 |
| **total** | **22** |

`n = 21`; `rank` of the constraint rows `20`; `rank` of everything `21`; redundancy
`22 − 21 = 1`; mobility `21 − 20 = 1`. Least squares.

The left null space is one-dimensional and its support names the two legs and the upper body's
norm row, because that is what combines to zero. The right null space shows the other face of
the same fact: fixing two points of a rigid body leaves rotation about the line joining them
free. Six rows, five independent — available as a number and a set of row names rather than an
assertion.

### §3 — the same, diagnosed

Same model, diagnosis at the mid-travel solution. There are 22 constraint rows and 21
unknowns, so the zero singular value does not appear in the singular-value list at all
(`svd` of a tall matrix returns `min(m, n)` values): the rank is 21, all 21 singular values
are healthy, and the redundancy is visible only as the one-dimensional **left** null space.
That distinction is worth a figure, and it is why `svd_diagnose` returns row *names*: `uca.unit`
and the four `uca` spherical rows are what the dependence is made of.

A second study with `drop: [uca-front.ty]` is square and well conditioned, and acceptance
test 2 asserts its Newton solution equals the least-squares solution to `1e-9`. Dropping a row
*outside* the dependent set leaves the system singular — the test asserts that too, because it
is the difference between removing a redundancy and removing a constraint.

### §4 `double-wishbone-compliant` — equilibrium of a loaded corner

§1's topology with `tie-rod: {modes: {length: spring}}`, a compliance in geometry, and a
load case in the study: a vertical and a lateral force at the contact patch. Constraint rows
`16 + 3`, driver `1`, so the null space of the constrained Jacobian has dimension `1` — the
tie rod's axial extension — and one equilibrium row closes the system: `21` rows, `21`
unknowns.

The equilibrium row is `Zᵀ∇Π / f_ref` with `Π = ½δ²/c − F·x` and `Z` the free direction. Its
derivative is *not* supplied analytically: `∂/∂q (Zᵀ∇Π)` contains `(∂Z/∂q)ᵀ∇Π`, which vanishes
at the solution but not on the way there. An attempt to supply the truncated block made the
trust region stall outright (197 evaluations, no convergence); differentiating the residual
numerically converges in three or four iterations per frame. The constraint Jacobian inside
the residual is still symbolic and exact. The solver is a trust-region least-squares method
with a callback, so `iterations[]` is a list of *accepted iterates*, as it is for §1 and §2.

If the book later wants the *bushes* at the inner pivots rather than the tie rod, it is
`kind: bushed` on those joints and no new code: six spring components where five hard rows
were, six equilibrium rows and no change to the derivation. The actuator then may not sit
on one of those components (validation rule 7) — a bushed wishbone is driven by the
chassis, or loaded, not by prescribing the bush it is meant to be measuring. That is the
config-layer claim, and this is where it gets tested.

### The road-driven model — same mechanism, different input

`examples/ride-height/` asks the same corner a different question: hold the car still and
move the road. Nothing about the linkage changes. What changes is which body is inertial,
what is attached to it, and which actuator the study drives:

```yaml
bodies:
  chassis: {fixed: true}          # the inertial frame, instead of a body called `ground`
  lca: {}
  uca: {}
  up: {}
  road: {}                        # the floor, as a body

roles: {reference: chassis, road: road}

joints:
  # ... the same five linkage joints, with `chassis` in place of `ground` ...
  - {name: road-chassis, kind: prismatic, a: chassis, b: road, site: road-slide}
  - {name: contact, kind: plane-gap, a: road, b: up, from: patch, to: wc}

actuators:
  - {name: road-height, joint: road-chassis, dof: tz, effort: position}
```

`road-slide` is a site at the road's origin whose z is world up; `patch` is a site on the
road at the design contact point with the same z; `geometry.joints.contact.gap` is the tyre
radius. The geometry is the §7 file plus two points (`road-origin`, `patch` — directly below
`wc` at design), one axis `up: [0, 0, 1]`, and that one `gap`. Everything else is unchanged,
which is the point. The study sweeps `road-height` over ±42 mm, and the driven
coordinate *is* ride height, which is why `meta.travel.unit` is `"m"` for this example.

| row group | rows |
|---|---|
| `lca-chassis` revolute | 5 |
| `uca-chassis` revolute | 5 |
| `lca-up`, `uca-up` sphericals | 6 |
| `tie-rod` distance | 1 |
| `road-chassis` prismatic (`tx ty rx ry rz`, `tz` free) | 5 |
| `contact` plane gap | 1 |
| driver `road-chassis.tz` | 1 |
| unit quaternions (`lca`, `uca`, `up`, `road`) | 4 |
| **total** | **28** |

`n = 28` (four moving bodies); `rank` of the constraints `27`; mobility `28 − 27 = 1`; redundancy `0`.
Newton. The road is fully located by its prismatic joint plus the driver, and the wheel is
located by the mechanism plus one contact row — which is exactly the statement that a
kinematic suspension has one degree of freedom once the tyre touches the ground.

**The two models are the same mechanism, and a test should say so.** For each road-driven
frame, read the lower arm's angle about its hinge out of the solved pose, solve the
arm-driven model at that angle, and compare every body pose to `1e-9`. Once you have done
that, "which end do you drive?" is visibly a property of the question rather than of the
model — and it is the clearest demonstration of what the actuator-as-separate-record
decision buys.

Three things this example makes possible that the arm-driven one cannot:

- **Ride height as the input**, and therefore camber, toe and track change plotted against
  it rather than against an arm angle a reader cannot picture.
- **The contact patch as a solved point**, measured where the tyre actually touches, so
  track change is a real lateral movement of the patch rather than a projection.
- **Roll and pitch of the road**: the road joint's mode map is configurable, so `free: tz,
  rx, ry` on a full-car model gives heave, roll and pitch with no new code. The mobility
  check then demands three drivers — or a load case — which is exactly the right
  conversation to have with a model that has four wheels and one road.

---

## 9. What the export reads, and from where

| export key | source |
|---|---|
| `meta.model` | `topology.id` |
| `meta.generator` | `"otkd"`, with no version: a version here would change every committed payload on every commit. The version lives on the release — `MANIFEST.json`, and each kernel's `provenance.json` |
| `meta.branch` | the assembly label from §7 for the design position |
| `meta.frames` | `len(frames)` |
| `meta.travel` | the single driver's law, `from`/`to`, and its **unit** — `"rad"` for a rotation, `"m"` for a translation |
| `sweep.input.values` | the driver law's grid |
| `sweep.frames[].theta` | the driver's applied value at that frame, in `meta.travel.unit` (the key keeps its frozen name; the unit field is what says whether it is an angle or a length) |
| `sweep.frames[].lbj/ubj/wc/tro` | world positions of the sites/points, `[x, z]` |
| `sweep.frames[].points` | all `geometry.points`, transformed by their body's pose |
| `sweep.frames[].camber/toe/ic/rc/track` | `analysis/outputs.py`, from the pose (`HANDOFF.md` §6) |
| `sweep.frames[].contact` | `[x, z]` of the contact patch: the solved point when the topology names a `road`, the lowest point of the wheel otherwise |
| `sweep.frames[].branchChanged` | §7 |
| `iterations[]` | `newton`/`least_squares` history at the frame nearest `theta = 0` |
| `agreement.analytic` / `numeric` / `max_abs_diff` | §8, computed at export and asserted in CI |

Nothing in the export recomputes a number a solver produced, and nothing in a figure has
to interpret a solver result. `ic`/`rc` are `null` rather than a large number when
undefined, which is the one place where `null` is a *result* and not a missing value.

---

## 10. Deliberately not built

- **No hand-written Jacobian.** The Jacobian is `sympy.Matrix(rows).jacobian(q)`; the
  KKT residual's `Zᵀ∇Π` is `Zᵀ · ∇Π` with `∇Π` symbolic and `Z` numeric per iterate.
- **No automatic differentiation library.** One derivative, at compile time, exact, and
  printable. A second mechanism for the same thing is a second truth.
- **No solver selection by model name, study name or example directory.**
- **No caching across processes.** The compile is fast enough; a cache is a second source
  of truth with a staleness bug in it.
