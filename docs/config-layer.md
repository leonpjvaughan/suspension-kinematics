# The config layer

**Status:** design, not yet implemented. This document supersedes
[`HANDOFF.md`](../HANDOFF.md) §4 (the config split) and refines §5 where the two
disagree. It does not change the export contract in §7 or the acceptance tests in §8.

The handoff asked for `Topology` and `Geometry`. That split is right, and it is not
enough. Three things are being decided at once when a model is written — what the mechanism
*is*, where its parts *are*, and what *question* is being asked of it. Folding the third
into either of the first two makes a design study a file edit. So a model is three files and
a library, and each is a different kind of statement.

This document defines the vocabulary, the constraint-kind library you supply, actuators, the
validation rules that make a model fail at load rather than converge to something, and how
the four sections of the book are expressed in it. The mapping from these problem
definitions to residuals, Jacobians, Newton and least squares is in
[`problem-mapping.md`](problem-mapping.md). The dynamics extension is in
[`dynamics.md`](dynamics.md).

---

## 1. Three files and a library

| file | answers | contains numbers? |
|---|---|---|
| `topology.yaml` | what bodies exist, how they are joined, which constraint kinds, where the actuators are | **no** |
| `geometry.yaml` | every coordinate, direction, length, compliance and mass | **all of them** |
| `study.yaml` | which actuators are driven, with what law, which loads are applied, which rows are dropped, what is held fixed | the study's parameters only |
| `constraints.yaml` | what `spherical`, `revolute`, `distance` … *mean*, as mode maps | no |

The rule that makes this worth three files is the one the handoff already states for two:
**a topology contains no numbers.** It has no lengths, no angles, no sweep range, no
stiffness. A sweep range is a set of numbers about a study, so it lives in the study, not
in the geometry. The geometry is the part that a design study loops over; the topology is
the part that must not change when it does.

```
examples/
  constraints.yaml            # the library, shared by everything below
  double-wishbone/
    topology.yaml
    geometry.yaml
    study.yaml
    out.js                    # committed, generated
```

The same topology with a second `geometry.yaml` is a second design. The same topology and
geometry with a second `study.yaml` is a second question — a different travel range, a
stiffness ladder, a row dropped to prove the model was redundant.

`constraints.yaml` is a **library, not a model file**, and it is **yours**: the package
ships no built-in default. Write one, or copy `examples/constraints.yaml`, and a topology
finds it beside itself or wherever `topology.constraints:` points. A kind is *data* — a
family and its components' modes — so it belongs in a file rather than a Python dictionary,
and one library shared by several models is the normal case. See §4, and
[`entry-points.md`](entry-points.md) for how a model is loaded and run.

---

## 2. Vocabulary

| term | meaning | lives in |
|---|---|---|
| **body** | a rigid body. `ground` (or any body marked `fixed`) is the inertial frame. | topology |
| **site** | a point with an optional orientation, named in the world at the design position. The place where a joint acts. | topology (name, which axes) + geometry (coordinates) |
| **component** | one degree of freedom a family can act on: `tx ty tz rx ry rz` for `frame`, `length` for `axial`, `gap` for `plane`. | vocabulary of the constraint layer |
| **mode** | what a component is constrained to: `locked`, `free`, or `spring`. | topology |
| **constraint kind** | a named bundle of a family's components and their modes: `spherical`, `revolute`, `distance`, `plane-gap`, … | a `constraints.yaml` library, referenced by the topology |
| **joint** | a constraint kind applied to two bodies at one site (or two sites, for `axial` and `plane` kinds). | topology |
| **role** | which body outputs are measured against, and which body is the road. Optional; inferred when there is one obvious answer. | topology |
| **actuator** | a named device acting on one free component of one joint: it prescribes that coordinate (kinematics) or applies an effort (dynamics). | topology |
| **driver** | an actuator plus a law: the pairing that a study sweeps or holds. | study |
| **row** | one scalar equation contributed by one component of one joint. | derived |

**Roles** are the only other structural statement: `roles: {reference: chassis, road: road,
axle: [wc, wco]}` names the body outputs are measured against, the body that is the road, and
the two points that define the wheel's axle. The reference defaults to the fixed body; the
road is needed only when the contact patch must be a solved point; the axle is required,
because camber is undefined without it. All of them are names, so they belong in topology.

**Bodies own points.** `bodies.<name>.points` lists the geometry points a body carries, so an
output point's position is `p + R o` with a known body. Ownership does not affect a
constraint — a site's origin is shared by both bodies and coincides at design — it only tells
the outputs which pose to use. Every geometry point must be owned exactly once, and the check
says which one is missing rather than defaulting it to the world.

A **site** is deliberately one concept rather than "point" plus "axis" plus "frame". The
handoff's `points` and `axes` remain as the geometric primitives — a site names them:

```yaml
sites:
  lca-hinge: {origin: lcaf, z: lca-axis}
  lbj:       {origin: lbj}
```

`origin` names a key in `geometry.points`; `x`, `y`, `z` name keys in `geometry.axes`.
An omitted axis defaults to the world axis of that name, which is only ever meaningful for
a joint whose rotations are all free (`spherical`); validation enforces that (§9 rule 2).

**A joint has one site, not two, because the two bodies coincide there at design.** That
is what a joint is: a place and an orientation that both bodies share before anything
moves. Making it structural removes the commonest way to author a broken model — two
frames that were supposed to be identical and were not — and it makes the design-position
check in §9 almost trivially true. Axial kinds (`distance`) are the exception: a tie rod
joins two points that do *not* coincide, so those joints name `from` and `to`.

**All bodies share the world frame at the design position.** A body's local frame is the
world frame at design, so geometry is authored in world coordinates exactly as a CAD
export gives them, no per-body pose is authored, and the unknown vector is identically
zero at the design position. There is no second coordinate system to get wrong. The cost
is that a body rotated ~π from its design orientation sits at the singularity of the
rotation parametrisation; that is documented, validated, and far outside a suspension's
travel.

---

## 3. The component model

Every joint between two bodies is a statement about the six relative degrees of freedom
between them, expressed in the joint's site frame. Name the site frame `C` (orientation,
constant, world at design), let body poses be `(p_A, R_A)` and `(p_B, R_B)`, and let the
site origin be `o` (world at design, and therefore also each body's local coordinate of
it).

The **relative transform** at the site:

```
x_A = p_A + R_A o                      world position of the site on A
x_B = p_B + R_B o                      world position of the site on B
d   = Cᵀ R_Aᵀ (x_B − x_A)              relative translation, in the site frame   (3)
V   = Cᵀ · vec(q_A⁻¹ q_B)               vector part of the relative quaternion  (3)
```

This block is the `frame` family, where one site is shared. Two other families exist, and
their components are different — the mode vocabulary (`locked`/`free`/`spring`) is the same,
which is what makes a new family cheap:

| family | components | statement | used for |
|---|---|---|---|
| `frame` | `tx ty tz rx ry rz` | the relative transform at one shared site | every joint that locates one body on another |
| `axial` | `length` | the distance between a `from` site on A and a `to` site on B | tie rods, links, a rack |
| `plane` | `gap` | the perpendicular distance from a plane at `from` to a point at `to` | a wheel on a road |

`axial` and `plane` name two sites rather than sharing one, because the two bodies do not
coincide there at design: a tie rod joins two distinct points, and a wheel centre is a
radius away from the road. The [problem mapping](problem-mapping.md) §3.3–§3.4 gives their
rows.

**In the axis-aligned families the site's z is the axis.** The hinge of a revolute, the
slide of a prismatic, the normal of a planar joint and the normal of a plane gap are all
the site's z. A site's x and y complete the frame and nothing about their direction
matters. That single sentence is why `prismatic` frees `tz` and not `tx`, and why a
vertical road slide is a site whose z is up rather than a special kind of joint.

`d = 0` and `V = 0` together mean the two bodies' site frames coincide and are aligned.
`V` is used because it is polynomial in the unknowns (`sin(θ/2) · axis`, no `log`, no
`atan2`), smooth through `θ = 0`, and zero exactly when the frames are aligned. This is
what "local xyz rotations but not local xyz translation" becomes:

```yaml
spherical:  {tx: locked, ty: locked, tz: locked, rx: free, ry: free, rz: free}
```

and a revolute:

```yaml
revolute:   {tx: locked, ty: locked, tz: locked, rx: locked, ry: locked, rz: free}
```

A `spherical` therefore contributes the three rows `d_x, d_y, d_z` and a `revolute`
contributes five: `d_x, d_y, d_z, V_x, V_y`. The free component `rz` is not a row; it is
the coordinate the mechanism is allowed to use, and it is the coordinate an actuator on
that joint prescribes.

Two consequences worth stating because they are easy to get wrong:

- **The free rotation of a revolute is the site's z axis.** The site is the axis. Give it
  `z: lca-axis` and the hinge is that line. To hinge about x, say `x: …` (or give the site
  a different `z`); the modes are about the site frame, not about the world.
- **Which two rotation components are locked is a choice, and `V` makes the choice
  harmless.** Locking `rx, ry` and freeing `rz` is the revolute; locking only `rx` leaves
  two free rotations and is a regular, universal-type joint; locking all three is welded.

### Modes

| mode | contributes | meaning |
|---|---|---|
| `locked` | the row `d_i = 0` or `V_i = 0` | a hard constraint. Supplying a compliance for it is an error. |
| `free` | no row | a coordinate the mechanism may use. An actuator may prescribe it. |
| `spring` | an equilibrium condition, not `g_i = 0` | a linear compliance: the component deflects by `c_i` per unit of applied effort. `c = 0` is the rigid row exactly. |

The `spring` entry is the one that needs care, because a compliant component is *free* but
*resisted*: its coordinate is not fixed, so `g_i = 0` is the wrong equation, and the
equation that closes the system is force balance along that component. Writing that as a
joint row would require knowing the load path before solving for it, so it is not a row in
the same sense as the other two — it changes the family of problem (a stationarity
condition rather than a square row system). The details are in
[`problem-mapping.md`](problem-mapping.md) §5, and the invariant worth stating here is
that **a spring preserves both the row count and the mobility**: freeing one coordinate and
adding one equilibrium equation is a swap, not a change of size. A compliant model is the
same size as the rigid one it replaces, which is what makes §4 the same sweep.

Compliance, not stiffness, is the number in the geometry, and that is a deliberate choice
with a numerical reason. With stiffness the equation is `k d − f = 0` and the limit the
book demonstrates (stiffness → ∞ means rigid) is a limit in which the Jacobian's
conditioning degrades as fast as the physics improves. With compliance the same limit is
`c → 0`: the Jacobian stays bounded, `c = 0` *is* the rigid row rather than a large-number
approximation of it, and §4's ladder is monotone for a physical reason. Stiffness is still
how the number is spoken about and printed; `c = 1/k`. Its units differ per component (m/N
for translation, rad/(N·m) for rotation), and the equilibrium rows that carry it are in
newtons or newton-metres, so a mixed-unit least-squares residual is unavoidable — which is
why the redundant-model tests compare *solutions* and not costs
(`problem-mapping.md` §6).

---

## 4. Constraint kinds are configuration

A kind is a named mode map, and it lives in a **library file** — `constraints.yaml` — that
the user supplies. The package has no built-in library: a model without its library is not a
model. [`examples/constraints.yaml`](../examples/constraints.yaml) is a working library and
the one the book's examples share; it is short enough to read in full, and it is the document
that defines what the book's words mean. Excerpt:

```yaml
constraint_kinds:
  spherical: {family: frame, modes: {tx: locked, ty: locked, tz: locked,
                                     rx: free,   ry: free,   rz: free}}
  revolute:  {family: frame, modes: {tx: locked, ty: locked, tz: locked,
                                     rx: locked, ry: locked, rz: free}}
  bushed:    {family: frame, modes: {tx: spring, ty: spring, tz: spring,
                                     rx: spring, ry: spring, rz: spring}}
  distance:  {family: axial, modes: {length: locked}}
```

The example library also defines `cylindrical`, `planar`, `prismatic`, `universal`, `rigid`
and `plane-gap`. A model resolves its kinds from exactly one library — `constraints.yaml`
beside the topology file, or the path in `topology.constraints:`, always resolved relative
to the topology file — and may add local kinds inline under `constraint_kinds:` in
`topology.yaml`, using the same shape. **Names must be unique**
across the two. There is no shadowing: a per-joint `modes:` override already expresses
"this revolute is about x", and a second definition of `revolute` would make a model
unreadable in exactly the way this design is trying to avoid.

`family` selects the residual shape, not the constraint: `frame` uses `d` and `V` at one
site, `axial` uses the distance between two sites. A kind may be **overridden per joint**,
which is how a rigid tie rod and a compliant one are the same file with one word changed:

```yaml
joints:
  - {name: tie-rod, kind: distance, a: ground, b: up,
     from: tri, to: tro, modes: {length: spring}}
```

Override is not a new kind and does not need a name; it is the same six-to-one component
vocabulary narrowed for this joint. Validation still applies: a `spring` override needs a
compliance in geometry, a `locked` override must not have one.

What is *not* expressible as a mode map is a **coupling** between components — a screw
(`tz` proportional to `rz`) or a geared pair. That is a real extension point, and it is
deliberately not in v1: a coupling row is `d_z − lead·θ_z = 0`, one row for two
coordinates, and it changes the row/unknown arithmetic. The registry is written so a
family can be added without touching joints, derivation or solvers
(`problem-mapping.md` §3).

---

## 5. Joints

```yaml
joints:
  - {name: lca-ground, kind: revolute,  a: ground, b: lca, site: lca-hinge}
  - {name: uca-ground, kind: revolute,  a: ground, b: uca, site: uca-hinge}
  - {name: lca-up,     kind: spherical, a: lca,    b: up,  site: lbj}
  - {name: uca-up,     kind: spherical, a: uca,    b: up,  site: ubj}
  - {name: tie-rod,    kind: distance,  a: ground, b: up,  from: tri, to: tro}
```

`a` and `b` are ordered, and the order matters only for signing the free coordinate and
the spring deflection: the coordinate is measured *from* `a` *to* `b`. Pick it so that the
actuator's sign is the sign a reader expects (positive rotation of `b` in the site frame).

Every joint's name is a namespace prefix for its rows: `lca-ground.rz`,
`tie-rod.length`. Those names are the handles used by the study (`drop:`), by diagnostics
and by the export's `iterations.residual`.

---

## 6. Actuators

An actuator is a device on one **free** component of one joint. It has no numbers; it has
an identity and an effort type.

```yaml
actuators:
  - {name: lca-drive, joint: lca-ground, dof: rz, effort: position}
  - {name: rack,      joint: rack-ground, dof: tx, effort: position}
```

`effort: position` means the coordinate is prescribed by a law. `effort: force` (or
`torque`) means an effort is applied and the coordinate is a genuine unknown, which is a
*kinetostatic equilibrium* problem rather than a pure kinematic one. v1 supports
`position`; `force` is accepted by the schema, rejected by the kinematics solver with a
message that names the problem it would need, and is what §4's "algebraic equilibrium"
becomes once there is an applied load rather than a prescribed motion.

An actuator on a rotational component is only well defined when the joint has **exactly
one** free rotation component: the prescribed row is the angle about that axis (see
`problem-mapping.md` §3), and "the angle about this axis" is not a coordinate of a joint
free to rotate about two. Validation rejects it by name.

The point of the indirection is that the *same* topology can be asked different questions:
one study drives `lca-drive` over ±0.35 rad; another holds it at zero and applies a force
to the tie rod; another drives a rack instead. None of that is a topology change.

---

## 7. Geometry

```yaml
points:                       # world coordinates at the design position, metres
  lcaf: [0.300,  0.120, 0.150]
  lcar: [0.300, -0.120, 0.160]
  lbj:  [0.720,  0.010, 0.150]
  ucaf: [0.380,  0.100, 0.480]
  ucar: [0.380, -0.100, 0.470]
  ubj:  [0.680,  0.000, 0.520]
  wc:   [0.760,  0.000, 0.320]
  wco:  [0.860,  0.000, 0.320]
  tri:  [0.330,  0.060, 0.300]
  tro:  [0.700,  0.040, 0.310]

axes:                         # directions, not positions; normalised by the compiler
  lca-axis: [0.0, -1.0, 0.0417]     # lcaf -> lcar
  uca-axis: [0.0, -1.0, -0.0556]    # ucaf -> ucar

joints:                       # per-joint numbers, keyed by joint name
  tie-rod: {length: 0.370673}       # optional; defaults to |tri - tro| at design
  # lca-ground: {compliance: {rz: 0.0}}   for a `spring` component

scalars:                      # named numbers used by outputs, not constraints
  tyre-radius: 0.200
```

Rules:

- Coordinates are **x lateral, positive outboard for the corner modelled; y forward; z
  up; metres** (handoff open question 3, unchanged).
- Axes are directions and are normalised on load; a zero axis is an error, a non-unit axis
  is normalised with a warning (silently rescaling a hand-typed axis is how a 2% geometry
  error survives review).
- `length` is optional and defaults to the design separation. Giving it explicitly is a
  check, not a requirement; a mismatch is an error that prints both numbers, because it
  means the geometry and the joint disagree about the design position.
- `compliance` is keyed by component (`length`, `rz`, …) and its presence must match the
  topology's modes exactly (§9 rule 4). **`stiffness` is accepted as sugar** — `c = 1/k`,
  exactly one of the two per component, converted once at load. Nothing downstream ever
  sees a stiffness: the argument for compliance in §3 is about conditioning, and it is not
  a reason to make a person divide.
- **No mass, inertia, gravity or damping in the kinematic core.** Those live in an
  optional `dynamics:` block that the kinematic solver ignores and the dynamics work will
  read — see [`dynamics.md`](dynamics.md). One file, two readers, no duplication.

---

## 8. The study

```yaml
analysis: kinematics

drivers:
  - actuator: lca-drive
    law: {kind: linear, from: -0.35, to: 0.35, steps: 81}

loads:                        # only meaningful when some component is `spring`
  - {site: wco, force: [0, 0, -3500], law: {kind: constant}}

drop: []                      # row refs, e.g. [lca-front.tx]
seed: previous                # previous | design
```

- **`law`** is how an actuator is driven: `constant` (hold at `value`), `linear` (from,
  to, steps inclusive; the grid is `from + k (to−from)/(steps−1)`), or `table` (explicit
  `values`). Any actuator in the topology that the study does not drive is held at its
  design value of zero. Documented and pinned by a test: an undriven actuator is *held*,
  not free, so the mobility check in §9 is about the model plus the study, not the model
  alone.
- **`loads`** apply a wrench at a site, in world axes, with the same `law` vocabulary as a
  driver, so a load can be constant or swept across the travel. A load is an input and
  never a function of the state; the moment it is, the config describes a vehicle rather
  than a mechanism (`dynamics.md` §3.2). A load with no `spring` component anywhere in the
  model has no effect and validation says so, because it usually means the compliance was
  forgotten.
- **`analysis: dynamics`** turns the study into a time span: `duration` (seconds) and `steps`
  (output count) are required, `drivers` are ignored, and `solve` integrates. `emit` refuses it.
- **`drop`** removes named rows from the problem. It is how §2's demonstration is
  performed: the model is redundant, the SVD says which rows participate in the
  dependence, and the study names one to drop. The name is explicit rather than `auto` so
  that the diff of a study says what was removed.
- **`seed`** controls branch continuity: `previous` seeds each frame from the last
  (the default, and the only setting that can follow a branch), `design` restarts from
  zero (useful for detecting a second assembly).
- **Multiple drivers** are allowed. One is the mobility check's expectation; two make the
  problem over-prescribed unless the mechanism has two degrees of freedom, and validation
  says so by naming the over-prescription rather than letting least squares average two
  incompatible demands.
- A **ladder** (the stiffness sweep of §4) is a loop over study files rather than a new
  file format in v1: the acceptance test can write each compliance and run the same study.
  A `parameters:` dimension is the obvious extension and is deliberately not specified
  until §4's load case is settled (`decisions.md` §3).

The handoff put the sweep inside `Geometry`. It moves here, and the reason is the one in
§1: a sweep is a question, and the mobility check ("one topology, many geometries") is
weakened if every geometry must carry a travel range it may not want.

---

## 9. Validation — fail at load, name the fault

The compiler refuses a model whose numbers cannot describe the topology, and the messages
name the file the number should have been in. The rules, in order:

1. **Names resolve.** Every `a`/`b` is a body; every `site`/`from`/`to` is a site; every
   site `origin` is in `geometry.points`; every `x`/`y`/`z` is in `geometry.axes`; every
   `kind` is defined in the referenced library or inline, with no name defined twice;
   every actuator's `joint` exists and `dof` is a component; every `drop` is an existing
   row. `roles.reference` and `roles.road` name bodies, a road is not the reference, and
   there is at most one of each.
2. **A site used by a joint with any constrained rotation declares that axis.** A joint
   whose rotations are all free (`spherical`) can use a bare point, because nothing is
   being said about orientation; every locked rotation component is a statement about one
   axis, so the site must name it (`z` for a revolute, `x` for a joint that welds rotation
   about x).
3. **`a` and `b` differ** and neither is the same body twice. A joint from a body to
   itself is not a constraint, it is a typo.
4. **Modes and numbers agree.** Every `spring` component has a compliance (or the
   stiffness it is converted from); every `locked` component has neither; an `axial`
   joint's `length` matches (or supplies) the design separation to `1e-9`.
5. **The design position satisfies the model.** At `q = 0`, every `locked` and `spring`
   row is zero to `1e-9`, and every row is reported if it is not. This is the check that
   turns "the solver found something" into "the solver found the thing you described", and
   it is what makes a solver failure mean a bug rather than a plausible wrong pose.
6. **The mobility is the one the actuators prescribe.** Compute the Jacobian at the design
   position; let `r = rank(J_hard)` and `n = #unknowns`, `m = #hard rows`. Every position
   actuator contributes one prescribed row — a `driver` only says how its value varies —
   so require `n − r == #position actuators`, and report `m − r` as the **redundancy**. A
   redundant model is not an error — §2 is one, on purpose — but it is printed. An
   under-determined model *is* an error, and the message names the free directions from
   the **right** null space of `J` ("`up` can translate in z; nothing constrains it"),
   which is the diagnosis a human wants. Over-prescribed (`n − r < #position actuators`)
   names the prescribed rows that are incompatible.
7. **An actuator's target is free and not compliant**, and a rotational actuator's joint
   has exactly one free rotation component (§6). A `spring` component is resisted by
   definition, so prescribing it is a contradiction that the schema can catch: the bush is
   either driven or compliant, not both.

Rule 6 replaces the handoff's Grübler/Kutzbach count, and the replacement is not
cosmetic. Grübler counts rows; §2 has 18 hard rows, 18 unknowns, and a rank of 17. The
count says mobility 0 and would reject the model that the chapter exists to explain. The
rank says mobility 1 and redundancy 1, which is the truth, and it is available because the
Jacobian is symbolic anyway. Counting rows stays as a fast pre-check that can only ever
*raise* suspicion.

`otkd check <model> [--study study.yaml]` prints, per row: joint, component, mode, residual
at design, unit; then the singular values, `rank`, `redundancy`, `mobility`, `driven`, and
the verdict. It is the command a human debugging YAML wants (handoff open question 7), and
`emit` runs the same checks before it writes anything.

---

## 10. The four sections, and a fifth model

One topology *shape*, four examples that differ on purpose. §2 is a topology change
because it is a different mechanism (two legs, not one hinge); §3 is the same topology
with a diagnosis; §4 is the same topology with one word changed and one number swept.

| | bodies (moving) | joints | modes that matter | rows | unknowns | rank | solver |
|---|---|---|---|---|---|---|---|
| §1 `double-wishbone` | lca, uca, up (3) | revolute ×2, spherical ×2, distance | all `locked` | 17 + 1 + 3 norm = 21 | 21 | 21 (red. 0) | Newton |
| §2 `double-wishbone-two-leg` | lca, uca, up (3) | revolute, spherical ×2 (upper arm), spherical ×2, distance | all `locked` | 18 + 1 + 3 norm = 22 | 21 | 21 (red. 1) | least squares |
| §3 | as §2 | as §2 | as §2 | as §2 | as §2 | as §2 | + `svd_diagnose` |
| §4 `double-wishbone-compliant` | as §1 | as §1 | `tie-rod.length: spring` | as §1 | as §1 | as §1 | Newton, swept in `c` |

§2's second leg is the interesting line, and it is three lines of YAML:

```yaml
joints:
  - {name: lca-front, kind: spherical, a: ground, b: lca, site: lcaf}
  - {name: lca-rear,  kind: spherical, a: ground, b: lca, site: lcar}
```

Two sphericals pin two points of one body. Six rows, five independent: the body can still
rotate about the line through the two points, so one of the six rows is implied by the
other five and the model carries one redundancy. The freedom and the redundancy are two
faces of the same fact, and the chapter's SVD shows the second one — a one-dimensional
combination of rows that is zero, with the rotation about the jointing line as its partner
in the unknown space. The "one rigid body, one degree of freedom" of §1 is the same
mechanism with the two point constraints replaced by the hinge they idealise.

§4's compliant tie rod is the same topology as §1 with `modes: {length: spring}`, and the
geometry carries `tie-rod: {compliance: 1.0e-6}`. `compliance: 0` reproduces §1 to
floating-point, which is acceptance test 4's limit and also its proof that the two models
are the same model. Its study **must carry a load**: with no load the equilibrium forces
every deflection to zero and the compliant curve is identical to the rigid one at every
stiffness (`decisions.md` §3).

There is a fifth model, designed but deliberately outside v1's four:
**`examples/ride-height/`**, the same mechanism with the chassis inertial and the **road**
driven, so the sweep is ride height rather than arm angle. It needs one `prismatic` joint,
one `plane-gap` tyre, one role and one actuator — no solver, no derivation. It is the test
of whether this layer generalises or merely fits, and the [problem
mapping](problem-mapping.md) §8 works it through, including the test that driving the road
reproduces driving the arm.

---

## 11. Python shape

As built: `src/otkd/config/` holds these, one module per concern. `Model` is the loaded set
of files plus the resolved kinds; `Model.with_geometry` swaps the numbers and keeps the
structure.

```python
Component = Literal["tx", "ty", "tz", "rx", "ry", "rz"]   # plus "length", "gap"
Mode = Literal["locked", "free", "spring"]

class BodySpec(BaseModel):
    fixed: bool = False
    points: list[str] = []          # the geometry points this body carries

class Site(BaseModel):
    origin: str
    x: str | None = None; y: str | None = None; z: str | None = None

class ConstraintKind(BaseModel):
    family: Literal["frame", "axial", "plane"]
    modes: dict[str, Mode]          # validated against the family's components

class Joint(BaseModel):
    name: str
    kind: str                       # a key in Topology.constraint_kinds
    a: str; b: str
    site: str | None = None         # frame kinds
    from_: str | None = None; to: str | None = None   # axial kinds
    modes: dict[str, Mode] = {}     # per-joint override

class Roles(BaseModel):
    reference: str | None = None
    road: str | None = None
    axle: list[str] | None = None    # two points, required for outputs

class Topology(BaseModel):
    id: str
    constraints: str = "constraints.yaml"   # relative to this topology file
    bodies: dict[str, BodySpec]
    roles: Roles = Roles()
    sites: dict[str, Site]
    constraint_kinds: dict[str, ConstraintKind] = {}   # local kinds, same shape as the library
    joints: list[Joint]
    actuators: list[Actuator]

class Geometry(BaseModel):
    points: dict[str, Vec3]
    axes: dict[str, Vec3]
    joints: dict[str, JointNumbers] = {}     # length, compliance
    scalars: dict[str, float] = {}
    dynamics: DynamicsNumbers | None = None

class Study(BaseModel):
    analysis: Literal["kinematics", "dynamics"]   # dynamics is reserved, see dynamics.md
    drivers: list[Driver]
    drop: list[str] = []
    seed: Literal["previous", "design"] = "previous"

class Model(BaseModel):
    topology: Topology; geometry: Geometry; study: Study | None
```

`constraint_kinds` resolves in one step: load the library named by `constraints:`, then
check the model's inline map for duplicate names — a duplicate is an error, not a shadow, for
the reason given in §4. The library is validated with the same loader whether it is yours or
the example's: a kind with a missing component or an unknown family is refused by name.

---

## 12. The boundary between what is written and what is solved

There are two vocabularies and it is worth naming the seam once, because "the config says
stiffness but the solver thinks compliance" is otherwise the sort of thing that produces a
debugging session.

**Internal** — what `Problem`, the derivation and the solvers see: metres, radians,
right-handed, x outboard, z up; body pose as a rotation vector; compliance (`m/N`,
`rad/(N·m)`); every angle in radians; every row in its SI unit.

**Authored** — what a person types: the same units (there is no unit field, and inches are
not a feature), and three conveniences that are converted *once, at load*:

| authored | converted to | where |
|---|---|---|
| `stiffness` | `compliance = 1/k` | geometry |
| a pose given as Euler XYZ degrees (only ever a dynamics initial condition) | a rotation vector | study |
| `drop` by row name | a row index | study |

**Exported and printed** — angles in degrees, lengths in metres, rounded to the precision a
figure draws (`HANDOFF.md` §7). A number is converted at the edge it crosses and nowhere
else: the solver never sees a degree, and a figure never sees a radian. That is one rule,
and it is why the four acceptance tests can compare a closed form, a Newton solve and a
least-squares solve without a conversion factor appearing in the middle of each. The same
rule covers a *dimensional* change of input rather than a unit change: the road-driven
example sweeps a length, and `meta.travel.unit` is what tells the figure whether its x-axis
is radians or metres.

---

## 13. Extension points, in the order they are likely to be wanted

1. **Coupling families** — screw, gear, rack-and-pinion expressed as a row rather than a
   mode map (§4).
2. **Point-on-line / point-on-plane / coplanar** families — one or two rows from a
   different expression; the row/unknown arithmetic is unchanged.
3. **Preload and damping** in the compliance (`δ − c f → δ − c (f − f₀)`; damping is a
   dynamics term). Preload breaks the "design position has zero residual" property in a
   *principled* way and needs its own validation story, so it is not slipped in.
4. **Limits** — a component that is free until it is not. Kinematics has no time, so this
   is a piecewise problem, and it is a real change to the solve loop.
5. **Mass properties and dynamics** — [`dynamics.md`](dynamics.md).

The invariant every extension must preserve: **a model's rows are a function of its
topology, its geometry and its study, in that order, and nothing else.** No global state,
no per-solver special case, no joint kind that the solver has heard of by name.
