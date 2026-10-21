# Handoff from `maths-site`: what the book cannot say yet

> **Answered here.** Request 2 is met in **0.5.0**: `solve --json` writes, per step, the fields a
> sweep frame carries (`camber`, `toe`, `ic`, `rc`, `track`, `contact`, the world points) beside
> `t`, `energy` and `constraint`; and every model in the bundle now carries `source/` and a
> `sources` digest map — dynamics included, which is the half that was a correctness question
> (`bundle: otkd-bundle-2`). Request 1 is accepted as
> [`follow-ups.md`](follow-ups.md)'s "A compliant upper joint, and a stiffness axis", in the
> package's shape: a number iterated outside the travel driver, each inner sweep seeded from the
> design pose, and the ladder gated on a measurement that camber responds.
>
> **The ladder exists.** `examples/double-wishbone-ubj` is the mechanism with the compliance moved
> to the upper ball joint, and its study sweeps 0.2, 1, 10 and 100 MN/m in one run — 164 frames,
> with `sweep.axes` naming the three numbers that moved and every frame carrying its stiffness.
> Camber over the travel: **2.0828°, 2.0397°, 2.0291°, 2.0280°**, converging to the rigid answer.
> One caveat for a page: the compiled kernel refuses that model ("an equilibrium kernel expects one
> free direction; this model has 3"), so an *interactive* ladder needs the kernel work in
> [`follow-ups.md`](follow-ups.md); the export is static for now.
>
> **§1.5's ladder needs one decision before it can be regenerated, and this note first got the
> reason wrong twice.** The example's own ladder, in the export's conventions (camber positive with
> the top of the wheel outboard; travel from the loaded rest pose), over ±40 mm:
> **0.2 MN/m: 6.14 mm of bush, rest −1.91°, change 1.96°; 1: 1.22 mm, −1.18°, 1.92°; 10: 0.12 mm,
> −1.02°, 1.91°; 100: 0.012 mm, −1.00°, 1.91°** — the page's `CAMBER0 = −1.0°` is the datum, so
> those are directly comparable with its `−2.05 …` column.
>
> Those numbers agree with the page's table to about ten per cent, and **that agreement is a
> coincidence of load cases**. On the same load — the page's 3 kN lateral at the contact patch — the
> two models differ by a factor of **2.7**: the export's bush moves 2.51 mm at 0.2 MN/m where the
> page's moves 6.8 mm. The reason is structural and worth teaching: the page's model is a
> front-view idealisation with no fore-aft structure, while the three-dimensional mechanism shares
> the load through its tie rod and its spherical lower joint. Softening the tie rod does *not*
> reproduce the front view — a rigid constraint removes a direction rather than adding a spring.
>
> So a table generated from the export would change the *experiment*, not just the numbers. The
> recommendation for the page: drive its figure from the vendored
> `double-wishbone-ubj/kernel/kernel.js` — the kernel takes the bush's compliance as a run-time
> number slot, which is the pattern the rigid `kernel-camber.js` already uses — so the figure, the
> table and the engine are one model and one load case, and the front view becomes the idealisation
> whose cost (2.7×) the section can measure.
>
> **And one contract change to know about.** A payload for a model with springs now reports a
> single converged `iterations[]` entry instead of a descent. The path of a stiff equilibrium solve
> is not reproducible between two machines — the frames were byte-identical while the first
> iterate's residual differed by a factor of two — and a committed artefact has to be reproducible.
> The entry carries the residual (floored, because below the solver's tolerance it is the
> arithmetic's number) and the singular values of the solution's own Jacobian, which is the part
> worth reading. Kinematic payloads keep their full descent.
>
> **That measurement has
> now been made, and it is a yes**: the equilibrium was the wall — a joint compliant in three
> components would not converge in the projected formulation — and solving the saddle point
> directly fixed it. Camber responds to the ball joint's compliance monotonically (2.0828° of travel
> at 0.2 MN/m down to 2.0280° at 100 MN/m), so the ladder is real. What remains before your §1.5
> can be generated is the export shape: an example with that joint, and a stiffness axis so that one
> run yields the ladder. [`follow-ups.md`](follow-ups.md) has the numbers. Request 3 is dropped for
> want of an owner in the book's queue; it can return with an item. The acceptance is the package's
> numbers, not the page's rounding — where they disagree, the page changes.

**From:** `open-textbook.org` (`maths-site`), which vendors your bundle
**At:** `otkd` 0.4.0 (commit `f5d2c92`), 2026-10-19
**Status:** requests, with the measurements that justify each. Nothing here is a bug report
unless it says so.
**Refreshed:** 2026-10-19, from the book's own queue (`.agents/todo.md`), which now names the
package as the blocker on two numbered items — 83 (the stiffness ladder) and 85 (the quarter-car
numbers). Those two are §1 and §2 below; the rest is unchanged.

Two things moved on the book's side since this was written, and neither is a request:
the geometry is now reconciled (§"What the site already uses"), and the quarter-car trajectory is
extracted by a committed script that records the digests of the three config files it read — which
is what makes §2's second half urgent rather than tidy.

## What the site already uses

The book's Chapter 1 runs your release bundle directly: `out.js` for the four kinematic
models and the compiled kernels for the rigid one. Two figures depend on it — camber from the
rigid kernel against the page's own front view (they agree to **0.033°** over ±40 mm), and the
package's `agreement.max_abs_diff` (**2 × 10⁻¹³**) printed from the vendored export. The
quarter-car trajectory was extracted with your Python API and is quoted per step. Your three
limits checks (rank/redundancy, the monotone compliance ladder, energy drift) appear on the
pages as your results.

**The queue names you.** `.agents/todo.md` in the book holds open items only, and two of them
say the same thing in different words: item 83 is blocked on a joint the package does not offer,
and item 85 is a dependency the package cannot currently prove is unchanged. Both are quoted
below in the book's own words, so the acceptance here and there cannot drift apart.

**Geometry is reconciled on our side.** As of 0.4.0 the book uses your numbers: upper arm
`ucaf z = 0.480`, `ucar z = 0.470`, `tri/tro y = 0.14/0.12`, and `tyre-radius = 0.200`, which
puts the ground at **z = 0.12** rather than z = 0. The roll centre now agrees with your export
to **1 mm** (19 mm against 18.5 mm). Your `docs/follow-ups.md` entry "reconcile the book's
geometry with the examples'" can be closed from where we sit; the only judgement left is
whether 0.200 is the radius you want (see §5).

## The requests

### 1. A compliant joint at the upper ball joint, and a stiffness study (the book's item 83)

**Why.** §1.5 of the book asks how stiff a bush at the **upper ball joint** must be before the
bushed model is within a tenth of a degree of the rigid one. The ladder is the section's
evidence, and it is currently the page's own model. Your compliant example softens the **tie
rod**, which is a different joint: it moves toe, not camber. Measured on the 0.4.0 kernel, with
the book's geometry:

| tie-rod compliance | toe | camber |
|---|---|---|
| 10⁻⁶ m/N | 0.009° | −0.0003° |
| 10⁻⁴ m/N | 0.58° | −0.015° |
| 10⁻² m/N | 1.46° | −0.037° |

A hundred times softer moves toe by 1.5° and camber by 0.04°. The book now says that, because
it is the right lesson — *which joint you soften decides which angle moves* — but it cannot
produce the ladder it asks for.

**Ask.** Allow `compliance` on a joint the book can use (`ubj`, or any spherical joint), and a
study axis over it, so that one model swept at k ∈ {0.2, 1, 10, 100} MN/m gives the ladder.
Your `geometry.yaml` already has the shape (`joints.<name>.compliance`); it needs the joint and
the sweep.

**Acceptance.**

- `otkd emit examples/… --format js` for a compliant-`ubj` model at four stiffnesses reports
  camber at rest and the change over ±40 mm for each.
- As compliance → 0 the curve is the rigid one, and your existing
  `test_zero_compliance_is_the_rigid_model_exactly` covers the endpoint.
- `otkd emit --format wasm` compiles it, with the compliance in `numbers[]` (as the tie-rod
  model already does).

The book's own done-means, so the two lists agree: a compliant-`ubj` model in the package sweeps
stiffness and its export reports the ladder; §1.5's table is generated from the vendored export
rather than typed; the page's model and the export agree to the rounding shown, and the panel says
which is which; and `grep -n "one stiffness" books/maths-applied/docs` prints nothing.

### 2. A dynamics trajectory with outputs per step — and the model that produced it (the book's item 85)

**Why.** `otkd solve --json` on a `dynamics` study writes `t`, `energy` and `constraint` only.
§1.6 needs what the *page* talks about at each step: travel and camber. We extracted it with a
30-line script against `problem.world_points` and `outputs.compute`
(`books/maths-applied/docs/assets/data/quarter-car/extract.py`), which works but is a second
implementation of your export, and it has to be re-run by a person.

**Ask, first half.** Either add the standard outputs to each trajectory sample (`travel`,
`camber`, `toe`, and `wc`), or a `--poses` flag on `solve` that writes them.

**Acceptance, first half.** `otkd solve examples/quarter-car --json` contains, per step, the same
fields as a `sweep.frames[]` entry, or a documented flag does. The energy and constraint remain.

**Ask, second half — and this one is a correctness question, not a convenience.** The bundle
ships nothing for a dynamics model: the manifest lists `quarter-car` with `"none (dynamics)"` for
both payload and kernel, which is right, and leaves the book unable to prove *which model* a
trajectory came from. The book vendors the trajectory separately, so a release that changed the
quarter car — adding a damper to the coil, say — would silently invalidate §1.6's table, figure
and its 0.006° agreement with nothing failing anywhere.

**What we do have.** The extraction writes the `sha256` of the three files it read —
`topology.yaml`, `geometry.yaml`, `study.yaml` — into `meta.sources` in the vendored
`trajectory.js`, and the data README repeats them. That is a ledger with nothing to check it
against, and it is short of the model as well: the constraint kinds the joints name live in
`examples/constraints.yaml`, outside the model directory, so a release that changed what `spring`
means would change this model without changing one recorded digest. Hashing the four files, or
hashing the bundle's copy of the library once, closes both gaps.

**Acceptance, second half.** For a dynamics model the manifest carries those digests — the four
config files' `sha256`, and ideally the bytes — so that a test in the book compares
`meta.sources` against the vendored bundle and fails when they disagree. No kernel and no
payload: the request is only that the bundle can witness the model.

The book's own done-means: the package's quarter car is unchanged at the vendored version, or
§1.6 is re-derived; if the sources are ever vendored, its data test compares the trajectory's
source digests with the bundle's files; and §1.6's table and figure are regenerated together,
never one without the other.

### 3. The SVD diagnosis in the payload

**Why.** §1.4 teaches that three rows and two unknowns always give a zero singular value, and
that the *null vector's support* is the evidence of which row is dependent. The page computes
that itself. Your solver already computes exactly this (`solvers/svd.py::diagnose`: rank,
redundancy, dependent rows, null-space basis, condition), and `otkd check` prints the rank and
redundancy — but `out.js` does not carry the diagnosis, so the page cannot read it.

**Ask.** A `diagnosis` block in `out.js`: `singular_values`, `rank`, `redundancy`,
`dependent_rows`, `condition`.

**Acceptance.** `otkd emit examples/double-wishbone-two-leg out.js` carries it, and it says
what `otkd check` says: **21 unknowns, 22 rows, rank 20, redundancy 1**.

### 4. A stiffness axis in the study shape (small)

If §1's study axis is more than a config change, the alternative is a `scalar` that the
`js` emitter sweeps. Either way, the shape the book needs is: one model, four stiffnesses,
one export.

### 5. `tyre-radius`, and which datum is the ground (small) — **answered on our side, still worth a sentence from you**

Your examples declare `tyre-radius: 0.200` with the wheel centre at `z = 0.32`, so the
exported contact patch is at `z = 0.12` and the roll centre is measured from there. We adopted
that number so the book and the payload agree, and the page now states the ground plane
explicitly. The book's figure and the payload therefore part company nowhere. If 0.200 was a
placeholder and 0.320 was intended, tell us and we will follow the release that changes it — but
the two repositories should not disagree about where the ground is.

## What we are **not** asking for

- No change to the rigid path: its numbers are what the book's chapter is built on.
- No change to the ABI: the vendored kernel modules are working unchanged.
- No new example model unless you want one. The requested sweeps are studies of the existing
  models, not new mechanisms.
- No damping in the quarter car until §2's second half exists. Adding it is a change to §1.6's
  numbers, and the order is: release, ledger digest, re-derive the chapter — never the reverse.

## What would make a request wrong to attempt

If a compliant spherical joint cannot be represented without changing the problem family — if it
breaks the rank classification, or the kernel refuses it for a reason that is about the
mathematics rather than the plumbing — say so and stop. An approximation built out of a compliant
*distance* joint would give a ladder of the right shape and the wrong physics, which is worse than
a typed table, and the book would print it.

## If you would like the book's side of the contract

`books/maths-applied/docs/assets/data/otkd/README.md` in
[`Open.Textbook`](https://github.com/leonpjvaughan/Open.Textbook) records how the bundle is
vendored: one pinned version and zip digest, verified offline against your `SHA256SUMS`. A new
release is picked up by bumping two constants in `scripts/vendor-otkd.sh` and re-running it.
