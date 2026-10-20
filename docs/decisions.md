# Decisions — rounds 1 and 2

Everything below is **accepted**; the documents are now the contract and this file is the
record of why. Where a decision replaced a handoff sketch, the handoff's own words are
quoted so the change is visible.

## Round 1

| # | question | decision |
|---|---|---|
| 1 | Three files or two? | **three**: `topology.yaml`, `geometry.yaml`, `study.yaml` |
| 2 | Mode vocabulary | **three modes**, and an actuator is a separate device, not a mode |
| 3 | What loads §4? | **a load case** in the study; acceptance 4 is "deflection under load" |
| 4 | Mobility by rank or by count? | **rank of the Jacobian**, with redundancy reported |
| 5 | Design pose convention | **identity** for every body; geometry is world-at-design |
| 6 | Rotation parametrisation | **rotation vector**, with conversions at the edges |
| 7 | Is §2's residual floor physical? | **no** — consistent model, floor is roundoff |
| 8 | How is the redundant row removed? | **`drop:`** by row name in the study |
| 9 | Actuator effort types in v1 | **`position` only**; `force`/`torque` in the schema, refused by the solver |
| 10 | Does v1 ship `validate`? | **yes**, and `export` runs the same checks |
| 11 | Where do constraint kinds live? | **`constraints.yaml`**, a library the user supplies |

### 1–2. Three files, three modes

A sweep range is a question, not a property of a mechanism, so it moves out of `Geometry`
into `study.yaml`. Driving is not a fourth kind of constraint but a device that prescribes a
coordinate the model says is free: that separation is what lets one topology be swept, held
or loaded without an edit, and it keeps mobility a property of the model and the actuator
count a property of the study.

### 3. §4 needs a load

With a `spring` component and no applied load, equilibrium along the free direction is
`−δ/c = 0`, so `δ = 0` for every compliance: the compliant pose equals the rigid pose to
machine precision, the stiffness ladder is flat, and acceptance 4 passes without
distinguishing anything. A bush deflects because the tyre pushes the wheel, so §4 carries a
vertical and a lateral force at the contact patch and the test becomes "the deflection under
load vanishes as the bushes stiffen". Cost: one `loads:` block, one potential term, one
validation rule. No new solver.

### 4. Mobility is a rank, not a count

Grübler counts rows and assumes they are independent. §2: `6(4−1) = 18` against
`3+3+5+3+3+1 = 18` rows gives mobility **0**, so the count-based check rejects the model the
chapter exists to explain. Two points pinning one rigid body fix it except for rotation
about the line joining them — six rows, five independent — so the hard-row rank is 17 and the
mobility is 1. A count cannot distinguish a redundant row from an independent one, and
redundancy is the subject of §2 and §3. The check is `n − rank(J_hard) == #position
actuators`, with `m − rank` reported; a rank re-checked per frame also finds a dead centre,
which the count could never do.

### 5–6. Identity design pose, rotation vector internally

Geometry is authored in world coordinates at design, and every body's local frame is that
world frame: no per-body pose is authored or can disagree with the points, `q = 0` is a
*known* solution, and a site's local coordinate is its design world coordinate. Rotation is
one rotation vector per body (`R = exp([φ]×)`): three parameters, no normalisation row, no
Euler order, one-to-one for `|φ| < π` with `∂R/∂φ` regular until `2π`. A quaternion plus a
normalisation row is the local fallback if expressions grow, because every row is written in
terms of `R`. Everything a person types — degrees, stiffness, Euler angles for an initial
pose — is converted once at load or export and nowhere else (`config-layer.md` §12).

### 7. §2 is consistent; its floor is roundoff

19 equations, 18 unknowns. If the legs agree, the 19th equation is implied by the other 18
and the least-squares minimum is zero up to floating point (`1e-16` per row, `cost` around
`1e-32`), which is what makes acceptance 2 — least squares ≡ the dropped-row direct solve to
`1e-9` — true. An inconsistent model would show a physical floor (`~1e-4` m for a 0.2 mm
error) and would break that test by three orders of magnitude. If the book wants a visible
floor it is a separate example named for it, with its own test.

### 8–10. The small ones

`drop: [lca-front.tx]` names what is removed, chosen after `svd_diagnose` reports
`dependent_rows`; `auto` is refused because *which* row of a dependent set is dropped is
arbitrary while the solution is not. `position` actuators only in v1, with `force` accepted
by the schema so files do not have to be rewritten later. `otkd check` ships,
because it is the command that makes the config layer usable and it is the same code
`export` already needs.

### 11. `constraints.yaml`

A library the **user supplies**, referenced by each topology as `constraints:` (defaulting to
`constraints.yaml` beside the topology file, resolved relative to it). Local kinds are
allowed inline and no name may be defined twice. A kind is data — a family and its
components' modes — so it belongs in a file a reader can open, and that file is then the one
place that defines what `spherical` means for that model.

Round 3 moved this one step further: the package ships **no built-in library at all**. The
only copy in the repository is `examples/constraints.yaml`, which is documentation and the
library the examples share. A model without a library is not a model, and saying so is
kinder than silently supplying a dictionary that happens to make it run.

---

## Round 2: the road-driven model (accepted, staged as v1.1)

The ask: hold the car still, move the floor, and read the suspension's behaviour against
ride height. That is the same mechanism with a different **inertial frame** and a different
**input**, which is exactly what the config layer should absorb without a new solver. It
needs three small additions, all of them already anticipated as extension points:

1. **A `plane` family**, component `gap`:
   `n · (x_B − x_A) − g = 0` — a point held a fixed perpendicular distance from a plane,
   i.e. a wheel of radius `g` on a road. One row, `locked`/`free`/`spring` like any other
   component, so a compliant tyre is the same machinery §4 uses.
2. **Roles.** `roles: {reference: chassis, road: road}` says which body the outputs are
   measured against and which body is the road. `reference` defaults to the fixed body;
   `road` is optional, and when present the contact patch is a *solved* point rather than a
   projection of the wheel centre.
3. **A convention, not a feature.** For the axis-aligned families the site's **z is the
   axis** — which is why `prismatic` frees `tz` and a vertical road slide is a site whose z
   is up. This replaces round 1's `prismatic` freeing `tx`.

`examples/ride-height/` is then: `chassis` fixed, `road` on a `prismatic` to it, a
`plane-gap` tyre from the road to the wheel centre, and an actuator on the road slide. Four
moving bodies, 24 rows, rank 23, mobility 1 — the arithmetic is in
[`problem-mapping.md`](problem-mapping.md) §8. The study sweeps ride height, and
`meta.travel.unit` becomes `"m"`.

**The test that earns it:** for each road-driven frame, read the lower arm's angle out of the
solved pose, solve the arm-driven model at that angle, and require every body pose to agree
to `1e-9`. That proves the two models are one mechanism, and that driving is a choice of
input rather than a change of model.

**Deliberately not included:** rolling without slipping. It is a relation between velocities,
not positions, so it is a dynamics constraint and the contact point slides in a kinematic
model (`dynamics.md` §6). Also not included: a full car. The road joint's mode map could be
`free: tz, rx, ry` for heave, roll and pitch, but four wheels on one road is then three
degrees of freedom and the mobility check will rightly demand three drivers or a load case.
That is the next example, not this one.

**Staging:** v1 is the handoff's four acceptance tests and does not wait for this. v1.1 adds
the `plane` family, roles, the example and its test — one row expression, one config key, one
example directory.

---

## Round 3: user-supplied configs and entry points (accepted)

The package is a *runner*, not a model with a config. Three decisions follow from that, and
all of them are in [`entry-points.md`](entry-points.md).

1. **No bundled default library.** `constraints.yaml` is yours; `examples/constraints.yaml`
   is an example. Resolution is a filename beside the topology, or `topology.constraints:`,
   always relative to the topology file so a model behaves the same however it is invoked.
   A missing library names both the path it tried and the key that changes it.
2. **A command line for the four things a user does**: `validate`, `run`, `export`,
   `kinds`, with `--study` so one model can be asked several questions. Exit codes are
   `0`/`1`/`2` so a script can tell a config error from a solver failure without reading
   prose. `run --json` and `export` share one serializer: a terminal and a figure cannot
   disagree about a number.
3. **A five-name Python API** — `load`, `validate`, `compile`, `sweep`, `export` — with
   `compile` separate from `load` because **the geometry is not baked into the derivation**.
   Points, axes, lengths, gaps and compliances are parameters of the compiled functions, so a
   design study compiles once and solves many geometries. This is a change to
   `problem-mapping.md`'s `Problem`, and it is what makes the config split's "one topology,
   many geometries" true rather than aspirational.

**Versioning.** Every config file may carry `version: 1`; omitted means 1, and an unknown
version is refused by name. Your files outlive a package upgrade, so they get one loud signal
instead of one quiet surprise.

**Not entry points.** No plugin system (a new *family* is code, a new *kind* is data), no
`--set` overrides (a study file is reviewable in a diff; a shell history is not), no
scaffolding command in v1 (copying `examples/<name>/` shows a model that already validates),
and no Pyodide build — that stays a comment in `emit/js.py`.

---

## Round 4: the name, the verbs, and the UI (accepted)

The package outgrew its first name. It is not a suspension package: nothing in the config
layer knows what a wishbone is, and `plane-gap` is as happy with a pantograph as with a tyre.
Its job is **config files in, mathematical problems out**, with solvers and emitters
downstream. See [`naming.md`](naming.md) for the whole argument.

1. **Name: `otkd` everywhere** — distribution, import and command, expanded once as *open
   textbook kinematics and dynamics*. One name in three places avoids the two failures that
   actually happen: `pip install` versus `import`, and the command versus the import. The
   distribution can be renamed later; the import should not be renamed twice.
2. **One distribution, subpackages by concern** — `config/`, `problem/`, `solve/`,
   `analysis/`, `emit/`, `cli/`, `ui/`. A split into separately versioned distributions
   (`otkd-ui` as an optional wheel) is a packaging change if it is ever needed, not a
   refactor, because the seam is already there.
3. **Verbs are the pipeline**: `check` → `compile` → `solve` → `emit`, plus `kinds` and
   `ui`. `validate`/`run`/`export` were renamed because `check` is a gate, `solve` is the
   job, and `emit` says what an exporter does once the artefact is a flag. `compile` is new
   because the derived problem was invisible, and it is the command a textbook wants:
   `--format latex` prints the equations the solver differentiates.
4. **The engine boundary is JSON** — `check`/`compile`/`solve` in, artefacts out. Python is
   the reference; `emit` is a formatter over `solve`'s payload, so the terminal, the UI and
   the book cannot disagree.
5. **A UI is planned, not built** ([`ui.md`](ui.md)): draft-first, round-trip YAML so a drag
   is a one-number diff, no JavaScript build step, and the UI computes nothing — it draws
   `solve`'s payload. The check badge (rank, mobility, redundancy, worst row) is the reason
   it exists.
6. **Rust is a window, not an engine, at first.** A small PyO3 launcher that opens the local
   URL in a webview, with `webbrowser` as the fallback. A Tauri shell with a sidecar is the
   next step if the window matters. A Rust engine is rejected until it reproduces `emit`'s
   bytes on the conformance models, because the book's claim is that the residual is derived.
   Embedding CPython in the shell is rejected for the obvious reason: it ships an
   interpreter to run sympy and saves one subprocess call.

**One contract value changes with the name:** the export's `meta.generator` becomes
`otkd <version>` (`HANDOFF.md` §7). The key is unchanged; a consumer that prints the
generator string will print a different one, and should be reading the version rather than
the name. `HANDOFF.md` keeps its own wording, and the repository name is the owner's to
change.

---

## Round 5: what building it changed

The implementation is in `src/otkd/`. These are the places where the code, or the model it was
asked to solve, disagreed with the design above; each is a deliberate resolution, not drift.

1. **Quaternion, not rotation vector** (round 1 §6 said rotation vector). The exponential
   map's derivative is `0/0` at zero rotation, which is the design position every model is
   assembled and validated at; making that safe needs series-guarded helpers with
   hand-supplied derivatives, against the project's one hard rule. A unit quaternion makes
   every row polynomial at the cost of one unknown and one normalisation row per body.
   `problem-mapping.md` §2 records the reasoning.
2. **Bodies own points.** `bodies.<name>.points` was not in the design; without it the outputs
   cannot know which pose moves a point like `wco`. Ownership is structural (names only), does
   not affect any constraint, and is validated to be exactly one body per point.
3. **`roles.axle`.** Camber needs an axle direction and no naming convention can supply one.
   Two point names, required, checked.
4. **Module names cannot collide with the verbs.** `otkd/solve/` and `otkd/check.py` shadowed
   `otkd.solve()` and `otkd.check()` the moment they were imported, because a package
   attribute wins over a function of the same name. They are `solvers/`, `emitters/` and
   `report.py`; the public names are unchanged. `naming.md` §3 has the tree.
5. **The equilibrium Jacobian is numerical.** The projected-potential block `Zᵀ∇Π` has a
   derivative containing `(∂Z/∂q)ᵀ∇Π`, which is zero at the solution but not on the way to it;
   supplying the truncated analytic block makes the trust region stall (197 evaluations, then
   failure). Differentiating the residual numerically costs eight evaluations per frame. The
   constraint Jacobian inside the residual stays symbolic.
6. **±0.1 rad, not ±0.35.** The handoff's "±0.35 rad ≈ ±40 mm" assumes a 0.2 m arm; the
   handoff's own geometry table gives a 0.42 m arm, for which ±0.35 rad is ±145 mm of wheel
   travel and a 290 mm sweep. The examples use ±0.1 rad, which is ±42 mm and the travel the
   handoff intended. The contract's *frame count* (81) is unchanged.
7. **The two-leg model is the upper arm.** Making the *lower* arm the two-leg one leaves the
   mechanism with no joint to drive (its freedom is the null direction of two sphericals), so
   the driven joint would change and the two camber curves would no longer be comparable.
   Putting the redundancy in the upper arm keeps `lca-ground.rz` and the comparison.
8. **The closed form covers a two-bush arm.** The analytic extractor treats a pair of
   sphericals to the reference body as a hinge line through their origins, so §2 gets an
   agreement panel too (9e-14), not just §1.
9. **The derivation is cached by topology and library.** Geometry is symbolic, so the compiled
   functions do not depend on the numbers; `compile` caches them, and `Problem.solve(geometry)`
   or `Model.with_geometry(geometry)` makes a design loop pay for the derivation once
   (10.5 s down to 1 ms for the second geometry).
10. **Roll centre uses the actual contact patch.** The handoff's parenthetical puts the contact
    patch at `x = 0`, which makes the construction degenerate. The implementation uses the
    patch's real position and the mirrored-corner assumption, and returns `null` when the
    instant centre is undefined or the line is vertical.
11. **The emitted module is compact.** One frame per line, one iterate per line, and the
    iteration history capped at 60 entries: 54 KB instead of 113 KB, which is the "tens of
    kilobytes" the handoff expects. `meta.generator` is `otkd 0.1.0`.

---

## Round 6: the compiled kernel (accepted)

`otkd emit --format wasm` writes a generated Rust kernel, builds it to WebAssembly, and inlines
it in an ES module. `tests/test_wasm.py` checks it against the Python engine; `docs/wasm.md` is
the explanation. The decisions worth recording:

1. **sympy → Rust → wasm, not C or Emscripten.** sympy's `rust_code` prints `sqrt`, `atan2` and
   `powi` correctly, and `std` on `wasm32-unknown-unknown` supplies exactly those maths
   functions. C would need libm (bare wasm) or wasi-sdk (bigger module); Emscripten brings libc
   and megabytes; Pyodide puts sympy in the browser, which is the wrong side of the boundary.
2. **The integer-literal trap.** sympy prints `2*x`, which Rust rejects against `f64`. Literals
   are promoted to `2.0` by a character walk that skips `.powi(...)` arguments and array
   indices — a regex cannot tell those apart from arithmetic.
3. **One static buffer, no allocator, no `wasm-bindgen`.** Fixed sizes are known at generation
   time, so the ABI is an offset table into one exported `f64` array. The one non-obvious thing
   is that offsets are relative to `MEM`, so JS starts at `MEM.value / 8`; getting that wrong
   reads and writes the wrong memory silently.
4. **`pack()` makes the kernel topology-scoped.** It completes each site's axes, defaults the
   tie-rod length and reads compliances inside the kernel, generated from the same symbol table
   as `Model.resolve()`. So one module serves every geometry, which is what a slider needs.
5. **Measured:** 0.09 ms per Newton frame against 7 ms in Python (about 75×), a 74 KB module
   (34 KB gzipped), and a Jacobian that agrees with the symbolic one exactly.
6. **Conformance is behavioural for the binary.** `kernel.rs` and `kernel.json` are byte-stable
   and diffed; `kernel.wasm` depends on the rustc version, so it is tested by solving the same
   drives under node and requiring 1e-9 on poses and 1e-12 on the Jacobian.
7. **The metric block is deferred.** The kernel returns the pose and the world points, not
   camber/toe/IC/RC; a page computes those from `world()` as `outputs.py` does. Generating the
   metrics too would remove the last duplication, and is the obvious next increment.
8. **How `maths-site` consumes it is deferred.** The recommendation is vendored artefacts plus a
   `provenance.json`, regenerated by one command, because the site's build is node-only and a
   submodule would drag Python, sympy and rustc into it. No private registry is needed.

---

## Round 7: releases as the transport for vendoring (accepted)

`HANDOFF.md` §2 says "Do not publish to PyPI. Do not add a release workflow." The first clause
stands — nothing is published as a package. The second is superseded: a release now carries the
compiled artefacts, because that is how a second repository gets them without a registry, and
because a versioned, hash-pinned zip is a better provenance story than a hand-copied folder.

1. **One zip per tag.** `scripts/build_release.py` builds every example's payload and kernel,
   writes a manifest and per-file digests, and packs a deterministic archive: sorted entries,
   fixed timestamps, no build time in the manifest. The release workflow rebuilds it and
   compares, so "reproducible" is checked rather than claimed.
2. **Pinned rustc, deliberately.** The `.wasm` bytes depend on the compiler; an unpinned
   toolchain would make every release's digest drift. The workflow pins `1.99.1` and bumping it
   is its own commit.
3. **The site fetches, it does not build.** `maths-site`'s CI downloads the pinned zip, verifies
   the `sha256`, unpacks the folders it needs into `assets/`, and builds. Its toolchain stays
   node-only, and the version and digest are two lines of configuration.
4. **Missing kernels are named, not hidden.** A redundant or compliant model has no compiled
   solver yet; the manifest records the reason, so the zip is honest about the current limit.
5. **The vendored folder is either ignored or committed, never both.** The site's `assets/` tree
   is drift-checked, so a build that writes into it would dirty the tree; ignoring the folder and
   producing it in the build is the direct reading of "download it in the pipeline", and
   committing it keeps local builds offline. Recorded as the site's choice.

---

## Round 8: versions from commits, releases from `main` (accepted)

`HANDOFF.md` §2's "do not add a release workflow" is superseded (round 7); this round decides
*how* a version is chosen.

1. **Conventional Commits are the version source.** `feat` → minor, `fix`/`perf` → patch, `!`
   or a `BREAKING CHANGE:` footer → major, everything else → no release. While the version is
   `0.x`, a breaking change moves the minor: before 1.0 every change may break, so declaring an
   API stable by jumping to `1.0.0` would be a claim nobody has made. `scripts/version.py` is the
   implementation and is tested against a table.
2. **`dev` is where work happens; `main` is release-only.** Every merge to `main` is a release
   attempt; if the commits do not warrant one, nothing is released, so the flow is idempotent.
3. **No `v` prefix.** The tag *is* the PEP 440 version, `0.2.0`.
4. **The version is written into `src/otkd/__init__.py` by the release job**, not derived from
   `git describe`. A git-derived version changes with every commit, and `meta.generator` sits in
   every emitted artefact: the committed `out.js` would go stale after every commit and the
   byte-identical regeneration rule would be dead. One commit changes the version, and it is the
   release commit.
5. **The release job regenerates the artefacts it just versioned**, runs the tests, commits
   `chore(release): <version>`, tags, pushes, builds the bundle, rebuilds it to prove
   reproducibility, and publishes with `gh`. Nothing goes to PyPI.
6. **Malformed commits fail the pull request** (`scripts/version.py lint`), because a commit that
   does not parse is a release that silently does not happen.

---

## Round 9: the tag is the version (accepted, supersedes round 8's version storage)

Round 8 decided the version from Conventional Commits and stored it in
`src/otkd/__init__.py`. Round 9 keeps the decision rule and moves the storage to the tag:

1. **`hatch-vcs` serves `otkd.__version__` from the git tag.** No version text exists in the
   repository; a tag is the only thing that changes the version. A push to `main` decides the
   number, builds everything at it, and creates the tag only once the artefacts exist, so a failed
   build leaves no orphan tag.
2. **The payloads no longer carry the version.** `meta.generator` is `"otkd"`. A scm version is
   `0.2.0.dev3+gabc1234` between tags, so a version inside every `out.js` would dirty four
   committed artefacts after every commit and break the byte-identical regeneration rule. The
   version lives on the release, in `MANIFEST.json` and `provenance.json`. This is the second
   deliberate change to a `HANDOFF.md` §7 *value* (the first was the name); the key is unchanged.
3. **The package is built into the bundle.** The wheel and sdist are built with
   `SOURCE_DATE_EPOCH` pinned and `SETUPTOOLS_SCM_PRETEND_VERSION` set to the decided version,
   which is what makes the zip's rebuild-and-compare meaningful. Nothing is published to PyPI.
4. **`scripts/version.py` keeps one job**: deciding the number. It no longer writes anything, and
   the hand-rolled release script is gone — the tag is created by CI, not by a local command.
5. **Conventional Commits still gate the merge.** The `commits` job fails a pull request whose
   commits do not parse, because a malformed commit is a release that silently does not happen.

---

## Round 10: limitations that turned out to be improvable (accepted)

The first release review listed three limitations. Two were real, one was not what it looked
like, and the rest is now an explicit follow-up list in [`follow-ups.md`](follow-ups.md).

1. **Instant centre.** The front-view construction is exact only when the hinge axes are
   perpendicular to the view: the projection of a tilted 3D rotation is a rotation *plus a
   stretch*, so no exact front-view centre exists at all. The approximation was kept — it is the
   standard construction the book teaches — but the pivot is now the **foot of the perpendicular
   from the ball joint onto the hinge line** rather than an arbitrary bushing origin, and a test
   pins the exact case and the two-bush arm.
2. **Compliant `iterations[]`.** The equilibrium solve now uses a trust-region solver with a
   callback, so the export carries **accepted iterates** (three or four per frame) instead of
   every residual evaluation (over a hundred), matching what the key means in §1 and §2.
3. **Wheel travel.** Not a code limitation: the handoff's ±0.35 rad assumed a 0.2 m arm, and its
   own geometry has 0.42 m. The examples use ±0.1 rad (±42 mm), and a test now fails if the
   stroke drifts out of a 70–100 mm band.
4. **Dynamics** is prepared rather than started: the config is validated at load, and
   [`dynamics.md`](dynamics.md) §7 is a task list with acceptance tests, so the work is bounded.
5. **The compiled kernel's remaining stages** — metrics, least squares, SVD and equilibrium —
   are accepted as [`follow-ups.md`](follow-ups.md) §1, each with a done condition. The UI is
   deferred and may never be built; nothing depends on it.

---

## Round 11: kernel metrics, readable equations, and the command line (accepted)

1. **The kernel computes the drawn quantities.** `metrics()` writes camber, toe, track, the
   contact patch and the instant and roll centres, and the conformance test compares them with
   `analysis/outputs.py` to `1e-9`. They are formulas in the template rather than symbolic
   expressions because the instant centre must report *undefined* for a parallel pair, which a
   plain expression cannot. The ABI is now `otkd-kernel-2` because the memory layout grew.
2. **`meta.generator` stays `"otkd"`.** A tag-derived version inside a committed payload would
   change on every commit; the version belongs to the release bundle. This answers the question
   directly: the version *is* dynamic, in `MANIFEST.json` and `provenance.json`, where it cannot
   churn.
3. **The LaTeX printer uses display names.** `P_{\text{lcaf},x}`, `C_{\text{lca-hinge},11}`,
   `L_{\text{tie-rod}}`, `\theta_{\text{lca-drive}}` — the command exists to typeset the book,
   so the symbols have to read as the prose does.
4. **The command line is tested end to end**, including exit codes, rather than only through the
   library.
5. **The book-chapter proposals are gone from `docs/`.** They were copied to the book repository;
   this tree holds the package's design record only.

---

## Round 12: least squares in the kernel (accepted)

1. **The emitter chooses a solver from the classification**, not from a flag: damped Newton for a
   square model, Levenberg–Marquardt on the normal equations for a redundant one, and a refusal
   naming the gap for a compliant one. `kernel.json` records `solver`.
2. **The Jacobian layout was fixed for non-square systems.** The generator had iterated the
   Jacobian as if every model were square; it now writes row-major `jac[row*N + col]`, which is
   the same layout for a square model and the only correct one for a redundant model.
3. **The Newton step no longer clobbers its linearisation.** A rejected trial used to overwrite
   the residual and Jacobian of the accepted iterate; rejected trials now use their own buffers.
   The bug was invisible while full steps were always accepted.
4. **A compliant model is still refused**, with the reason, rather than emitted. That is the last
   remaining solver.

---

## Round 13: roundoff is reported as a bound (accepted)

The first CI run failed the "emitted data is committed and stable" check, and it was right to.
`agreement.max_abs_diff` came out `1.02e-13` locally and `1.34e-13` on the runner, and the
compliant model's iterate trace differed in its last digits. Those digits are a property of the
BLAS and the scalar library, not of the model: at roundoff, a number is a *bound*, not a
measurement.

- The agreement is now rounded to **one significant figure** — "they agree to 1e-13" is the claim
  a panel should print, and its second digit means nothing. This supersedes `HANDOFF.md` §7's
  "three significant figures", which assumed a platform-stable value.
- The iterate trace is rounded to three significant figures for residuals and four for singular
  values, and **a residual below `1e-9` is reported as zero**. Below the solver's own tolerance a
  residual is roundoff: its digits say what the BLAS did, not what the model is. A first attempt
  with a `1e-15` floor still differed in CI, which is how the floor found its right value.
- The converged data — poses, metrics, camber to four decimal places — was never affected; the
  diff in CI showed no frame changed.

---

## Round 14: the sdist must not contain the build output (fix)

The first release run failed its own reproducibility check, correctly: the two bundles differed.
The cause was not numerics but packaging. When the bundle is written into `dist/` inside the
repository, `uv build` packages the project directory *including* `dist/` — so the second build
contained the first build's archive, and the two could never match. The sdist now excludes
`/dist*`, `/.venv`, `/build` and the pytest cache, and the release is reproducible with the
output inside the tree. Build output in the source tree is the kind of thing that works once.

---

## Round 15: the compliant equilibrium, compiled (accepted)

1. **The last solver is in the kernel.** A compliant model gets Newton on
   `[constrained rows ; Ẑ·∇Π / f_ref]`, with the study's load case baked in as constants, and it
   agrees with the Python solver to `1e-12` on the pose and `1e-10` on the metrics.
2. **No SVD was needed.** The free direction is one-dimensional, so appending the spring row's
   gradient to the constrained Jacobian makes a square, non-singular matrix whose solution spans
   the null space — one linear solve with the LU the kernel already had. The SVD diagnosis stays
   in Python because §3's figure shows its result rather than asking a reader to move a slider
   through a decomposition.
3. **The projected potential is differentiated numerically**, by central differences, for the
   same reason the Python solver does: the analytic block contains `(∂Z/∂q)ᵀ∇Π`, which is the
   term that stalled both implementations.
4. **The bundle is self-describing.** `kernel.json` carries `solver` and `default_numbers` — the
   design values for the length, gap and compliance slots — so a page can pack a geometry without
   reconstructing the emitter's conventions.

---

## Round 16: accelerations and reactions (accepted)

1. **v2.0 is implemented** in `src/otkd/problem/dynamics.py`: the saddle-point system
   `[[M, Φ_qᵀ], [Φ_q, 0]] [q̈; λ] = [f; γ]`, with `M` assembled per body from the centre-of-mass
   Jacobians (7×7 and dense, because a body's translation and rotation couple through its
   centre-of-mass offset), springs and dampers as forces, and `γ` differentiated numerically
   along `q̇`.
2. **Springs leave the constraint set.** In dynamics a compliance is a force, not an equation, so
   `Φ` is the hard rows, the normalisation and the drivers — the kinematic set minus the springs.
   That is why the same model solves both ways without a second derivation.
3. **The tests assert what can be asserted exactly**: zero acceleration at a converged
   equilibrium, the mass matrix's symmetry and its `½mv²` limit, and — independently of how `γ`
   is computed — that the acceleration reduces the constraint residual from `O(h²)` to `O(h³)`.
4. **No study shape yet.** A dynamics study needs a trajectory, which is v2.1; until then the
   entry point is a function of a state, not a sweep.

---

## Round 17: time stepping (accepted)

1. **RK4 with projection after every step.** The constraint residual is held at roundoff rather
   than allowed to drift by `O(h⁴)` per step, and the state is projected onto both `Φ = 0` and
   `Φ_q q̇ = 0`. Baumgarte stabilisation was not used: it trades constraint drift for energy drift,
   which is the quantity the test measures.
2. **Energy includes the applied loads.** `V` gains `−F·x` for every load. Without it the "energy"
   of a loaded mechanism is not conserved — the first version of the test measured the load's work
   instead of the integrator, and the drift did not shrink with the step size, which is how the
   omission was found.
3. **A dynamics study is a time span.** `analysis: dynamics` requires `duration` and `steps`,
   ignores `drivers`, and `otkd solve` prints energy and constraint residual per step (`--json`
   writes the trajectory). `emit` refuses it: the export contract is a sweep.
4. **The stiff-spring limit is documented by the test.** The energy test uses a softer tie rod
   because a 1 kN/mm spring oscillates at ~290 rad/s and RK4 needs a much smaller step than the
   test takes; the drift is the integrator's, not the model's.

---

## Round 18: merge commits, not squash (accepted)

The squash-only setting produced a footgun that the first three releases walked into: after a
squash merge, `dev` is no longer an ancestor of `main`, so the next pull request shows as
conflicted until `dev` is reconciled — and the obvious reconciliation (`git reset --hard
origin/main`) cannot be pushed back to a branch that forbids force pushes. The repository now
takes **merge commits** (squash and rebase disabled, linear history not required), so `dev` stays
an ancestor and a release never needs a ritual. The version calculation always ignored merge
commits (`git log --no-merges`), so nothing about versioning changes; the pull request title stops
mattering, and the `commits` job keeps the individual commits conventional.

---

## Round 19: the velocity-product term, and a quarter car (accepted)

1. **`C(q, q̇)` was missing and is now there.** The equations were `M q̈ = f − Φᵀλ`, which is only
   right when `M` is constant. The quarter-car energy test found it: the total energy varied along
   the motion by `2.8e-3` relative, and — the tell — the variation did not shrink when the step was
   divided by four. The term is `Ṁ q̇ − ∂(½q̇ᵀMq̇)/∂q`, built from two finite differences over the
   mass matrix. With it, the drift is `3.6e-8` at 101 steps and `1.3e-10` at 401: it now converges.
   Every earlier dynamics test used a state at rest or a tangent velocity, where `C` vanishes, so
   nothing before this had exercised it.
2. **`examples/quarter-car/`** is the multi-body example: a heaving chassis, a coil-over expressed
   as a compliant `distance` joint, a tyre on a road plane, and a dynamics study with no loads.
3. **`check` understands dynamics studies.** A dynamics model is *supposed* to have freedom, so the
   rule becomes "the constraint set is independent" (`redundancy == 0`) and the freedom is
   reported, instead of demanding that the freedom equal the number of actuators. Springs are
   excluded from the constraint rank, because in dynamics a spring is a force.
4. **A full four-corner model is measured, not guessed**: 13 bodies, 91 unknowns, 167 s to derive,
   11 ms per Jacobian. Feasible for a figure, not for real time — the compiled kernel is the
   answer, and the derivation is the wall, not the equations.
5. **A dynamics model ships neither a payload nor a kernel**, and the release manifest says so.

---

## Round 20: CI split, and two things not done (accepted)

1. **The kernel tests are their own CI job.** `tests/test_wasm.py` builds three Rust modules and is
   several minutes of work for one interpreter's worth of information; the main matrix now runs
   everything else and the `wasm` job runs that file once, on one Python.
2. **The four-corner model stays a measurement.** It compiles (167 s for 91 unknowns) but an example
   CI cannot afford to load is documentation, not an example. If a figure needs it, the compiled
   kernel or an on-demand script is the route.
3. **Two repository items are not happening**: the secret-scanning non-provider patterns and
   validity checks (the API will not set them and the manual toggle is declined; push protection is
   on), and renewing the interaction limit when it lapses in 2027-04. Issues and discussions stay
   disabled, so a lapsed limit reopens comments on pull requests and nothing else. Both are recorded
   as declined in [`security.md`](security.md) rather than left as open tasks.

---

## Round 21: the book's two requests, accepted in the package's shape (accepted)

`maths-site` asked for two capabilities in [`handoff-from-maths-site.md`](handoff-from-maths-site.md).
Both are accepted, the shape is changed in three places, and one request is dropped.

1. **A trajectory sample carries the same quantities as a sweep frame.**
   `analysis.outputs.quantities` names them; `emitters.js.sample` rounds them; `otkd solve --json`
   writes, per step, `camber`, `toe`, `ic`, `rc`, `track`, `contact` and every world point, beside
   `t`, `energy` and `constraint`. `theta` and `branchChanged` are sweep concepts and are absent,
   which is the only difference. The book's own extractor can then go: a second reader of the model
   was the thing to avoid, not to duplicate.
2. **The payload's key order is the quantity table's.** `contact` moved before `branchChanged`, so
   the four committed `out.js` files were regenerated. Every frame was compared field by field with
   its predecessor before committing: the values are identical and only the order is canonical. The
   reorder is not new data and is not announced as though it were.
3. **Every model in the bundle carries its config files and their digests.** The kernel already
   wrote `kernel/source/` and recorded digests in `provenance.json`. The bundle now writes
   `models/<id>/source/` and `manifest.models.<id>.sources` for *every* model, which is the only
   provenance a dynamics model has: it ships no payload and no kernel, so a consumer that vendored
   a trajectory separately had nothing to compare it against. The library is included because it
   lives outside the model directory — a release that changed what `spring` means would otherwise
   change the model with no digest moving. The format is now `otkd-bundle-2`: a promise to witness
   provenance should be visible in the promise's own version.
4. **A compliant upper joint and a stiffness axis are accepted as a follow-up**, in the package's
   shape rather than the book's: the axis is a number iterated *outside* the travel driver, each
   inner sweep seeded from the design pose because a soft joint and a stiff joint are different
   branches, and the book's ladder is gated on a measurement that camber responds. Their evidence
   is that the tie rod moves toe by 1.5° and camber by 0.04°, so "soften the ball joint instead" is
   a hypothesis until a run shows it.
5. **Dropped: the SVD diagnosis in `out.js`.** It has no item in the book's queue, so nobody is
   waiting for it. `otkd check` prints the diagnosis, and the chapter computes it as the exercise
   it is; a payload field with no reader is a wish, and a request with no owner should not sit in a
   handoff. It can return with an item.
6. **Governance.** The book's acceptance was "agrees to the rounding shown", which makes the
   package answerable to a figure. It is the other way round: the package's own numbers are the
   source, and where the page disagrees the page changes. A package tuned to a textbook cannot then
   be evidence for it.

---

## Round 22: the spring gradient, and what a stiffness axis would take (accepted)

1. **`∇Π_spring` is assembled from the rows, not differentiated.** `Π = ½ Σ gᵢ²/cᵢ` is a sum over
   the compliant rows, so `∇Π = Σ (gᵢ/cᵢ) ∇gᵢ` — the row Jacobian times a per-row weight, both of
   which the residual already computes. The symbolic gradient was a *new* expression per compliant
   component, and it does not scale: a model with three compliant translation components at one
   frame joint took **63 s** to compile and then never returned from the gradient at all. With the
   assembly, that model compiles in about a second, and the committed compliant payload regenerates
   byte-identically, so the number is unchanged. `Problem.hessian` was dead code carrying the same
   defect and was removed with it.
2. **The measurement the follow-up needed, and it is a no for now.** The same three-component joint
   classifies as `equilibrium`, checks clean, and **does not converge** — at k = 0.2, 1, 10 and
   100 MN/m it stops on `xtol` with a residual of 0.056, about 300 N of unbalanced force. The hard
   rows are exact at the design pose (7e-18), so this is the projected-potential part, and the
   likely mechanism is the null-basis sign anchoring: it anchors each column against a *spring
   row's* gradient, and several compliant rows at one joint can have near-parallel gradients, so
   the sign flickers and the trust region stops at a kink. Recorded in
   [`follow-ups.md`](follow-ups.md) with the route: anchor against a carried-over direction or the
   design pose rather than against the spring rows.
3. **A stiffness axis is a study shape, not an example.** An axis is a *number* iterated outside
   the travel driver; `axes:` joins `drivers:` and `loads:` in `study.yaml`, each entry naming a
   `g` symbol with a law. Nesting is axes outside, drivers inside, and each inner sweep is seeded
   from the **design pose**: `solvers/sweep.py`'s predecessor seeding exists for branch continuity
   *within* a sweep, and a soft joint and a stiff joint are different branches. Frames carry both
   values, so the payload's `sweep.frames[]` stays one file with `N × M` entries — which is what
   the book asked for — and the kernel is untouched, because it already takes the compliance as a
   run-time number.
4. **The book's ladder stays gated on a measurement.** Until the equilibrium converges and a run
   shows camber responding, the package says no rather than shipping a ladder of the right shape
   and the wrong physics.
