# HANDOFF — `suspension-kinematics`, v1

**Status:** ready to execute. The repository exists and holds this file, a README and
the MIT licence. Everything else is yours to build.
**Written:** 2026-10-18
**Reader of this file:** an agent with no prior context for this work.

You are building a small, public Python package that solves the kinematics of a
suspension linkage and emits the numbers a set of figures draws. It exists because a
book about *modelling* needs a model it can point at: the mechanism's connectivity and
its geometry are separate data, the solver is derived rather than hand-written, and
every claim the solver makes about itself is a test.

Read this file once, end to end, before writing code. **Open questions are at the
bottom.** Resolve them with the author rather than guessing; they are listed because
each one changes a public contract.

---

## 1. What this package is for

One consumer: `books/maths-applied` in the `maths-site` repository. Its chapters are

| § | Model | Solver it needs |
|---|---|---|
| 1 | wishbone as one rigid body, one degree of freedom | square Newton solve, checked against a closed form |
| 2 | the same wishbone as **two legs**, so the model is redundant | least squares, with a singular value reported |
| 3 | the same, diagnosed | SVD: singular values, null space, which rows are dependent |
| 4 | bushes instead of a rigid constraint | algebraic equilibrium, swept the same way |

The book's argument is executed in this repository's test suite: **analytic ≡ numeric**,
**least squares ≡ direct once the redundancy is removed**, **SVD finds exactly one null
direction**, and **stiffness → ∞ equals the rigid model**. Those four tests are the
deliverable that matters most. Code that passes them is done; code that does not is not,
however tidy.

### Why the numbers are precomputed

A one-degree-of-freedom mechanism needs about two hundred frames of a handful of numbers
each — tens of kilobytes. The consumer emits them as a **static ES module**, so there is
no runtime network, no download on a phone, no async race, and it works offline. The
payload shape is the same whether a sweep is produced by the CLI at export time or (later)
in a Web Worker by Pyodide, so a future live path is a drop-in for one panel.

**Design the seam, do not build it.** One comment where the worker would go, and no more.

---

## 2. Ground rules

- **MIT**, already in `LICENSE`. No per-file licence headers unless the author asks.
- **`uv` + `hatchling` + `src` layout.**
- **`requires-python = ">=3.12"`** (the consumer's engine is `>=3.12`; change only if the
  author says so — see open question 2).
- **`scipy` is allowed here**, and only here. The consumer's engine declined scipy for a
  different job (proving textbook answers to a stated accuracy); this is a numerical
  solver, where `scipy.optimize.least_squares` and `scipy.linalg.svd` are the right tools.
  `numpy`, `scipy`, `sympy`, `pydantic`, `PyYAML` are the runtime dependencies. Nothing
  else without a reason in `pyproject.toml`.
- **Type hints everywhere**, and `pydantic` models for anything a reader of the YAML
  writes.
- **No hand-differentiation. Anywhere.** The Jacobian is symbolic (see §5).
- **The emitted `.js` is committed** and its regeneration is byte-identical. A build
  artefact that changes on every run cannot be reviewed in a diff.
- **Do not publish to PyPI.** Do not add a release workflow.
- Comments explain *why*, not what. The consumer's repository holds a large body of
  prose-as-design-record; match that tone: calm, specific, no exclamation marks.

---

## 3. Layout

```
pyproject.toml
README.md
LICENSE
HANDOFF.md
src/suspension/
  __init__.py
  config.py       Topology, Geometry, Model, Driver, Sweep, validation
  joints.py       revolute / spherical / distance: residual contributions
  derive.py       sympy: residual + Jacobian -> lambdify(cse=True)
  analytic.py     circle-circle, circle-sphere closed forms
  solve.py        newton(), least_squares(), svd_diagnose()
  sweep.py        drive the DOF, tabulate, enforce branch continuity
  outputs.py      camber, toe, instant centre, roll centre, track change
  export.py       emit an ES module
  cli.py          `suspension export <model> <out.js>`
examples/
  double-wishbone/            topology.yaml geometry.yaml     (rigid, 1 DOF)
  double-wishbone-two-leg/    topology.yaml geometry.yaml     (redundant, least squares)
  double-wishbone-compliant/  topology.yaml geometry.yaml     (bushes, §4)
tests/
  test_config.py
  test_analytic_matches_numeric.py     acceptance 1
  test_least_squares_matches_direct.py acceptance 2
  test_svd.py                          acceptance 3
  test_stiffness_limit.py              acceptance 4
  test_export.py                       determinism and shape
.github/workflows/tests.yml
```

Each example directory holds exactly two files — `topology.yaml` and `geometry.yaml` —
and exports its own committed `.js` beside them (e.g. `examples/double-wishbone/out.js`).
The three examples share one topology *shape* and differ on purpose: that difference is
what §2 and §3 of the book are about.

---

## 4. The config split — do this first

Everything else rests on it. Get it right before writing the solver; a change here
afterwards invalidates the derivation, the export and the tests at once.

```python
class Joint(BaseModel):
    name: str
    kind: Literal["revolute", "spherical", "distance"]
    body_a: str                 # body name; "ground" is the chassis
    body_b: str


class Driver(BaseModel):
    """Which joint is driven, and about which axis. Names only — no numbers."""
    joint: str                  # a Joint.name
    axis: str                   # a Geometry.axes key


class Sweep(BaseModel):
    """The driven coordinate's range. A number, so it lives in Geometry."""
    start: float                # radians, relative to the design position
    stop: float
    steps: int                  # inclusive count


class Topology(BaseModel):
    """Connectivity only. No coordinates, no lengths, no masses, no angles."""
    bodies: list[str]
    joints: list[Joint]
    driver: Driver


class Geometry(BaseModel):
    """Every number, keyed by names the Topology declares."""
    points: dict[str, Vec3]
    axes: dict[str, Vec3]           # directions, not positions
    lengths: dict[str, float]       # fixed lengths: distance joints, tyre radius
    sweep: Sweep
    stiffness: dict[str, float] = {}   # bushing stiffness, per distance joint


class Model(BaseModel):
    topology: Topology
    geometry: Geometry
```

Requirements:

- **`Topology` contains no numbers.** A sweep is a set of numbers, so it belongs to
  `Geometry`; the driver names a joint and an axis and nothing else. This deliberately
  differs from an earlier draft that put the range in `Driver` — stated here so it is not
  "fixed" back.
- **`Geometry` contains all of them**, including the design positions, the axis
  directions and the fixed lengths.
- `Model` validates that **every name `Topology` references exists in `Geometry`**, and
  fails loudly and specifically: a missing point names the point, the joint and the file
  it should have been in. A silent default is the defect this check exists to prevent.
- `Model` computes the **mobility** of the topology (Grübler/Kutzbach: `6·(n−1) − Σ
  constraints`, with revolute 5, spherical 3, distance 1) and **fails unless it is the
  number of driven coordinates** — 1 for every example. A model with two degrees of
  freedom and one driver is under-determined, and it should say so at load, not converge
  to something.
- **One topology, many geometries.** A sweep is a loop over `Geometry` only. This is what
  makes design optimisation possible later without re-deriving constraints; it is also
  what makes §4 (a stiffness slider) a loop rather than a rewrite.

Coordinates, once (see open question 3): **x lateral, positive outboard for the corner
modelled; y forward; z up; metres.** A right-front corner is the default example.

Example geometry, as a starting point — plausible, documented as illustrative and not a
real car (first-pass numbers; tune so the model is well conditioned and the camber curve
is believable):

| point | x | y | z |
|---|---|---|---|
| `lcaf` lower arm inner, front | 0.300 | 0.120 | 0.150 |
| `lcar` lower arm inner, rear | 0.300 | −0.120 | 0.160 |
| `lbj` lower ball joint | 0.720 | 0.010 | 0.150 |
| `ucaf` upper arm inner, front | 0.380 | 0.100 | 0.480 |
| `ucar` upper arm inner, rear | 0.380 | −0.100 | 0.470 |
| `ubj` upper ball joint | 0.680 | 0.000 | 0.520 |
| `wc` wheel centre | 0.760 | 0.000 | 0.320 |
| `wco` wheel centre outboard | 0.860 | 0.000 | 0.320 |
| `tri` tie-rod inner | 0.330 | 0.060 | 0.300 |
| `tro` tie-rod outer | 0.700 | 0.040 | 0.310 |

Axes: `lca-axis` = `lcaf → lcar` direction, `uca-axis` = `ucaf → ucar` direction.
Lengths: `tie-rod` = |`tri − tro`| at design, `tyre-radius` = 0.20.

Bodies: `ground`, `lca` (lcaf, lcar, lbj), `uca` (ucaf, ucar, ubj), `up` (lbj, ubj, wc,
wco, tro). Joints: revolute `lca–ground` about `lca-axis`; revolute `uca–ground` about
`uca-axis`; spherical `lca–up` at `lbj`; spherical `uca–up` at `ubj`; distance `up–ground`
of `tie-rod`. Mobility = 1.

---

## 5. The solver

### Residuals — one function per joint kind

- **revolute** (body B rotates about body A on an axis through a point): 5 rows —
  3 for the two bodies sharing the axis point, 2 for the axis directions remaining
  parallel (two independent perpendicular components; state which two and why).
- **spherical**: 3 rows — the joined point in A's frame minus the same point in B's frame,
  in world coordinates.
- **distance**: 1 row — `|p_B − p_A| − length`.

`joints.py` returns rows as **expressions**, given symbolised unknowns and the geometry's
constants. It must not know about numpy.

### Derivation

`derive.py`: build the residual vector, `sympy.Matrix(...).jacobian(unknowns)`, then
`lambdify(..., cse=True, modules="numpy")` for both. **Never hand-differentiate** — the
Jacobian is where the bugs live and sympy cannot get it wrong. Cache the lambdified
function; report its build time if it is slow, do not optimise by hand.

### Solutions

- `newton(...)` — square system (unknowns with one driver fixed), a documented step limit
  (e.g. 60) and residual tolerance (e.g. `1e-12`, on the infinity norm). Returns the
  solution, iterations used, and the residual history. A non-convergence raises, naming
  the frame.
- `least_squares(...)` — `scipy.optimize.least_squares(..., method="lm")` for the
  redundant system. Returns the solution, `cost`, `optimality`, and the singular values
  of `J` at the solution (`scipy.linalg.svd(J, compute_uv=False)`).
- `svd_diagnose(...)` — returns the full singular values, a null-space basis **over
  constraint rows**, and **which row indices are dependent**. §3 of the book displays
  exactly this; do not compute it in JavaScript, and do not return something that a
  figure then has to interpret.
- `analytic.py` — closed-form circle-circle (2D) and circle-sphere (3D): the intersection
  of two circles/spheres in a plane, which is what a two-leg wishbone is. Used as the
  fast path and as the **independent cross-check** for the numeric solver. Independent
  means it does not call the symbolic derivation.

### Branch continuity — mandatory

Every solve **seeds from the previous frame**, and the sweep records any frame where the
selected branch changed (compare the solution with the analytic branch label, or the sign
of the discriminant in the closed form). A silent branch flip destroys a camber curve and
looks like a physics bug. If a branch change is detected, the frame records it — do not
raise, do not paper over it.

### Sweeping

`sweep.py` drives the DOF over `geometry.sweep`, one frame per step. Frame 0 is the design
position (`theta = 0`); the sweep runs from `start` to `stop` inclusive. Every frame
carries the driven angle, the solved positions, the outputs of §6, and the branch label.

---

## 6. Outputs — the quantities the book draws

`outputs.py`, all from the solved pose, in the front-view 2D projection (x, z) unless
stated. Signs matter as much as values; document each in the README and pin each with a
test:

- **camber**, degrees: positive when the **top of the wheel leans outward**, away from the
  car. From the wheel-axle unit vector `n` (design: `wc → wco`), `camber = degrees(asin(−n_z))`.
- **toe**, degrees: the signed change in the axle's plan-view heading relative to the
  design position, positive when the wheel steers **inboard at the front** (toe-in). A
  steered test pose with a known sign is what pins this.
- **instant centre**, `[x, z]` or `null`: front-view intersection of the line through
  `lbj` and the projected lower inner axis with the line through `ubj` and the projected
  upper inner axis. `null` when the two are parallel — do not emit a huge number.
- **roll centre**, `[0, z]` or `null`: where the line from the centreline contact patch
  (`x = 0`, `z = wc_z − tyre-radius`) to the instant centre crosses `x = 0`. `null` when
  the instant centre is `null` or on the centreline.
- **track change**, metres: lateral displacement of the contact patch from its design
  position (positive outboard).

---

## 7. The export contract

`export.py` emits **an ES module, not JSON**: static import, no `fetch`, no async, works
offline. Shape — this is a public contract because the book imports it, so keep the names
and units exactly:

```js
export const meta = {
  model: "dw-1dof",              // the model's own id
  units: { length: "m", angle: "deg" },
  branch: "upper",               // the selected assembly branch
  frames: 81,                    // frames.length
  travel: { start: -0.35, stop: 0.35, unit: "rad" },  // the driven coordinate
  generator: "suspension-kinematics 0.1.0",           // name and version
};

export const sweep = {
  input: { name: "theta", unit: "rad", values: [/* ... */] },
  frames: [
    {
      theta,                     // rad, driven coordinate
      // front view: what `wishbone-hero` draws
      lbj: [x, z], ubj: [x, z], wc: [x, z], tro: [x, z],
      // all three dimensions: what `wishbone-3d` draws
      points: { lcaf: [x, y, z], /* every named point */ },
      // the quantities
      camber,                    // deg
      toe,                       // deg
      ic: [x, z] | null,
      rc: [0, z] | null,
      track,                     // m, change from design
      branchChanged,             // bool, this frame only
    },
    // ...
  ],
};

export const iterations = [
  { k, residual: [/* every row of the residual at this iterate */],
       singular_values: [/* of J at this iterate */] },
  // ...
];

export const agreement = {
  analytic: [/* camber, one per frame, closed form */],
  numeric:  [/* camber, one per frame, numeric solver */],
  max_abs_diff: 1.2e-14,
};
```

Rules:

- **`iterations` is one representative frame** — mid-travel, the frame nearest `theta = 0`
  — because the book steps through it. State which frame in a comment in the emitted file.
  For the **two-leg** example, `iterations` is the least-squares iterate history and
  `residual` is the redundant residual, so the figure can show a floor rather than zero.
- **`agreement` is not decoration.** It is asserted in CI and it is read by the book's
  agreement panels; a panel never prints a number a human typed.
- **Rounding:** round to the precision the figure draws — 6 decimal places for lengths in
  metres, 4 for angles in degrees — *after* computing `max_abs_diff`, and round
  `max_abs_diff` itself to 3 significant figures. Otherwise rounding can dominate the
  number the reader is asked to trust.
- **Deterministic:** re-running the export reproduces the file byte for byte, including
  `generator` — so take the version from the installed package, not `datetime.now()`.
- The emitted module **imports nothing**. It is data.

CLI: `suspension export <model-dir> <out.js>`; `<model-dir>` holds `topology.yaml` and
`geometry.yaml`. Add `--quiet`. No other subcommand in v1 unless open question 7 says
otherwise.

---

## 8. Acceptance — v1 is done when all of this is true

`uv run pytest -q` passes, and it **includes these four tests**. They are the chapter's
argument, executed:

1. **analytic ≡ numeric.** Over the full travel range, the closed form and Newton agree to
   `|Δ| < 1e-9` on camber (and on the solved positions).
2. **least-squares ≡ direct.** With the redundant constraint removed, the least-squares
   solution equals the square direct solve, to `1e-9`.
3. **SVD identifies exactly one null direction** in the two-leg model, and dropping that
   row restores a well-conditioned system whose solution matches (2). Assert the count, not
   a threshold that hides a second one.
4. **stiffness → ∞ converges to rigid.** The compliant camber curve approaches the
   kinematic one **monotonically** as bushing stiffness increases; assert monotonicity
   across a stiffness ladder, not just the last value.

Also:

- `uv run suspension export examples/double-wishbone examples/double-wishbone/out.js`
  then `git add -A && git diff --cached --exit-code` reports no change (the emitted data is
  committed and stable). Repeat for all three examples.
- **The README states the limits honestly, in these words or better:** rigid bodies, small
  deformations, no friction, no tyre deflection, no dynamics. Plus the sign conventions of
  §6, the coordinate convention, and one sentence on what the package is *not* (a vehicle
  dynamics package, a tyre model, a simulator).
- CI (`.github/workflows/tests.yml`) runs `uv run pytest -q` on `ubuntu-latest` for Python
  3.12 and 3.13, and fails if a committed `out.js` is not reproducible. Keep it one job per
  interpreter, `uv sync --frozen`, `astral-sh/setup-uv`.
- No `maths-site` import, no path outside this repository, no network at solve time.

---

## 9. What the consumer will read (treat as frozen)

The book imports these three modules and nothing else. Renaming a key breaks a chapter:

| file | read by |
|---|---|
| `examples/double-wishbone/out.js` | `wishbone-hero`, `wishbone-3d`, agreement panel of §1 |
| `examples/double-wishbone-two-leg/out.js` | `solver-loop`, §2 and §3 agreement panels, the rank test |
| `examples/double-wishbone-compliant/out.js` | `stiffness-agreement`, §4 agreement panel |

Keys the book reads: `meta.*`, `sweep.input.values`, `sweep.frames[].{theta,lbj,ubj,wc,tro,
points,camber,toe,ic,rc,track,branchChanged}`, `iterations[].{k,residual,singular_values}`,
`agreement.{analytic,numeric,max_abs_diff}`. Everything else in the payload is yours.

If a key must change, say so in a commit message that names the book as the consumer, and
note it as a **breaking** change. The consumer's figures will be generated against this
contract, with a placeholder where a value is still missing.

---

## 10. Open questions for the author

Each one changes a public contract, so resolve it before it is built on.

1. **Distribution name.** Import name `suspension`, distribution `suspension-kinematics`.
   Confirm, or say what you prefer (the repository name and the import name need not
   match).
2. **Python floor.** `>=3.12` matches the consumer's engine; `>=3.11` reaches further.
   Which?
3. **Coordinate convention.** Default here is **x lateral outboard, y forward, z up**, for
   a right-front corner. ISO vehicle axes would be x forward, y left, z up — which means
   every sign in §6 flips. Confirm the default.
4. **Camber and toe signs** as written in §6 (positive camber = top of the wheel leans out;
   positive toe = toe-in). Confirm, or state the convention the book should teach.
5. **`Topology` carries no numbers**, so the sweep range lives in `Geometry`. Confirm — it
   is a deliberate departure from the earlier sketch.
6. **The Pyodide seam**: one comment in the code, or also a short `docs/pyodide.md` that
   records what the worker would have to do? No implementation either way.
7. **CLI surface for v1**: `export` only, or also `validate <model>` (checks names,
   mobility, and prints the residuals at the design position)? The consumer needs only
   `export`, but `validate` is what a human debugging a YAML wants.
8. **Frames and travel for the examples**: 81 frames over ±0.35 rad (~±20°) of lower-arm
   rotation — roughly ±40 mm of wheel travel. Confirm, or state the range the book's
   figures should span.
9. **Should the CLI also emit an SVG** of the linkage for eyeballing a geometry? Default
   is **no** — the book draws through its own kit, and a second renderer is a second truth.
   Confirm.
10. **Example geometry**: the table in §4 is a starting point, not a measured car. Say if
    you want specific numbers (a known kit-car or Formula Student corner) instead.

---

## 11. Verification, in one block

```bash
cd suspension-kinematics
uv sync
uv run pytest -q
for model in double-wishbone double-wishbone-two-leg double-wishbone-compliant; do
  uv run suspension export "examples/$model" "examples/$model/out.js"
done
git add -A && git diff --cached --exit-code    # emitted data is committed and stable
```

Then open a PR. In the description: the four acceptance tests and their numbers, the
mobility check, and which of the open questions above were resolved and how. If any of the
four cannot be made to pass, say so in the PR rather than weakening the test — a failing
invariant is information about the model, and the book is about exactly that.
