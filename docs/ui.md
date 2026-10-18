# The config UI

**Status:** design, not yet implemented, and post-v1 by construction. The engine it talks to
is specified in [`naming.md`](naming.md) §5; the model it edits is
[`config-layer.md`](config-layer.md).

`otkd ui <model>` opens a window that lets you configure a kinematics model and watch it
move. It is a **modeller over the four config files** — not CAD, not a simulator, and not a
second solver. The test of whether it is working is that you reach for it instead of a text
editor when you want to know what a number does, and reach for the text editor when you want
to know why.

## 1. Draft first, files second

```
load four files ──► draft (in memory) ──► solve on the draft ──► Save ──► four files
                            ▲                                            │
                            └────────────── edits ───────────────────────┘
```

- **Edits never write.** Dragging a point runs `check` and `solve` against the draft. Nothing
  touches the disk until you save; a drag is not sixty YAML rewrites a second, and a bad edit
  is undoable rather than a git checkout.
- **Save is an explicit act**, with a dirty marker and an undo stack. `Save` writes the
  files the draft came from; `Save as` scaffolds a new model directory (the four files, with
  the library path made explicit if the new directory is elsewhere).
- **Writes preserve the file.** Round-trip YAML — comments, key order, quoting, blank lines —
  so the diff of a drag is one number and a reviewer can see it. This is the one place the
  package adds a dependency (`ruamel.yaml`), isolated behind `ui/` and installed as an extra
  (`otkd[ui]`); the core never depends on it.
- **External edits** are detected by modification time and offered as a reload; a dirty draft
  is never silently replaced.
- **The draft is validated with the same code as the CLI.** Every check the UI shows is
  `check`'s report, not a re-implementation, and every solve is `solve`'s payload.

## 2. What it configures

| level | editing | the UI's job |
|---|---|---|
| structure | bodies, sites, joints, actuators, roles | pick a kind from **your** library and show its row count; offer only components that are free and not compliant when creating an actuator, because the library already says which those are |
| numbers | points, axes, lengths, gaps, compliances, the dynamics block | drag in the front view for `(x, z)`, type `y`; every number editable as a number |
| study | driver law, loads, `drop` | a slider over the driven coordinate; a load list; the sweep regenerates on change |

The structural editor is not a text box with a parser bolted on: it offers the choices the
config layer says are legal and refuses the rest, using the same validator. That is the whole
reason the layer exists, and the UI is where it pays off.

## 3. What it shows

- **The front view**, at the current frame, with the linkage, the wheel, and the contact
  patch — and animated across the sweep.
- **The curves**: camber, toe, track change and contact height against the driven coordinate,
  drawn from the payload.
- **Instant centre and roll centre** marked where they exist, and labelled *undefined* where
  they do not — never a huge number.
- **The check badge**: rank, mobility, redundancy, and the worst row by name when something
  is wrong. This is the feature a CAD tool does not have, and it is the book's argument made
  interactive: the model tells you *why* it is redundant, not just that it fails.
- **The derived residual**, on request: the rows the solver differentiates, in the same text
  and LaTeX forms `otkd compile` prints.

One rule holds the whole thing together: **the UI computes nothing.** Every number on screen
comes from `check` or `solve`; the UI turns numbers into SVG and nothing else. A second
computation path is a second truth, which is exactly what this package exists to avoid.

## 4. How it runs

`otkd ui <model>`:

1. loads the model with the same loader as every other verb;
2. starts a loopback HTTP server on an ephemeral port, serving the static front end and the
   engine endpoints;
3. opens the front end — a webview window if the optional launcher is installed, otherwise
   the default browser;
4. prints the URL, so it is usable over SSH without the window.

Endpoints mirror the engine boundary exactly: `GET /report`, `POST /solve`, `POST /save`
(draft → files), `GET /draft`, `POST /reload`. Nothing else. Bound to `127.0.0.1` only,
with no authentication, no remote mode, and no telemetry: the server exists for the length
of the session and dies with the command.

**No JavaScript build step.** The front end is committed HTML, CSS, JS and SVG, served
straight from the package. It matches the repository's position on generated artefacts — if
it cannot be reviewed in a diff, it does not ship — and it means the UI has no toolchain of
its own to go stale. If it ever outgrows hand-written JS, that is the signal to reconsider
the architecture, not to add a bundler quietly.

### Measured: what the front end can ask for

Numbers from this repository, on the built package, for one corner:

| operation | time | what it means for the UI |
|---|---|---|
| load the four files and compile | 10.3 s | once per session; a spinner, not a spinner per edit |
| re-resolve a geometry (new numbers) | ~2 ms | free |
| solve one frame, rigid model | 7 ms | drag a point and the linkage follows |
| solve one frame, compliant model | 340 ms | solve on release, or debounce; see below |
| evaluate the rows / the Jacobian | 0.4 / 0.8 ms | this is the budget dynamics lives in |
| full 81-frame sweep, rigid | ~2.3 s | too slow for every keystroke, fine for a curve |
| payload per frame | ~0.7 KB | a whole sweep is smaller than the JS bundle of a charting library |

So the interaction pattern is a **single-frame endpoint** while dragging and a **sweep
endpoint** when the drag ends. The compliant model's 340 ms is the one number worth fixing:
the equilibrium solve currently differentiates the projected potential numerically in 21
dimensions, and the free direction is one-dimensional, so a reduced solve along that
direction (a scalar root-find, each evaluation a warm-started rigid Newton) should land near
40 ms. That is an optimisation inside `solvers/equilibrium.py`, not a new architecture.

**Framework.** FastAPI behind the optional `otkd[ui]` extra, not in the core. The reasons are
not novelty: the request and response bodies *are* the pydantic config models, so validation
comes free and cannot drift from the CLI; server-sent events for "solve as you drag" are a few
lines; and an OpenAPI description makes the engine boundary self-documenting. A stdlib
`http.server` would be ~150 lines and no dependencies, so the honest alternative is real — the
deciding factor is streaming and schema, not speed.

**Why not run the engine in the browser (Pyodide).** The derivation is sympy. Shipping sympy and
mpmath into a page is several megabytes and turns a 10 s compile into something a reader will
not wait for, and every config change would re-derive. The browser is excellent at *consuming*
a payload — which is what the book already does — and that is the side of the boundary it
belongs on.

## 5. Where Rust fits

Three shapes, ordered by what they cost. Only the first is recommended.

**1. Window only (recommended).** A small PyO3 module — call it `otkd-ui` — exposing one
function:

```rust
#[pyfunction]
fn open(url: &str, title: &str) -> PyResult<()>   // a wry/tao webview window
```

`otkd ui` starts the server, then calls it if it is installed, and falls back to
`webbrowser.open` if not. That is the whole binding: no maths, no file access, no IPC, no
packaging of a Python runtime. It is exactly "expose the launch entry point to the Python
CLI" — and `otkd ui` still works on a machine with no Rust at all.

**2. Shell and sidecar.** A Tauri application that owns the window, the menus, the file
dialogs and the recent-models list, and spawns `otkd check --json` / `otkd solve --json` for
the engine. The front end is the same files. Cost: two artefacts to ship and keep in step,
and a decision about what happens when the sidecar is missing. Worth it only if the window
stops feeling like a browser.

**3. Engine in Rust.** Rejected for now. The derivation is sympy, and the book's claim is
that the residual and Jacobian are *derived rather than transcribed*. A Rust engine is a
legitimate future (a Pyodide mirror and a native app both want one) but it must reproduce
`emit`'s bytes on the conformance models before it may replace Python anywhere. Until then
it would be a second truth with better latency.

**Why not embed CPython in the shell (PyO3's embed mode)?** Because the engine needs sympy,
so you would ship an interpreter, pin a Python version, and lose crash isolation — to save
one subprocess call per edit. A compiler invoked as a subprocess over a JSON contract is the
cheaper and more debuggable seam, and it is the same seam the CLI already offers.

## 6. Staging

| stage | adds | earned by |
|---|---|---|
| v1 | nothing | the four acceptance tests |
| v1.x | `otkd ui`: draft/save, structure and numbers, front view, sweep, check badge | you can configure a corner faster than in a text editor |
| then | the optional window (`otkd-ui`) | the browser tab is annoying |
| maybe | the Tauri shell | the window is annoying |
| never | a forked engine, a JS solver, a renderer that recomputes | — |

## 7. Out of scope

3D shading and mesh import (the 3D points stay a wireframe at most), tyre models, contact
mechanics, multi-user editing, a hosted version. The UI edits a kinematic model and shows
what the engine says about it; anything else belongs to a different program.
