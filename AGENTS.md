# AGENTS.md — repository instructions

This repository is **otkd**, the engine behind a textbook about modelling: config files in,
mathematical problems out. It reads a mechanism's topology, geometry and study, derives the
residual and its Jacobian symbolically, and hands them to a solver or an emitter. A suspension
corner is the first subject; nothing in the config layer knows what a wishbone is.

The code is small and the prose is not. The documents are the design record — read them before
changing a contract, and update them in the same commit as the code.

## Where to look

| If you are… | Read |
|---|---|
| Starting work | this file, then [`docs/README.md`](docs/README.md) |
| Touching the config layer or validation | [`docs/config-layer.md`](docs/config-layer.md), [`examples/constraints.yaml`](examples/constraints.yaml) |
| Touching the derivation, solvers or diagnostics | [`docs/problem-mapping.md`](docs/problem-mapping.md) |
| Adding or changing an example | [`docs/config-layer.md`](docs/config-layer.md) §10, [`docs/problem-mapping.md`](docs/problem-mapping.md) §8 |
| Changing the CLI, the API or the errors | [`docs/entry-points.md`](docs/entry-points.md), [`docs/naming.md`](docs/naming.md) |
| Touching the compiled kernel | [`docs/wasm.md`](docs/wasm.md) |
| Releasing, branching, versioning | [`docs/releasing.md`](docs/releasing.md) |
| Wondering why something is the way it is | [`docs/decisions.md`](docs/decisions.md) — every choice, including the ones implementation changed |
| Picking up unfinished work | [`docs/follow-ups.md`](docs/follow-ups.md) |
| Dynamics | [`docs/dynamics.md`](docs/dynamics.md) |
| The UI, if it is ever built | [`docs/ui.md`](docs/ui.md) |
| The frozen specification from the author | [`HANDOFF.md`](HANDOFF.md) |

## Ground rules

- **`dev` is where work happens; `main` is release-only.** Every merge to `main` is a release
  attempt. Conventional Commits decide the version; the tag is the version; `hatch-vcs` serves
  it. No `v` prefix. Nothing is published to PyPI.
- **`uv` + `hatchling` + `src` layout, `requires-python >= 3.12`.** Add a dependency only with a
  reason in `pyproject.toml`.
- **No hand-differentiation, anywhere.** The residual and the Jacobian come from sympy. If a
  derivative is needed at solve time, differentiate the symbolic expression numerically rather
  than writing it down.
- **Emitted artefacts are committed and reproducible.** `out.js` and the kernel sources must be
  byte-identical on regeneration; the `.wasm` is a toolchain artefact and is tested by
  behaviour. Never put a value in an artefact that changes without a release (a git-derived
  version, a timestamp).
- **Type hints everywhere; pydantic for anything a user writes in YAML.** A configuration error
  is a list of faults naming file, key and expectation — never a silent default.
- **Comments say why.** Match the tone of the documents: calm, specific, no exclamation marks.
- **Change a contract and you change `HANDOFF.md`'s consequences**, not the file. Record the
  divergence in `docs/decisions.md` with the reason.

## Solver findings are teaching material

This package exists to be explained, so a finding is not finished when the code is fixed — it is
finished when the textbook can teach it. The textbook is `maths-site`, and what it asks of the
package is [`docs/handoff-from-maths-site.md`](docs/handoff-from-maths-site.md).

- Record a finding in `docs/decisions.md` as usual, **and** say in the handoff whether the book
  could use it: a figure, a worked step, a "here is how you would know" box.
- The ones worth the most have a **tell** — a measurement that separates two explanations. The
  energy error that did not shrink when the step was quartered. The residual that did not move
  when the load changed by three orders of magnitude. The iteration trace that differed between
  two machines while every frame was identical. A wrong number with a reproducible reason teaches
  more than a right one.
- Name the insight, the measurement and where it came from. Do not write the chapter's prose here;
  the book decides how to teach it.

## Commands

```console
$ uv sync                                   # install (builds the scm version file)
$ uv run pytest -q                          # the whole suite, ~2 minutes
$ uv run otkd check examples/double-wishbone
$ uv run otkd solve examples/double-wishbone
$ uv run otkd emit examples/double-wishbone examples/double-wishbone/out.js
$ uv run otkd emit <model> <dir> --format wasm    # generated Rust kernel
$ uv run python scripts/build_release.py --version <v> --out dist --package
```

`hatch-vcs` writes `src/otkd/_version.py` from the tag. A `uv build` with
`SETUPTOOLS_SCM_PRETEND_VERSION` writes that version into it, so re-sync afterwards.

## End every run with this

Three short sections, in this order. No preamble, no restating the request.

1. **A table** — what changed or was verified in this run: one row per item, with the evidence
   (a test count, a number, a command result). If nothing changed, say so in one row.
2. **Issues** — bullet points, each one line: the problem, then the proposed solution. Anything
   that needs the author's decision is marked **decision needed** and says what the options are.
   Do not bury a failure here; a failing check is the first bullet.
3. **Remaining tasks** — bullet points, the concrete next actions in order, not a wish list. Each
   should be small enough to start without another conversation.

Keep all three short. If a section would be long, the run did too much and the detail belongs in
a document instead.
