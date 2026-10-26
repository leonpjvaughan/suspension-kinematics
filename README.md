# otkd has moved

> **Archived, 2026-10-26.** This repository is read-only and kept as the record.
> Development continues in the collection it was built for:
> [**`Open.Textbook`**](https://github.com/leonpjvaughan/Open.Textbook), at
> [`solvers/otkd`](https://github.com/leonpjvaughan/Open.Textbook/tree/main/solvers/otkd).
> The history below this commit stays here and is not carried across.

`otkd` compiles a mechanism description — `topology.yaml`, `geometry.yaml`,
`study.yaml`, and a `constraints.yaml` of your own — into a mathematical problem:
a symbolic residual and Jacobian, then a solver or an emitter. It serves the
`maths-applied` part of the collection, whose numbers it produces as static ES
modules and WebAssembly kernels.

## Where things are now

| | |
|---|---|
| The package | [`solvers/otkd`](https://github.com/leonpjvaughan/Open.Textbook/tree/main/solvers/otkd) — imports as `open_textbook.otkd`, the command is still `otkd` |
| The ABI the kernels speak | [`solvers/ABI.md`](https://github.com/leonpjvaughan/Open.Textbook/blob/main/solvers/ABI.md) |
| The hand-written kernels | [`solvers/ot-solve`](https://github.com/leonpjvaughan/Open.Textbook/tree/main/solvers/ot-solve) |
| The book that uses it | [`books/maths-applied`](https://github.com/leonpjvaughan/Open.Textbook/tree/main/books/maths-applied) |
| The design record | [`DECISIONS.md`](https://github.com/leonpjvaughan/Open.Textbook/blob/main/DECISIONS.md), entry of 2026-10-26 |

## Version

There is no separate version any more. The package reports the **collection's
release tag** — for example `26.299.0` — read from the repository tag by
`hatch-vcs`, so the package, the bundle and the site always agree. Nothing is
published to PyPI, and there are no tags or releases in this repository after
`0.11.0`.

## Using it

```bash
git clone https://github.com/leonpjvaughan/Open.Textbook.git
cd Open.Textbook/solvers/otkd
uv sync
uv run otkd --help
```

The bundle the site vendors — `out.js`, `kernel.wasm` and the manifest for each
model — is committed, and `scripts/build-otkd.sh` in the collection rebuilds it
from the source here.

## Licence

MIT, as it was here. See [`LICENSE`](LICENSE).
