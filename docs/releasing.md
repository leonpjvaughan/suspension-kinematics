# Releasing

**Status:** implemented. `.github/workflows/release.yml`, `scripts/version.py`,
`scripts/build_release.py`.

## Branches

- **`dev`** is where work happens.
- **`main`** is release-only: it receives merges from `dev`, and nothing else.
- **A push to main is a release attempt.** The version becomes a tag, created by CI; nobody
  writes a version by hand and there is no `v` prefix: the tag is `0.2.0`.

## Where the version comes from

Two separate questions, answered by two different things:

1. **Which number?** [Conventional Commits](https://www.conventionalcommits.org/) since the last
   tag, in [PEP 440](https://peps.python.org/pep-0440/) form:

   | commits since the last tag | bump |
   |---|---|
   | `!` after the type, or a `BREAKING CHANGE:` footer | **major** — while `0.x`, **minor** |
   | any `feat:` | **minor** |
   | any `fix:` or `perf:` | **patch** |
   | only `docs:`, `chore:`, `test:`, `ci:`, `build:`, `style:`, `refactor:` | none — no release |

   The 0.x rule is deliberate: before 1.0 every change may break, so a breaking change moves the
   minor rather than claiming an API stability nobody promised.

2. **What number is it?** The **tag**. `hatch-vcs` reads the tag and serves `otkd.__version__`
   from it, so the package, the wheel, the manifest and `otkd --version` all agree without a
   single line of version text in the repository.

Between releases `otkd --version` reports a development version — `0.1.0.dev3+gabc1234` — which
is exactly what scm is for. A released build reports the tag.

## What a push to main does

1. **Decide** the next version from the commits since the last tag
   (`uv run python scripts/version.py next`). If the commits do not warrant a release, stop: the
   flow is idempotent, so pushing to main twice releases once.
2. **Test** (`uv run pytest -q`).
3. **Adopt** the version: `SETUPTOOLS_SCM_PRETEND_VERSION=<V> uv sync --reinstall-package otkd`,
   so the installed package reports the version before the tag exists.
4. **Build the bundle**: the wheel and sdist, every example's `out.js` payload, and every
   compiled kernel (`uv run python scripts/build_release.py --version <V> --out dist --package`).
5. **Rebuild and compare** the zip, byte for byte. The wheel is reproducible because
   `SOURCE_DATE_EPOCH` is pinned, the kernels because rustc is pinned, the payloads because they
   are deterministic.
6. **Tag** `<V>` and push the tag — only now, when the artefacts exist.
7. **Publish**: `gh release create <V> dist/otkd-<V>.zip dist/otkd-<V>.zip.sha256`.

The tag is last on purpose: a failed test or a failed build leaves no tag behind, so the next
push retries cleanly instead of skipping the release it never produced.

## What is in the zip

```
otkd-<V>/
  MANIFEST.json     generator, git commit, python and rustc versions, per-model status
  SHA256SUMS        every file's digest
  package/          otkd-<V>-py3-none-any.whl and otkd-<V>.tar.gz
  models/<name>/out.js                 the static payload
  models/<name>/kernel/kernel.js       the compiled solver (self-contained)
  models/<name>/kernel/kernel.json     the ABI
  models/<name>/kernel/kernel.rs       generated source (provenance)
  models/<name>/kernel/kernel.wasm     the module (provenance; inlined in kernel.js)
  models/<name>/kernel/provenance.json input hashes, toolchain
  models/<name>/kernel/source/*.yaml   the four config files
```

A model without a compiled solver still ships its payload, and the manifest says why the kernel
is missing rather than leaving a hole.

## Why the version is not in the payloads

`meta.generator` is `"otkd"`, not `"otkd 0.2.0"`. A scm version changes with every commit, so a
version inside every `out.js` would dirty four committed artefacts after each one and break the
byte-identical regeneration rule. The payload is data; the version belongs to the release, where
`MANIFEST.json` and `provenance.json` record it.

This is a deliberate departure from `HANDOFF.md` §7's example value, on the same grounds as the
rename: the key is unchanged, the value is the generator's name.

## Between releases, and by hand

- `otkd --version` reports the scm version, which is a development version until the tag exists.
- A `uv build` with `SETUPTOOLS_SCM_PRETEND_VERSION` writes that version into
  `src/otkd/_version.py`, so re-run `uv sync` afterwards or the source tree and the installed
  metadata will disagree until the next install. `scripts/build_release.py` refuses to label a
  bundle with a version the installed package does not report, which catches it.
- There is no hand-tagging path: the workflow triggers on a push to `main`. If CI fails after the
  tag is created (it should not — the tag is last), delete the tag and push a fix; the next push
  computes the same version again.
- Branch protection must let the workflow push a tag. It does not push commits, so it does not
  need to bypass protection on `main`. Tags may be created but not moved or deleted
  ([`security.md`](security.md)).
- **After each release, reset `dev` to `main`.** `main` is squash-only, so the release commit is a
  new commit that `dev` does not have; the content is identical, so
  `git fetch && git reset --hard origin/main` is the whole reconciliation. Do this before the
  next piece of work, or the next pull request carries a spurious merge.
- Dependabot opens its pull requests against `dev`, so a dependency bump takes the same path as
  any other change.

## What a consumer pins

The textbook repository pins **the version and the digest**:

```
OTKD_VERSION=0.2.0
OTKD_SHA256=<from the release>
```

and verifies before unpacking. `docs/wasm.md` §6 has the step.
