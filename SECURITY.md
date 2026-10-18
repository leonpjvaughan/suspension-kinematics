# Security policy

This repository builds a numerical engine and its generated artefacts. It has no server, no
network access at solve time, and no secrets of its own.

## Reporting a vulnerability

Use GitHub's private reporting: **Security** → **Advisories** → **Report a vulnerability**.
That opens a private draft advisory visible only to the maintainers. Please do not open a public
issue or pull request for a security problem.

Include what you can: the model files or a minimal reproduction, the version (`otkd --version`),
and what an attacker gains. A crash on malformed input *is* in scope — the loader is supposed to
refuse rather than repair.

## What is in scope

- The loader and validator (`src/otkd/config/`): parsing untrusted YAML, path resolution for the
  constraint library, and the guarantee that a bad file produces a fault rather than a solve.
- The symbolic derivation and solvers (`src/otkd/problem/`, `src/otkd/solvers/`).
- The emitters (`src/otkd/emitters/`), including the generated Rust: a model file must not be
  able to emit code that escapes the fixed-size buffer contract.
- The command line and the release workflow.

## What is not

- Numerical results: a model that converges to an unexpected pose is a modelling question, not a
  vulnerability. This repository has no issue tracker by design; use the advisory form only for
  security, and the book's repository for questions about the mathematics.
- Dependencies: report those through GitHub's advisory database; Dependabot is enabled.
- The example geometry: it is illustrative and not a real car.
