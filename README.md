# suspension-kinematics

Configurable multibody kinematics for suspension geometry.

A suspension linkage is described by two separate things: its **topology** (which bodies,
joined by which joints) and its **geometry** (every number). One topology can be swept
over many geometries, so a design study is a loop over numbers rather than a rewrite of
constraints. The residual and its Jacobian are derived symbolically, so the solver cannot
disagree with the model it claims to solve.

It exists to serve a book about *modelling*: a chapter that starts with one rigid triangle,
finds the model redundant when the wishbone gains a second leg, diagnoses the redundancy
with an SVD, and then replaces a rigid constraint with a spring to show the two models
meeting. The numbers those chapters draw are produced here, as a static ES module, so the
pages need no runtime solver.

## Status

**Not yet implemented.** [`HANDOFF.md`](HANDOFF.md) is the specification: the config
split, the solver, the export contract, the four acceptance tests, and the open questions
that need an answer before the contract is fixed.

## Limits, stated up front

The model is a kinematic one. It is rigid bodies and small deformations: no friction, no
tyre deflection, no dynamics, no compliance that is not a linear bush. It answers
questions about where the linkage *is*, and about how a chosen model earns its place —
not about how a car behaves. That is what it is for, and it is not an apology.

## Licence

MIT. See [`LICENSE`](LICENSE).
