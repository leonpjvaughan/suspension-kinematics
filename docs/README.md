# Design record

[`HANDOFF.md`](../HANDOFF.md) is the specification: what the package is for, the four
acceptance tests, the export contract the book reads. It is a contract, and a contract
should not be quietly edited. It also predates the rename and the module layout: where it
names the package, read `otkd` ([`naming.md`](naming.md) §2); where it lays out modules, read
§3.

These documents are the design that satisfies it. Where one of them disagrees with a handoff
sketch, it says so and gives the reason; [`decisions.md`](decisions.md) records the answer.

| document | answers |
|---|---|
| [`config-layer.md`](config-layer.md) | what a model *is*: bodies, sites, components, modes, constraint kinds, joints, actuators, geometry, study, and the rules that make a bad model fail at load |
| [`problem-mapping.md`](problem-mapping.md) | what is *built* from a model: unknowns, rows, the Jacobian, how the chapters' problems are selected, and the three solvers |
| [`dynamics.md`](dynamics.md) | what a dynamics study would need, and precisely which parts of the above it reuses unchanged |
| [`entry-points.md`](entry-points.md) | what a user supplies, the CLI, the Python API, and how a bad file fails |
| [`releasing.md`](releasing.md) | branches, Conventional Commits, how the version is decided, and what a push to `main` does |
| [`wasm.md`](wasm.md) | the compiled kernel: what a generated kernel is mathematically, how sympy becomes Rust becomes WebAssembly, and how the book consumes it |
| [`naming.md`](naming.md) | the name, the module layout, the verbs, and the engine boundary every front end speaks |
| [`ui.md`](ui.md) | the config UI: what it edits, what it shows, and where Rust fits (a window, not an engine) |
| [`follow-ups.md`](follow-ups.md) | work that is agreed and deliberately not done yet, with what would make each item done |
| [`decisions.md`](decisions.md) | every decision, its resolution, and the reasoning behind the ones that needed it |

Suggested reading order for someone picking the work up: `config-layer.md` §1–§3 (the split
and the component model), then `../examples/constraints.yaml` (the whole vocabulary, in one
page), then `problem-mapping.md` §8 (the row tables — that is the whole book in numbers),
then `entry-points.md` if you are going to run it, then `decisions.md`.

The one-sentence summary, if you only read one: **three files and a library describe a
mechanism; a joint is a set of components, each locked, free or compliant; a constraint kind
is a named bundle of those; an actuator prescribes one free component; and the solver is
chosen from the rank of the symbolic Jacobian, never from a flag.**
