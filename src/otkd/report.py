"""`check`: does this model mean anything?

Two questions, both of which need the symbolic layer. Does the design position satisfy every
equation the model claims? And is the mechanism's mobility exactly what the actuators
prescribe? The second is a rank, not a count, because a redundant model has more rows than
independent ones and counting cannot tell the difference (see docs/decisions.md §4).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .config.model import Model
from .errors import ConfigError, Fault
from .problem.compile import HARD, NORM, SPRING, Problem, Row
from .solvers.system import rank_and_null

DESIGN_TOL = 1e-9
RANK_RTOL = 1e-10


@dataclass
class Report:
    model: Model
    problem: Problem
    rows: list[tuple[Row, float]]
    singular_values: np.ndarray
    rank: int
    redundancy: int
    mobility: int
    actuators: int
    warnings: tuple[str, ...] = ()
    free_directions: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return True

    def worst(self, count: int = 3) -> list[tuple[Row, float]]:
        return sorted(self.rows, key=lambda item: abs(item[1]), reverse=True)[:count]

    def summary(self) -> str:
        lines = [
            f"{self.model.topology.id}",
            f"  constraints   {self.model.files.get('constraints', '')} ({len(self.model.kinds)} kinds)",
            f"  bodies        {len(self.model.topology.bodies)} "
            f"({self.problem.n // 7} moving; fixed: {', '.join(sorted(self.problem.fixed))})",
            "  rows",
        ]
        for row, value in self.rows:
            lines.append(f"    {row.name:24s} {row.group:8s} {value:+.3e} {row.unit or '-':3s}")
        dynamics = self.model.study is not None and self.model.study.analysis == "dynamics"
        lines += [
            f"  jacobian      {self.problem.n} unknowns, {len(self.rows)} rows",
            f"  rank          {self.rank}   redundancy {self.redundancy}",
            (
                f"  freedom       {self.mobility}   dynamics   ok"
                if dynamics
                else f"  mobility      {self.mobility}   actuators {self.actuators}   ok"
            ),
        ]
        for warning in self.warnings:
            lines.append(f"  warning       {warning}")
        return "\n".join(lines)


def check(model: Model) -> Report:
    problem = Problem(model)
    g = problem.numbers()
    q0 = problem.q0()
    theta0 = problem.theta0()
    values = problem.evaluate(q0, g, theta0)
    rows = [(row, float(values[i])) for i, row in enumerate(problem.rows)]

    faults: list[Fault] = []
    g_file = model.files.get("geometry", "geometry.yaml")
    for row, value in rows:
        if abs(value) > DESIGN_TOL:
            faults.append(
                Fault(
                    g_file,
                    row.name,
                    f"the design position leaves this row at {value:+.3e} {row.unit or ''}; "
                    f"the model is not assembled",
                )
            )

    jac = problem.jacobian(q0, g, theta0)
    # In kinematics a compliant component is still an equation — a different one, but it closes
    # the same degree of freedom — so it counts towards the mobility. In dynamics a spring is a
    # *force*: the constraint set is the hard rows and the normalisation, and the freedom it
    # leaves is the motion the integrator will follow.
    dynamics = model.study is not None and model.study.analysis == "dynamics"
    constraints = [
        i
        for i, row in enumerate(problem.rows)
        if row.group in (HARD, NORM) or (not dynamics and row.group == SPRING)
    ]
    rank, null, singular = rank_and_null(jac[constraints], rtol=RANK_RTOL)
    mobility = problem.n - rank
    actuators = sum(1 for actuator in model.topology.actuators if actuator.effort == "position")
    if dynamics:
        redundant_rows = constraints
    else:
        redundant_rows = [i for i, row in enumerate(problem.rows) if row.group != "spring"]
    redundancy = len(redundant_rows) - int(
        (np.linalg.svd(jac[redundant_rows], compute_uv=False) > RANK_RTOL).sum()
    )

    # A dynamics study has no driven coordinates: the mechanism is *supposed* to have freedom,
    # and the question is whether the constraint set is independent (a redundant row makes the
    # DAE singular) rather than whether the freedom equals the actuators.
    free: list[str] = []
    if dynamics:
        if redundancy:
            faults.append(
                Fault(
                    model.files.get("topology", "topology.yaml"),
                    "joints",
                    f"a dynamics model must not be redundant: {redundancy} dependent row(s) make "
                    f"the equations of motion singular",
                )
            )
    elif mobility > actuators:
        for column in null.T:
            free.append(_describe_direction(problem, column))
        faults.append(
            Fault(
                model.files.get("topology", "topology.yaml"),
                "actuators",
                f"mobility is {mobility} but {actuators} actuator(s) prescribe it; unconstrained: "
                + "; ".join(free),
            )
        )
    elif mobility < actuators:
        faults.append(
            Fault(
                model.files.get("topology", "topology.yaml"),
                "actuators",
                f"mobility is {mobility} but {actuators} actuator(s) prescribe it: the model is over-driven",
            )
        )

    if faults:
        raise ConfigError(faults)
    return Report(
        model=model,
        problem=problem,
        rows=rows,
        singular_values=singular,
        rank=rank,
        redundancy=redundancy,
        mobility=mobility,
        actuators=actuators,
        warnings=problem.resolved.warnings,
    )


def _describe_direction(problem: Problem, column: np.ndarray) -> str:
    """A human sentence for a direction in unknown space, e.g. `up can translate in y`."""
    best = int(np.argmax(np.abs(column)))
    magnitude = abs(column[best])
    body = problem.moving[best // 7]
    local = best % 7
    kind = ("translate", "rotate")[local >= 3]
    return f"{body} can {kind} along ({', '.join(f'{c:+.2f}' for c in column[best // 7 * 7 : best // 7 * 7 + 3])})"


def diagnose_redundancy(model: Model):
    """Convenience for the CLI and tests: the SVD diagnosis at the design position."""
    from .solvers.svd import diagnose
    from .solvers.system import System

    problem = Problem(model)
    system = System(problem, problem.numbers(), problem.theta0())
    return diagnose(system, problem.q0())
