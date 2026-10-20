"""Drive the study's actuators and tabulate the mechanism.

Frame 0 is seeded from the design position; every later frame is seeded from its predecessor,
which is what branch continuity needs. A branch change is recorded, never raised.
"""

from __future__ import annotations

import difflib
import itertools
from dataclasses import dataclass, field

import numpy as np

from ..analysis.outputs import Outputs, Reference, compute, make_reference
from ..config.model import Model
from ..errors import ConfigError, Fault, SolverError
from ..problem.compile import Problem
from .equilibrium import equilibrium
from .least_squares import least_squares
from .newton import newton
from .system import System

#: A frame that moves further than this from its seed is treated as a different assembly.
_JUMP_ANGLE = 0.25  # radians
_JUMP_DISTANCE = 0.05  # metres


@dataclass
class Frame:
    index: int
    values: dict[str, float]
    q: np.ndarray
    points: dict[str, np.ndarray]
    outputs: Outputs
    branch_changed: bool
    history: list[tuple[np.ndarray, np.ndarray]]
    singular_values: np.ndarray
    #: One value per study axis, in the order the axes are declared. Empty for a plain sweep.
    axis: tuple[float, ...] = ()


@dataclass
class Sweep:
    model: Model
    problem: Problem
    reference: Reference
    frames: list[Frame]
    primary: str
    unit: str
    kind: str
    #: The geometry numbers each axis swept, and the values it took — for a payload that has to
    #: say what a frame's `axis` entry means.
    axis_numbers: list[list[str]] = field(default_factory=list)
    axis_values: list[list[float]] = field(default_factory=list)
    analytic: list[float] | None = None
    agreement: float | None = None
    branch: str = "design"

    @property
    def theta(self) -> np.ndarray:
        return np.array([frame.values[self.primary] for frame in self.frames])


def sweep(model: Model) -> Sweep:
    if model.study is None:
        raise ConfigError([Fault(model.files.get("study", "study.yaml"), "", "no study")])
    study = model.study
    if not study.drivers:
        raise ConfigError([Fault(model.files.get("study", "study.yaml"), "drivers", "a kinematics study needs at least one driver")])

    problem = Problem(model)
    base = problem.numbers()
    reference = make_reference(problem)
    loads = _load_case(problem, study)

    grids = [driver.law.grid() for driver in study.drivers]
    lengths = {len(grid) for grid in grids}
    if len(lengths) != 1:
        raise ConfigError(
            [
                Fault(
                    model.files.get("study", "study.yaml"),
                    "drivers",
                    f"drivers have different frame counts: {[len(g) for g in grids]}",
                )
            ]
        )
    frames = len(grids[0])

    numbers, indices, axis_grids = _axes(problem, study, model)
    drop = tuple(study.drop)
    seed = problem.q0()
    out: list[Frame] = []
    kind = "square"
    for combination in itertools.product(*axis_grids):
        g = _with_axes(base, indices, combination)
        # A different stiffness is a different assembly, so each pass starts from the design pose
        # rather than from the last frame of the pass before it. Classification is per pass too:
        # a compliance can be swept to zero, and zero is a rigid row.
        previous = seed
        first = System(
            problem,
            g,
            problem.theta_for({d.actuator: grids[i][0] for i, d in enumerate(study.drivers)}),
            drop,
        )
        kind = first.classify()
        if kind == "under":
            raise SolverError("the model is under-determined: fewer independent constraints than unknowns")

        for index in range(frames):
            values = {driver.actuator: float(grids[i][index]) for i, driver in enumerate(study.drivers)}
            theta = problem.theta_for(values)
            system = System(problem, g, theta, drop)
            if index == 0 or study.seed == "design":
                guess = seed
            else:
                guess = previous

            if kind == "equilibrium":
                if loads:
                    try:
                        q, _, history = equilibrium(system, guess, loads)
                    except SolverError:
                        # The predecessor is the right seed for branch continuity and the wrong one
                        # for a joint that has just become very stiff. The design pose satisfies the
                        # constraints by construction, so it is worth one more attempt before the
                        # frame is called unsolvable.
                        if np.array_equal(guess, seed):
                            raise
                        q, _, history = equilibrium(system, seed, loads)
                else:
                    # No load, no preload: the equilibrium deflection is zero, so the compliant
                    # model is the rigid one and solving it as a root-find would be theatre.
                    q, _, history = newton(system, guess, rows=list(system.rows))
            elif kind == "redundant":
                q, _, history, _ = least_squares(system, guess)
            else:
                q, _, history = newton(system, guess)

            singular = np.linalg.svd(system.jacobian(q), compute_uv=False)
            changed = index > 0 and _jumped(problem, q, previous)
            outputs = compute(problem, q, reference)
            out.append(
                Frame(
                    index=len(out),
                    values=values,
                    axis=tuple(float(value) for value in combination),
                    q=q,
                    points=problem.world_points(q),
                    outputs=outputs,
                    branch_changed=changed,
                    history=history,
                    singular_values=singular,
                )
            )
            previous = q

    primary = study.drivers[0].actuator
    unit = problem.resolved.actuators[primary].unit
    result = Sweep(
        model=model,
        problem=problem,
        reference=reference,
        frames=out,
        primary=primary,
        unit=unit,
        kind=kind,
        axis_numbers=numbers,
        axis_values=[[float(value) for value in grid] for grid in axis_grids],
    )
    _attach_analytic(result)
    return result


def _axes(problem: Problem, study, model: Model) -> tuple[list[list[str]], list[list[int]], list[np.ndarray]]:
    """The study's axes, resolved to geometry-number indices.

    An unknown name is a configuration fault that names the file, the axis and the nearest number
    that does exist; the alternative is a `KeyError` from inside the compiler.
    """
    names: list[list[str]] = []
    indices: list[list[int]] = []
    grids: list[np.ndarray] = []
    known = problem.number_names
    for position, axis in enumerate(study.axes):
        resolved = []
        for name in axis.numbers:
            try:
                resolved.append(problem.number_index(name))
            except KeyError:
                close = difflib.get_close_matches(name, known, n=1)
                hint = f"; did you mean {close[0]}?" if close else ""
                raise ConfigError(
                    [
                        Fault(
                            model.files.get("study", "study.yaml"),
                            f"axes[{position}].numbers",
                            f"no such geometry number: {name}{hint}",
                        )
                    ]
                ) from None
        names.append(list(axis.numbers))
        indices.append(resolved)
        grids.append(axis.law.grid())
    return names, indices, grids


def _with_axes(base: np.ndarray, indices: list[list[int]], values: tuple[float, ...]) -> np.ndarray:
    g = base.copy()
    for axis_indices, value in zip(indices, values):
        for i in axis_indices:
            g[i] = value
    return g


def _attach_analytic(result: Sweep) -> None:
    """The independent closed form, where the model has one. Never raises: a model without
    the wishbone structure simply has no agreement panel."""
    from ..analysis import analytic

    # The closed form is a rigid, angle-driven statement. A compliant or length-driven model
    # has no directly comparable closed form, and pretending otherwise would put a number on
    # an agreement panel that means nothing.
    if result.problem.has_springs or result.unit != "rad":
        return
    wishbone = analytic.extract(result.problem)
    if wishbone is None:
        return
    thetas = result.theta
    result.analytic = analytic.camber_curve(wishbone, thetas)
    result.agreement = max(
        abs(a - frame.outputs.camber) for a, frame in zip(result.analytic, result.frames)
    )


def _load_case(problem: Problem, study) -> list[tuple[str, str, np.ndarray]]:
    out = []
    for load in study.loads:
        site = problem.topology.sites[load.site]
        body = problem.resolved.point_body[site.origin]
        out.append((body, site.origin, np.asarray(load.force, dtype=float)))
    return out


def _jumped(problem: Problem, q: np.ndarray, seed: np.ndarray) -> bool:
    a = problem.poses(q)
    b = problem.poses(seed)
    for body in a:
        delta = a[body][1].T @ b[body][1]
        angle = float(np.arccos(np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0)))
        if angle > _JUMP_ANGLE or np.linalg.norm(a[body][0] - b[body][0]) > _JUMP_DISTANCE:
            return True
    return False
