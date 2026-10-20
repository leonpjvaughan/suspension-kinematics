"""Config -> mathematics.

The compiled `Problem` contains the topology, not the numbers: every geometry value is a
symbol supplied at solve time, so one `compile` serves a design study. See
docs/problem-mapping.md.

Rotation is a unit quaternion per body, `(q0, q1, q2, q3)` scalar first, plus one
normalisation row per moving body. The reason is not taste: the exponential map's derivative
is 0/0 at zero rotation, which is exactly the design position every model is assembled and
validated at, whereas every expression here is polynomial. Rows are written in terms of the
rotation matrix, so a future change of parametrisation stays local to this file.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from typing import Callable

import numpy as np
import sympy as sp

from ..config.kinds import ROTATION, TRANSLATION
from ..config.model import Model, Resolved, ResolvedJoint

HARD, DRIVER, NORM, SPRING = "hard", "driver", "norm", "spring"

#: The symbolic build depends on the topology and the library, never on the numbers, so it is
#: cached across models that share them. A design study then compiles once and solves many
#: geometries, which is the promise the config split makes.
_CACHE: dict[str, dict] = {}


@dataclass(frozen=True)
class Row:
    name: str
    unit: str
    group: str


@dataclass(frozen=True)
class Drive:
    actuator: str
    joint: str
    dof: str
    unit: str
    theta: sp.Symbol
    row: str


class Problem:
    """The symbolic problem for one (topology, library). Geometry is an argument."""

    def __init__(self, model: Model):
        self.model = model
        self.topology = model.topology
        self.resolved: Resolved = model.resolve()
        key = _key(model)
        cached = _CACHE.get(key)
        if cached is None:
            self._build()
            _CACHE[key] = {k: v for k, v in self.__dict__.items() if k not in ("model", "resolved")}
        else:
            self.__dict__.update(cached)
            self.model = model
            self.resolved = model.resolve()

    # -- construction ------------------------------------------------------------------

    def _build(self) -> None:
        topo, res = self.topology, self.resolved
        self.fixed = {body for body, is_fixed in res.bodies.items() if is_fixed}
        self.moving = [body for body in topo.bodies if body not in self.fixed]

        self.q: list[sp.Symbol] = []
        self._qstart: dict[str, int] = {}
        self._pose: dict[str, tuple[sp.Matrix, sp.Matrix]] = {}
        for body in topo.bodies:
            if body in self.fixed:
                self._pose[body] = (sp.zeros(3, 1), sp.eye(3))
                continue
            self._qstart[body] = len(self.q)
            syms = sp.symbols(f"q_{body}_0:7", real=True)
            self.q.extend(syms)
            px, py, pz, w, x, y, z = syms
            self._pose[body] = (sp.Matrix([px, py, pz]), _rotation(w, x, y, z))
        self.n = len(self.q)

        self.g: list[sp.Symbol] = []
        self._gkey: list[tuple] = []
        self._point_sym: dict[str, sp.Matrix] = {}
        self._site_c: dict[str, sp.Matrix] = {}
        self._allocate_geometry()

        self.rows: list[Row] = []
        self._exprs: list[sp.Expr] = []
        self._add_joint_rows()
        self._add_drivers()
        self._add_norm_rows()

        self.row_index = {row.name: i for i, row in enumerate(self.rows)}
        self.hard = [i for i, row in enumerate(self.rows) if row.group in (HARD, NORM)]
        self.driver = [i for i, row in enumerate(self.rows) if row.group == DRIVER]
        self.spring = [i for i, row in enumerate(self.rows) if row.group == SPRING]

        # `Π = ½ Σ gᵢ²/cᵢ`, so `∇Π = Σ (gᵢ/cᵢ) ∇gᵢ`. That is assembled from the row Jacobian at
        # call time (see `gradient`) rather than differentiated symbolically: a symbolic gradient
        # is a new expression for every compliant component, and three of them at one frame joint
        # take minutes to compile and then do not evaluate at all. The rows are the same
        # expressions the residual already uses, so the number is the same one.

        self._f_rows = self._lambdify(self._exprs)
        self._f_jac = self._lambdify(self._cache_jacobian())
        self._point_cache: dict[tuple[str, str], tuple[Callable, Callable]] = {}

    def _allocate_geometry(self) -> None:
        topo, res = self.topology, self.resolved

        def geo(key: tuple, count: int, prefix: str) -> list[sp.Symbol]:
            syms = list(sp.symbols(f"{prefix}_0:{count}", real=True))
            self.g.extend(syms)
            self._gkey.extend([key + (i,) for i in range(count)])
            return syms

        c_sites: set[str] = set()
        for name in res.joint_order:
            joint = res.joints[name]
            if joint.family == "frame":
                c_sites.add(joint.site)
            elif joint.family == "plane":
                c_sites.add(joint.from_)
        # Every geometry point gets symbols, not only the ones a joint touches: a load or an
        # output may name any of them, and the symbol table must not depend on the study.
        for point in sorted({p for spec in topo.bodies.values() for p in spec.points}):
            self._point_sym[point] = sp.Matrix(geo(("point", point), 3, f"g_{point}"))
        for site in sorted(c_sites):
            self._site_c[site] = sp.Matrix(3, 3, geo(("site", site), 9, f"gc_{site}"))
        for name in res.joint_order:
            joint = res.joints[name]
            if joint.family == "axial":
                geo(("length", name), 1, f"gl_{name}")
            if joint.family == "plane":
                geo(("gap", name), 1, f"gg_{name}")
            for component in joint.spring_components:
                geo(("compliance", name, component), 1, f"gk_{name}_{component}")

    def _add_joint_rows(self) -> None:
        for name in self.resolved.joint_order:
            joint = self.resolved.joints[name]
            if joint.family == "frame":
                d, v_site, _ = self._frame_relative(joint)
                for component in joint.components:
                    mode = joint.modes[component]
                    if mode == "free":
                        continue
                    if component in TRANSLATION:
                        expr = d[TRANSLATION.index(component), 0]
                        unit = "m"
                    else:
                        expr = v_site[ROTATION.index(component), 0]
                        unit = ""
                    self._row(f"{name}.{component}", unit, expr, SPRING if mode == "spring" else HARD)
            elif joint.family == "axial":
                mode = joint.modes["length"]
                if mode == "free":
                    continue
                delta = self._world(joint.b, joint.to) - self._world(joint.a, joint.from_)
                expr = sp.sqrt(delta.dot(delta)) - self._find(("length", name))
                self._row(f"{name}.length", "m", expr, SPRING if mode == "spring" else HARD)
            else:
                mode = joint.modes["gap"]
                if mode == "free":
                    continue
                normal = self._pose[joint.a][1] * self._site_c[joint.from_] * sp.Matrix([0, 0, 1])
                delta = self._world(joint.b, joint.to) - self._world(joint.a, joint.from_)
                expr = (normal.T * delta)[0, 0] - self._find(("gap", name))
                self._row(f"{name}.gap", "m", expr, SPRING if mode == "spring" else HARD)

    def _add_drivers(self) -> None:
        self.drives: list[Drive] = []
        for actuator in self.topology.actuators:
            joint = self.resolved.joints[actuator.joint]
            if joint.family != "frame":
                raise ValueError(f"actuator {actuator.name!r}: only `frame` joints can be driven in v1")
            theta = sp.Symbol(f"t_{actuator.name}", real=True)
            d, v_site, w_rel = self._frame_relative(joint)
            if actuator.dof in TRANSLATION:
                expr = d[TRANSLATION.index(actuator.dof), 0] - theta
                unit = "m"
            else:
                expr = 2 * sp.atan2(v_site[ROTATION.index(actuator.dof), 0], w_rel) - theta
                unit = "rad"
            row = f"{joint.name}.{actuator.dof}"
            self._row(row, unit, expr, DRIVER)
            self.drives.append(Drive(actuator.name, joint.name, actuator.dof, unit, theta, row))

    def _add_norm_rows(self) -> None:
        for body in self.moving:
            i = self._qstart[body]
            w, x, y, z = self.q[i + 3 : i + 7]
            self._row(f"{body}.unit", "", w**2 + x**2 + y**2 + z**2 - 1, NORM)

    # -- symbolic helpers --------------------------------------------------------------

    def _frame_relative(self, joint: ResolvedJoint) -> tuple[sp.Matrix, sp.Matrix, sp.Expr]:
        p_a, r_a = self._pose[joint.a]
        p_b, r_b = self._pose[joint.b]
        origin = self._site_point(joint.site)
        c = self._site_c[joint.site]
        d = c.T * r_a.T * ((p_b + r_b * origin) - (p_a + r_a * origin))
        w_a, v_a = self._quat(joint.a)
        w_b, v_b = self._quat(joint.b)
        w_rel = sp.expand(w_a * w_b + (v_a.T * v_b)[0, 0])
        v_site = c.T * (w_a * v_b - w_b * v_a - v_a.cross(v_b))
        return d, v_site, w_rel

    def _quat(self, body: str) -> tuple[sp.Expr, sp.Matrix]:
        if body in self.fixed:
            return sp.Integer(1), sp.zeros(3, 1)
        i = self._qstart[body]
        w, x, y, z = self.q[i + 3 : i + 7]
        return w, sp.Matrix([x, y, z])

    def _site_point(self, site: str) -> sp.Matrix:
        return self._point_sym[self.topology.sites[site].origin]

    def _world(self, body: str, site: str) -> sp.Matrix:
        p, r = self._pose[body]
        return p + r * self._site_point(site)

    def _find(self, key: tuple) -> sp.Symbol:
        for i, k in enumerate(self._gkey):
            if k[: len(key)] == key:
                return self.g[i]
        raise KeyError(key)

    def _row(self, name: str, unit: str, expr: sp.Expr, group: str) -> None:
        self.rows.append(Row(name, unit, group))
        self._exprs.append(sp.expand(expr))

    # -- lambdify ----------------------------------------------------------------------

    def _args(self) -> list[sp.Symbol]:
        return [*self.q, *self.g, *[drive.theta for drive in self.drives]]

    def _lambdify(self, expr) -> Callable:
        return sp.lambdify(self._args(), expr, modules=["numpy"], cse=True)

    def _cache_jacobian(self):
        self.jacobian_expr = sp.Matrix(self._exprs).jacobian(self.q)
        return self.jacobian_expr

    # -- numeric surface ---------------------------------------------------------------

    def numbers(self, resolved: Resolved | None = None) -> np.ndarray:
        resolved = resolved or self.resolved
        values = np.zeros(len(self.g))
        for i, key in enumerate(self._gkey):
            kind = key[0]
            if kind == "point":
                values[i] = resolved.points[key[1]][key[2]]
            elif kind == "site":
                values[i] = resolved.sites[key[1]].C[key[2] // 3, key[2] % 3]
            elif kind == "length":
                values[i] = resolved.length[key[1]]
            elif kind == "gap":
                values[i] = resolved.gap[key[1]]
            elif kind == "compliance":
                values[i] = resolved.compliance[(key[1], key[2])]
            else:  # pragma: no cover
                raise KeyError(key)
        return values

    def compliance(self, g: np.ndarray) -> dict[tuple[str, str], float]:
        out = {}
        for i, key in enumerate(self._gkey):
            if key[0] == "compliance":
                out[(key[1], key[2])] = float(g[i])
        return out

    def q0(self) -> np.ndarray:
        q = np.zeros(self.n)
        for body in self.moving:
            q[self._qstart[body] + 3] = 1.0
        return q

    def theta0(self) -> np.ndarray:
        return np.zeros(len(self.drives))

    def theta_for(self, values: dict[str, float]) -> np.ndarray:
        return np.array([values.get(d.actuator, 0.0) for d in self.drives], dtype=float)

    def evaluate(self, q, g, theta) -> np.ndarray:
        return np.asarray(self._f_rows(*q, *g, *theta), dtype=float).reshape(-1)

    def jacobian(self, q, g, theta) -> np.ndarray:
        return np.asarray(self._f_jac(*q, *g, *theta), dtype=float).reshape(-1, self.n)

    # -- energy ------------------------------------------------------------------------

    @property
    def has_springs(self) -> bool:
        return bool(self.spring)

    def gradient(self, q, g, theta, rows=None) -> np.ndarray:
        """The gradient of the spring potential, assembled row by row.

        `Π = ½ Σ gᵢ²/cᵢ` is a sum over the compliant rows, so its gradient is
        `Σ (gᵢ/cᵢ) ∇gᵢ` — the row Jacobian times a per-row weight, which is what this computes.
        Differentiating the sum symbolically instead produces one enormous expression: it was
        fine for a single compliant component (the tie rod) and unusable for three at one frame
        joint, where the model took 63 s to compile and the gradient never returned.

        `rows` restricts the sum to the rows that actually deflect; a component whose compliance
        is zero is a rigid constraint, not a spring, and `1/0` is not a force.
        """
        indices = self.spring if rows is None else list(rows)
        if not indices:
            return np.zeros(self.n)
        values = self.evaluate(q, g, theta)
        jacobian = self.jacobian(q, g, theta)
        compliance = self.compliance(g)
        out = np.zeros(self.n)
        for i in indices:
            name, component = self.rows[i].name.split(".", 1)
            out += (values[i] / compliance[(name, component)]) * jacobian[i]
        return out

    def point_derivatives(self, body: str, point: str) -> tuple[Callable, Callable]:
        key = (body, point)
        if key in self._point_cache:
            return self._point_cache[key]
        p, r = self._pose[body]
        origin = self._point_sym[point]
        expr = p + r * origin
        args = [*self.q, *self.g]
        jac = expr.jacobian(self.q)
        hess = [sp.Matrix(self.n, self.n, lambda a, b: expr[c].diff(self.q[a], self.q[b])) for c in range(3)]
        entry = (sp.lambdify(args, jac, modules=["numpy"], cse=True), sp.lambdify(args, hess, modules=["numpy"], cse=True))
        self._point_cache[key] = entry
        return entry

    # -- solving -----------------------------------------------------------------------

    def solve(self, geometry, study=None):
        """Solve a study against a different set of numbers, reusing this derivation.

        The symbolic build is cached by (topology, library), so a design loop pays for the
        derivation once and only re-resolves the numbers.
        """
        from ..solvers.sweep import sweep

        model = self.model.with_geometry(geometry)
        if study is not None:
            model.study = study
        return sweep(model)

    # -- poses and points --------------------------------------------------------------

    def poses(self, q) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        out = {}
        for body, is_fixed in self.resolved.bodies.items():
            if is_fixed:
                out[body] = (np.zeros(3), np.eye(3))
                continue
            i = self._qstart[body]
            out[body] = (
                np.asarray(q[i : i + 3], dtype=float),
                _rotation_np(*np.asarray(q[i + 3 : i + 7], dtype=float)),
            )
        return out

    def world_points(self, q) -> dict[str, np.ndarray]:
        poses = self.poses(q)
        return {
            point: poses[body][0] + poses[body][1] @ self.resolved.points[point]
            for point, body in self.resolved.point_body.items()
        }


# -- quaternion helpers ----------------------------------------------------------------


def _rotation(w, x, y, z) -> sp.Matrix:
    return sp.Matrix(
        [
            [w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z],
        ]
    )


def _rotation_np(w: float, x: float, y: float, z: float) -> np.ndarray:
    xx, yy, zz = x * x, y * y, z * z
    return np.array(
        [
            [w * w + xx - yy - zz, 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), w * w - xx + yy - zz, 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), w * w - xx - yy + zz],
        ]
    )


def _key(model: Model) -> str:
    return model.topology.model_dump_json() + "|" + "|".join(
        f"{name}:{kind.model_dump_json()}" for name, kind in sorted(model.kinds.items())
    )


def compile(model: Model) -> Problem:  # noqa: A001 - the public name is `compile`
    return Problem(model)
