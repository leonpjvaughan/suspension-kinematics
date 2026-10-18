"""Loading, resolving and validating a model.

Two stages, deliberately: `load` parses and resolves names and numbers (everything that can
be checked without mathematics), and `check` compiles and evaluates the design position and
the rank. A model that cannot be resolved never reaches the symbolic layer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from pydantic import BaseModel, ValidationError

from ..errors import ConfigError, Fault
from .geometry import Geometry
from .kinds import FAMILY_COMPONENTS, ROTATION, ConstraintKind, Mode, load_library
from .study import Study
from .topology import Topology

@dataclass(frozen=True)
class SiteFrame:
    origin: np.ndarray  # (3,) world at design
    C: np.ndarray  # (3, 3), columns are the site's x, y, z axes in world at design

    @property
    def normal(self) -> np.ndarray:
        return self.C[:, 2]


@dataclass(frozen=True)
class ResolvedJoint:
    name: str
    kind_name: str
    family: str
    a: str
    b: str
    modes: dict[str, Mode]
    site: str | None
    from_: str | None
    to: str | None

    @property
    def components(self) -> tuple[str, ...]:
        return FAMILY_COMPONENTS[self.family]

    @property
    def free_rotations(self) -> list[str]:
        if self.family != "frame":
            return []
        return [c for c in ROTATION if self.modes[c] == "free"]

    @property
    def spring_components(self) -> list[str]:
        return [c for c in self.components if self.modes[c] == "spring"]


@dataclass(frozen=True)
class ResolvedActuator:
    name: str
    joint: str
    dof: str
    effort: str
    unit: str


@dataclass(frozen=True)
class Resolved:
    bodies: dict[str, bool]
    point_body: dict[str, str]
    reference: str
    road: str | None
    axle: tuple[str, str]
    points: dict[str, np.ndarray]
    sites: dict[str, SiteFrame]
    joints: dict[str, ResolvedJoint]
    joint_order: list[str]
    length: dict[str, float]
    gap: dict[str, float]
    compliance: dict[tuple[str, str], float]
    actuators: dict[str, ResolvedActuator]
    warnings: tuple[str, ...] = ()


@dataclass
class Model:
    topology: Topology
    geometry: Geometry
    study: Study | None
    kinds: dict[str, ConstraintKind]
    files: dict[str, str] = field(default_factory=dict)

    # -- loading -----------------------------------------------------------------------

    @classmethod
    def load(cls, path: str | Path, *, study: str | Path | None = None) -> "Model":
        topology_path = Path(path)
        if topology_path.is_dir():
            directory = topology_path
            topology_path = directory / "topology.yaml"
        else:
            directory = topology_path.parent
        if not topology_path.exists():
            raise ConfigError([Fault(str(topology_path), "", "no such file")])

        faults: list[Fault] = []
        topology = _read(Topology, topology_path, faults)
        geometry_path = directory / "geometry.yaml"
        geometry = _read(Geometry, geometry_path, faults)
        study_path = Path(study) if study is not None else directory / "study.yaml"
        study_model: Study | None = None
        if study_path.exists():
            study_model = _read(Study, study_path, faults)
        elif study is not None:
            faults.append(Fault(str(study_path), "", "no such study file"))
        if faults:
            raise ConfigError(faults)
        assert topology is not None and geometry is not None

        library_path = (topology_path.parent / topology.constraints).resolve()
        kinds = load_library(library_path)
        for name, kind in topology.constraint_kinds.items():
            if name in kinds:
                faults.append(
                    Fault(
                        str(topology_path),
                        f"constraint_kinds.{name}",
                        f"defined twice: also in {library_path.name}",
                    )
                )
            kinds[name] = kind
        if faults:
            raise ConfigError(faults)

        if study_model is not None and study_model.analysis == "dynamics":
            raise ConfigError(
                [
                    Fault(
                        str(study_path),
                        "analysis",
                        "dynamics is designed but not implemented; see docs/dynamics.md",
                    )
                ]
            )

        return cls(
            topology=topology,
            geometry=geometry,
            study=study_model,
            kinds=kinds,
            files={
                "topology": str(topology_path),
                "geometry": str(geometry_path),
                "study": str(study_path) if study_model else "",
                "constraints": str(library_path),
            },
        )

    def with_geometry(self, geometry: Geometry) -> "Model":
        """The same topology, library and study against a different set of numbers."""
        return Model(
            topology=self.topology,
            geometry=geometry,
            study=self.study,
            kinds=self.kinds,
            files=self.files,
        )

    # -- resolution --------------------------------------------------------------------

    def resolve(self) -> Resolved:
        topo, geom, files = self.topology, self.geometry, self.files
        t_file, g_file = files.get("topology", "topology.yaml"), files.get("geometry", "geometry.yaml")
        faults: list[Fault] = []
        warnings: list[str] = []

        # bodies and point ownership
        point_body: dict[str, str] = {}
        for body, spec in topo.bodies.items():
            for point in spec.points:
                if point not in geom.points:
                    faults.append(Fault(t_file, f"bodies.{body}.points", f"no point {point!r} in geometry"))
                elif point in point_body:
                    faults.append(
                        Fault(t_file, f"bodies.{body}.points", f"point {point!r} is owned by {point_body[point]!r} too")
                    )
                else:
                    point_body[point] = body
        for point in geom.points:
            if point not in point_body:
                faults.append(
                    Fault(t_file, "bodies", f"point {point!r} belongs to no body: list it under bodies.<name>.points")
                )

        points = {name: np.asarray(v, dtype=float) for name, v in geom.points.items()}
        axes = {}
        for name, raw in geom.axes.items():
            vec = np.asarray(raw, dtype=float)
            norm = float(np.linalg.norm(vec))
            if norm == 0.0:
                faults.append(Fault(g_file, f"axes.{name}", "zero-length axis"))
                continue
            if abs(norm - 1.0) > 1e-6:
                warnings.append(f"{g_file}: axes.{name} is not unit length ({norm:.6g}); normalised")
            axes[name] = vec / norm

        # sites
        sites: dict[str, SiteFrame] = {}
        for name, site in topo.sites.items():
            origin = points.get(site.origin)
            if origin is None:
                faults.append(Fault(t_file, f"sites.{name}.origin", f"no point {site.origin!r} in geometry"))
                continue
            vecs = []
            for axis_name in (site.x, site.y, site.z):
                if axis_name is None:
                    vecs.append(None)
                elif axis_name not in axes:
                    faults.append(Fault(t_file, f"sites.{name}", f"no axis {axis_name!r} in geometry"))
                    vecs.append(None)
                else:
                    vecs.append(axes[axis_name])
            try:
                frame = _frame(vecs[0], vecs[1], vecs[2])
            except ValueError as exc:
                faults.append(Fault(t_file, f"sites.{name}", str(exc)))
                continue
            sites[name] = SiteFrame(origin=origin, C=frame)

        # joints
        joints: dict[str, ResolvedJoint] = {}
        order: list[str] = []
        for joint in topo.joints:
            if joint.name in joints:
                faults.append(Fault(t_file, f"joints.{joint.name}", "duplicate joint name"))
                continue
            kind = self.kinds.get(joint.kind)
            if kind is None:
                faults.append(Fault(t_file, f"joints.{joint.name}.kind", f"unknown kind {joint.kind!r}"))
                continue
            for end in (joint.a, joint.b):
                if end not in topo.bodies:
                    faults.append(Fault(t_file, f"joints.{joint.name}", f"unknown body {end!r}"))
            if joint.a == joint.b:
                faults.append(Fault(t_file, f"joints.{joint.name}", "a joint from a body to itself is not a constraint"))
            try:
                effective = kind.overridden(joint.modes)
            except ValueError as exc:
                faults.append(Fault(t_file, f"joints.{joint.name}.modes", str(exc)))
                continue
            if kind.family == "frame":
                if joint.site is None or joint.site not in sites:
                    faults.append(Fault(t_file, f"joints.{joint.name}.site", f"needs a site; {joint.site!r} is not one"))
            else:
                for attr, value in (("from", joint.from_), ("to", joint.to)):
                    if value is None or value not in sites:
                        faults.append(Fault(t_file, f"joints.{joint.name}.{attr}", f"needs a site; {value!r} is not one"))
            if kind.family == "plane" and joint.from_ in topo.sites:
                site = topo.sites[joint.from_]
                if site.z is None:
                    faults.append(
                        Fault(t_file, f"joints.{joint.name}.from", f"site {joint.from_!r} must declare `z` (the plane normal)")
                    )
            joints[joint.name] = ResolvedJoint(
                name=joint.name,
                kind_name=joint.kind,
                family=kind.family,
                a=joint.a,
                b=joint.b,
                modes=effective.modes,
                site=joint.site,
                from_=joint.from_,
                to=joint.to,
            )
            order.append(joint.name)

        # numbers: length, gap, compliance
        length: dict[str, float] = {}
        gap: dict[str, float] = {}
        compliance: dict[tuple[str, str], float] = {}
        for name in order:
            joint = joints[name]
            numbers = geom.joints.get(name)
            if joint.family == "axial":
                computed = _design_separation(joint, sites)
                given = numbers.length if numbers else None
                if given is None:
                    if computed is None:
                        continue
                    length[name] = computed
                    warnings.append(f"{g_file}: joints.{name}.length defaulted to the design separation {computed:.9g} m")
                elif computed is not None and abs(given - computed) > 1e-9:
                    faults.append(
                        Fault(
                            g_file,
                            f"joints.{name}.length = {given:.6f}",
                            f"the design position puts the two sites {computed:.6f} apart; "
                            f"fix the length or delete it to default",
                        )
                    )
                else:
                    length[name] = given
            if joint.family == "plane":
                computed = _design_gap(joint, sites)
                given = numbers.gap if numbers else None
                if given is None:
                    faults.append(Fault(g_file, f"joints.{name}.gap", "plane joints need a gap (the tyre radius, for a road)"))
                elif computed is not None and abs(given - computed) > 1e-9:
                    faults.append(
                        Fault(
                            g_file,
                            f"joints.{name}.gap = {given:.6f}",
                            f"the design position puts the point {computed:.6f} from the plane; "
                            f"fix the gap or move the geometry",
                        )
                    )
                else:
                    gap[name] = given
            for component in joint.components:
                mode = joint.modes[component]
                c_val = numbers.compliance.get(component) if numbers else None
                k_val = numbers.stiffness.get(component) if numbers else None
                if mode == "spring":
                    if c_val is not None and k_val is not None:
                        faults.append(
                            Fault(g_file, f"joints.{name}", f"{component}: give `compliance` or `stiffness`, not both")
                        )
                    elif k_val is not None:
                        if k_val <= 0:
                            faults.append(Fault(g_file, f"joints.{name}.stiffness.{component}", "must be positive"))
                        else:
                            compliance[(name, component)] = 1.0 / k_val
                    elif c_val is not None:
                        if c_val < 0:
                            faults.append(Fault(g_file, f"joints.{name}.compliance.{component}", "must not be negative"))
                        else:
                            compliance[(name, component)] = c_val
                    else:
                        faults.append(
                            Fault(
                                g_file,
                                f"joints.{name}",
                                f"{component} is `spring` in the topology, so give `compliance` (or `stiffness`) here",
                            )
                        )
                elif c_val is not None or k_val is not None:
                    faults.append(
                        Fault(
                            g_file,
                            f"joints.{name}",
                            f"{component} is `{mode}` in the topology; a compliance here would be silently ignored",
                        )
                    )

        # actuators
        actuators: dict[str, ResolvedActuator] = {}
        for actuator in topo.actuators:
            if actuator.name in actuators:
                faults.append(Fault(t_file, f"actuators.{actuator.name}", "duplicate actuator name"))
                continue
            joint = joints.get(actuator.joint)
            if joint is None:
                faults.append(Fault(t_file, f"actuators.{actuator.name}.joint", f"unknown joint {actuator.joint!r}"))
                continue
            if actuator.dof not in joint.components:
                faults.append(
                    Fault(
                        t_file,
                        f"actuators.{actuator.name}.dof",
                        f"{actuator.dof!r} is not a component of {joint.family} (the joint's kind is {joint.kind_name!r})",
                    )
                )
                continue
            mode = joint.modes[actuator.dof]
            if mode != "free":
                faults.append(
                    Fault(
                        t_file,
                        f"actuators.{actuator.name}.dof",
                        f"{actuator.dof} is `{mode}`, not `free`: a compliant or locked component cannot be driven",
                    )
                )
                continue
            if actuator.dof in ROTATION and len(joint.free_rotations) != 1:
                faults.append(
                    Fault(
                        t_file,
                        f"actuators.{actuator.name}.dof",
                        f"the joint has {len(joint.free_rotations)} free rotations; "
                        f"an angle about one axis is a coordinate only when there is exactly one",
                    )
                )
                continue
            unit = "rad" if actuator.dof in ROTATION else "m"
            actuators[actuator.name] = ResolvedActuator(
                name=actuator.name, joint=actuator.joint, dof=actuator.dof, effort=actuator.effort, unit=unit
            )

        # roles
        reference = topo.roles.reference
        if reference is None:
            fixed = [body for body, spec in topo.bodies.items() if spec.fixed]
            if len(fixed) == 1:
                reference = fixed[0]
            elif not fixed:
                faults.append(Fault(t_file, "bodies", "no fixed body: mark the inertial body `fixed: true`"))
            else:
                faults.append(
                    Fault(t_file, "roles.reference", f"{len(fixed)} fixed bodies; say which one outputs use")
                )
        elif reference not in topo.bodies:
            faults.append(Fault(t_file, "roles.reference", f"unknown body {reference!r}"))
        road = topo.roles.road
        if road is not None:
            if road not in topo.bodies:
                faults.append(Fault(t_file, "roles.road", f"unknown body {road!r}"))
            elif road == reference:
                faults.append(Fault(t_file, "roles.road", "the road cannot be the reference body"))

        axle: tuple[str, str] | None = None
        raw_axle = topo.roles.axle
        if raw_axle is None:
            faults.append(
                Fault(t_file, "roles.axle", "name the two geometry points that define the wheel axle, e.g. [wc, wco]")
            )
        elif len(raw_axle) != 2 or raw_axle[0] == raw_axle[1]:
            faults.append(Fault(t_file, "roles.axle", "needs exactly two distinct points"))
        else:
            missing = [point for point in raw_axle if point not in points]
            if missing:
                faults.append(Fault(t_file, "roles.axle", f"no such point(s) in geometry: {missing}"))
            else:
                axle = (raw_axle[0], raw_axle[1])

        # The dynamics block is optional and ignored by the kinematic solver, but a reference
        # inside it that does not resolve is still a broken file, and it is cheaper to say so at
        # load than when the dynamics work first reads it.
        dynamics = geom.dynamics
        if dynamics is not None:
            for body, numbers in dynamics.bodies.items():
                where = f"dynamics.bodies.{body}"
                if body not in topo.bodies:
                    faults.append(Fault(g_file, where, f"unknown body {body!r}"))
                    continue
                if topo.bodies[body].fixed:
                    warnings.append(f"{g_file}: {where}: a fixed body's mass has no effect")
                if numbers.mass <= 0:
                    faults.append(Fault(g_file, f"{where}.mass", "must be positive"))
                if isinstance(numbers.com, str):
                    if numbers.com not in points:
                        faults.append(
                            Fault(g_file, f"{where}.com", f"no point {numbers.com!r}; give a point name or [x, y, z]")
                        )
                elif not all(np.isfinite(numbers.com)):
                    faults.append(Fault(g_file, f"{where}.com", "not finite"))
                inertia = numbers.inertia
                if not all(np.isfinite(inertia)):
                    faults.append(Fault(g_file, f"{where}.inertia", "not finite"))
                elif min(inertia[0], inertia[1], inertia[2]) <= 0:
                    faults.append(
                        Fault(g_file, f"{where}.inertia", "the three principal moments (ixx, iyy, izz) must be positive")
                    )

        if faults:
            raise ConfigError(faults)

        assert reference is not None and axle is not None
        return Resolved(
            bodies={body: spec.fixed for body, spec in topo.bodies.items()},
            point_body=point_body,
            reference=reference,
            road=road,
            axle=axle,
            points=points,
            sites=sites,
            joints=joints,
            joint_order=order,
            length=length,
            gap=gap,
            compliance=compliance,
            actuators=actuators,
            warnings=tuple(warnings),
        )


# -- helpers ---------------------------------------------------------------------------


def _read(cls: type[BaseModel], path: Path, faults: list[Fault]) -> object:
    if not path.exists():
        faults.append(Fault(str(path), "", "no such file"))
        return None
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        faults.append(Fault(str(path), "", f"not valid YAML: {exc}"))
        return None
    if not isinstance(data, dict):
        faults.append(Fault(str(path), "", "expected a YAML mapping"))
        return None
    version = data.get("version", 1)
    if version != 1:
        faults.append(Fault(str(path), "version", f"this build understands version 1, not {version!r}"))
        return None
    try:
        return cls.model_validate(data)
    except ValidationError as exc:
        for err in exc.errors():
            loc = ".".join(str(p) for p in err["loc"])
            faults.append(Fault(str(path), loc, err["msg"]))
        return None


def _frame(x: np.ndarray | None, y: np.ndarray | None, z: np.ndarray | None) -> np.ndarray:
    """Complete a site's declared axes into a right-handed orthonormal frame, z last."""
    zz = z if z is not None else np.array([0.0, 0.0, 1.0])
    zz = zz / np.linalg.norm(zz)
    if x is not None:
        xx = x - float(x @ zz) * zz
        if np.linalg.norm(xx) < 1e-12:
            raise ValueError("site x axis is parallel to z")
        xx /= np.linalg.norm(xx)
        yy = np.cross(zz, xx)
    elif y is not None:
        yy = y - float(y @ zz) * zz
        if np.linalg.norm(yy) < 1e-12:
            raise ValueError("site y axis is parallel to z")
        yy /= np.linalg.norm(yy)
        xx = np.cross(yy, zz)
    else:
        helper = np.array([1.0, 0.0, 0.0]) if abs(zz[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        xx = helper - float(helper @ zz) * zz
        xx /= np.linalg.norm(xx)
        yy = np.cross(zz, xx)
    return np.column_stack([xx, yy, zz])


def _design_separation(joint: ResolvedJoint, sites: dict[str, SiteFrame]) -> float | None:
    if joint.from_ not in sites or joint.to not in sites:
        return None
    return float(np.linalg.norm(sites[joint.to].origin - sites[joint.from_].origin))


def _design_gap(joint: ResolvedJoint, sites: dict[str, SiteFrame]) -> float | None:
    if joint.from_ not in sites or joint.to not in sites:
        return None
    frame = sites[joint.from_]
    return float(frame.normal @ (sites[joint.to].origin - frame.origin))
