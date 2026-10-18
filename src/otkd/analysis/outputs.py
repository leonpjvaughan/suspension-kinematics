"""The quantities the book draws, all from a solved pose.

Every value is computed in the frame of the model's `reference` body: for the standing-car
models that body is fixed, so this is the world frame; for a road-driven model the chassis is
the reference and the moving road drops out of the answer, which is what a reader of a camber
curve means by "camber".

Sign conventions, stated once here and pinned by tests:

* **camber**, degrees, positive when the *top* of the wheel leans outboard. From the axle
  unit vector `n` (`roles.axle`, e.g. `wc -> wco`), `camber = asin(-n_z)`.
* **toe**, degrees, the change in the axle's plan-view heading from the design position,
  positive for toe-in. `heading = atan2(n_y, n_x)`; for a right-hand corner a positive `n_y`
  is toe-in.
* **track change**, metres, lateral movement of the contact patch, outboard positive.
* **instant centre**, the front-view intersection of the line through each ball joint and the
  foot of the perpendicular from it onto its arm's hinge line. This is the standard front-view
  construction and it is **exact when the hinge axes are perpendicular to the front view**,
  which is the idealisation the front view is for. With a tilted axis no exact front-view
  centre exists at all: the projection of a 3D rotation is a rotation *plus a stretch*, so
  there is no point where the projected velocity vanishes. The construction is then an
  approximation of order the tilt, and `test_outputs.py` pins both cases. `None` when the
  lines are parallel.
* **roll centre**, where the line from the contact patch through the instant centre crosses
  the centreline `x = 0` — the mirrored-corner assumption. `None` when undefined.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config.kinds import ROTATION
from ..problem.compile import Problem


@dataclass(frozen=True)
class Outputs:
    camber: float
    toe: float
    ic: tuple[float, float] | None
    rc: tuple[float, float] | None
    track: float
    contact: tuple[float, float] | None


@dataclass(frozen=True)
class Reference:
    heading: float
    contact: np.ndarray
    tyre_radius: float
    lines: tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]] | None


def make_reference(problem: Problem) -> Reference:
    resolved = problem.resolved
    q0 = problem.q0()
    poses = problem.poses(q0)
    points = problem.world_points(q0)
    p_ref, r_ref = poses[resolved.reference]

    def to_ref(point: np.ndarray) -> np.ndarray:
        return r_ref.T @ (point - p_ref)

    axle = _axle(problem, points)
    radius = _tyre_radius(problem)
    contact = to_ref(_contact(problem, points, poses, axle, radius))
    lines = None
    structure = _front_view(problem)
    if structure is not None:
        lines = (
            (to_ref(points[structure[0]]), to_ref(structure[1])),
            (to_ref(points[structure[2]]), to_ref(structure[3])),
        )
    return Reference(
        heading=float(np.arctan2(axle[1], axle[0])),
        contact=contact,
        tyre_radius=radius,
        lines=lines,
    )


def compute(problem: Problem, q: np.ndarray, reference: Reference) -> Outputs:
    poses = problem.poses(q)
    points = problem.world_points(q)
    p_ref, r_ref = poses[problem.resolved.reference]

    def to_ref(point: np.ndarray) -> np.ndarray:
        return r_ref.T @ (point - p_ref)

    axle = _axle(problem, points)
    heading = float(np.arctan2(axle[1], axle[0]))
    toe = float(
        np.degrees(np.arctan2(np.sin(heading - reference.heading), np.cos(heading - reference.heading)))
    )
    camber = float(np.degrees(np.arcsin(np.clip(-axle[2], -1.0, 1.0))))
    contact = to_ref(_contact(problem, points, poses, axle, reference.tyre_radius))
    track = float(contact[0] - reference.contact[0])

    ic = None
    rc = None
    if reference.lines is not None:
        structure = _front_view(problem)
        assert structure is not None
        lbj, _, ubj, _ = structure
        lower = to_ref(points[lbj])
        upper = to_ref(points[ubj])
        ic = _intersect(
            (
                np.array([lower[0], lower[2]]),
                np.array([reference.lines[0][1][0] - lower[0], reference.lines[0][1][2] - lower[2]]),
            ),
            (
                np.array([upper[0], upper[2]]),
                np.array([reference.lines[1][1][0] - upper[0], reference.lines[1][1][2] - upper[2]]),
            ),
        )
        if ic is not None:
            rc = _roll_centre(contact, ic)

    return Outputs(
        camber=camber,
        toe=toe,
        ic=ic,
        rc=rc,
        track=track,
        # Always reported: the solved patch where there is a road, the lowest point of the
        # wheel where there is not. A figure wants it either way.
        contact=(float(contact[0]), float(contact[2])),
    )


# -- pieces ----------------------------------------------------------------------------


def _axle(problem: Problem, points: dict[str, np.ndarray]) -> np.ndarray:
    a, b = problem.resolved.axle
    return _unit(points[b] - points[a])


def _tyre_radius(problem: Problem) -> float:
    for value in problem.resolved.gap.values():
        return float(abs(value))
    scalar = problem.model.geometry.scalars.get("tyre-radius")
    if scalar is None:
        return 0.0
    return float(scalar)


def _contact(
    problem: Problem,
    points: dict[str, np.ndarray],
    poses: dict[str, tuple[np.ndarray, np.ndarray]],
    axle: np.ndarray,
    radius: float,
) -> np.ndarray:
    a, b = problem.resolved.axle
    centre = 0.5 * (points[a] + points[b])
    road = problem.resolved.road
    if road is not None:
        for joint in problem.resolved.joints.values():
            if joint.family == "plane" and joint.a == road:
                p_a, r_a = poses[joint.a]
                site = problem.resolved.sites[joint.from_]
                normal = _unit(r_a @ site.normal)
                distance = float(normal @ (centre - (p_a + r_a @ site.origin)))
                return centre - distance * normal
    down = -np.array([0.0, 0.0, 1.0])
    in_plane = down - float(down @ axle) * axle
    return centre + radius * _unit(in_plane)


def _front_view(problem: Problem) -> tuple[str, np.ndarray, str, np.ndarray] | None:
    """The two ball joints and the point on each arm's hinge line nearest to its ball joint.

    Found structurally: the upright owns the axle and is joined to two arms by sphericals. Each
    arm's hinge is a revolute (its site's z), or the line through the two sphericals that pin a
    two-bush arm to the reference body. The nearest point on that line is what the front view
    wants, not an arbitrary bushing origin.
    """
    resolved = problem.resolved
    upright = resolved.point_body.get(resolved.axle[0])
    if upright is None:
        return None
    spheres = [
        joint
        for joint in resolved.joints.values()
        if joint.family == "frame"
        and all(joint.modes[c] == "locked" for c in ("tx", "ty", "tz"))
        and all(joint.modes[c] == "free" for c in ROTATION)
        and upright in (joint.a, joint.b)
    ]
    if len(spheres) != 2:
        return None

    out: list[tuple[str, np.ndarray]] = []
    for sphere in spheres:
        arm = sphere.b if sphere.a == upright else sphere.a
        ball = problem.topology.sites[sphere.site].origin
        hinge = _hinge_line(problem, arm)
        if hinge is None:
            return None
        point, direction = hinge
        foot = point + float((resolved.points[ball] - point) @ direction) * direction
        out.append((ball, foot))
    return (out[0][0], out[0][1], out[1][0], out[1][1])


def _hinge_line(problem: Problem, arm: str) -> tuple[np.ndarray, np.ndarray] | None:
    resolved = problem.resolved
    joints = [
        joint
        for joint in resolved.joints.values()
        if joint.family == "frame" and {joint.a, joint.b} == {arm, resolved.reference}
        and joint.site is not None
    ]
    revolutes = [
        joint
        for joint in joints
        if joint.modes["rz"] == "free"
        and all(joint.modes[c] == "locked" for c in ("tx", "ty", "tz", "rx", "ry"))
    ]
    if len(revolutes) == 1:
        site = resolved.sites[revolutes[0].site]
        return site.origin, _unit(site.normal)
    if len(joints) == 2:
        first = resolved.sites[joints[0].site].origin
        second = resolved.sites[joints[1].site].origin
        return first, _unit(second - first)
    return None


def _unit(v: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    return v / norm if norm else v


def _intersect(
    first: tuple[np.ndarray, np.ndarray], second: tuple[np.ndarray, np.ndarray]
) -> tuple[float, float] | None:
    """Intersection of two front-view lines given as (point, direction), in (x, z)."""
    (a, u), (c, v) = first, second
    cross = u[0] * v[1] - u[1] * v[0]
    scale = float(np.linalg.norm(u) * np.linalg.norm(v))
    if scale == 0.0 or abs(cross) < 1e-12 * scale:
        return None
    delta = c - a
    t = (delta[0] * v[1] - delta[1] * v[0]) / cross
    point = a + t * u
    return (float(point[0]), float(point[1]))


def _roll_centre(contact: np.ndarray, ic: tuple[float, float]) -> tuple[float, float] | None:
    x_c, z_c = float(contact[0]), float(contact[2])
    x_i, z_i = ic
    if abs(x_i - x_c) < 1e-12:
        return None
    t = (0.0 - x_c) / (x_i - x_c)
    return (0.0, float(z_c + t * (z_i - z_c)))
