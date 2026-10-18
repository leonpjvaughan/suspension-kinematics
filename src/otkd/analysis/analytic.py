"""Closed-form kinematics for the wishbone, as an independent cross-check.

The derivation is deliberately different from the solver's: explicit arm angles about the
hinge axes, circle-circle intersections solved in closed form, and no quaternion, no
Jacobian, no Newton. If the two agree over a sweep, the symbolic derivation is right; if they
disagree, the symbolic derivation is wrong. That is the whole function of this module.

The construction:

1. the lower ball joint traces a circle about the lower hinge, parametrised by the driven
   angle `theta`;
2. the upper ball joint traces a circle about the upper hinge; the upright's fixed length
   `|lbj - ubj|` gives `A cos(phi) + B sin(phi) = C`, which has two solutions — the two
   assembly branches;
3. the upright's remaining freedom is a rotation `psi` about the current `lbj - ubj` line,
   fixed by the tie rod, which gives the same kind of equation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config.kinds import ROTATION
from ..problem.compile import Problem


@dataclass(frozen=True)
class Wishbone:
    lcaf: np.ndarray
    lca_axis: np.ndarray
    lbj: np.ndarray
    ucaf: np.ndarray
    uca_axis: np.ndarray
    ubj: np.ndarray
    tri: np.ndarray
    tro: np.ndarray
    wc: np.ndarray
    wco: np.ndarray
    l_upper: float
    l_tie: float


def extract(problem: Problem) -> Wishbone | None:
    """The wishbone structure, or None if this model is not one.

    Requires each arm to be attached to the reference body by a single revolute, because
    that is what makes the closed form available; a two-bush arm is redundant and has no
    such parametrisation.
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
    arms = []
    for sphere in spheres:
        arm = sphere.b if sphere.a == upright else sphere.a
        hinges = [
            joint
            for joint in resolved.joints.values()
            if joint.family == "frame"
            and {joint.a, joint.b} == {arm, resolved.reference}
            and joint.site is not None
        ]
        # One revolute: its site's z is the hinge. Two sphericals: the line through their
        # origins is the hinge, which is exactly what the two-leg model means.
        revolutes = [
            joint
            for joint in hinges
            if joint.modes["rz"] == "free"
            and all(joint.modes[c] == "locked" for c in ("tx", "ty", "tz", "rx", "ry"))
        ]
        if len(revolutes) == 1:
            arms.append((sphere.site, revolutes[0].site, None))
        elif len(hinges) == 2 and all(
            all(joint.modes[c] == "locked" for c in ("tx", "ty", "tz"))
            and all(joint.modes[c] == "free" for c in ROTATION)
            for joint in hinges
        ):
            first = resolved.sites[hinges[0].site].origin
            second = resolved.sites[hinges[1].site].origin
            arms.append((sphere.site, hinges[0].site, _unit(second - first)))
        else:
            return None

    def point(name: str) -> np.ndarray:
        return resolved.points[name]

    (lbj_site, lower_hinge, lower_axis), (ubj_site, upper_hinge, upper_axis) = arms
    axial = [joint for joint in resolved.joints.values() if joint.family == "axial"]
    if len(axial) != 1:
        return None
    lbj = resolved.sites[lbj_site].origin
    ubj = resolved.sites[ubj_site].origin
    tri = resolved.sites[axial[0].from_].origin
    tro = resolved.sites[axial[0].to].origin
    return Wishbone(
        lcaf=resolved.sites[lower_hinge].origin,
        lca_axis=lower_axis if lower_axis is not None else _unit(resolved.sites[lower_hinge].normal),
        lbj=lbj,
        ucaf=resolved.sites[upper_hinge].origin,
        uca_axis=upper_axis if upper_axis is not None else _unit(resolved.sites[upper_hinge].normal),
        ubj=ubj,
        tri=tri,
        tro=tro,
        wc=resolved.points[resolved.axle[0]],
        wco=resolved.points[resolved.axle[1]],
        l_upper=float(np.linalg.norm(ubj - lbj)),
        l_tie=float(np.linalg.norm(tro - tri)),
    )


def walk(wishbone: Wishbone, thetas: np.ndarray) -> list[dict]:
    """The closed-form solution at each driven angle, following the branch nearest the last.

    Continuation is anchored at the design position and walks outward in both directions,
    because the two assembly branches are equidistant from it at the first grid point: a
    sweep that starts at the end of its range has no way to say which assembly it is on.
    """
    thetas = np.asarray(thetas, dtype=float)
    poses: list[dict | None] = [None] * len(thetas)
    anchor = int(np.argmin(np.abs(thetas)))

    def step(index: int, phi: float, psi: float) -> tuple[dict, float, float]:
        theta = float(thetas[index])
        lower = _circle(wishbone.lcaf, wishbone.lca_axis, wishbone.lbj)
        upper = _circle(wishbone.ucaf, wishbone.uca_axis, wishbone.ubj)
        lbj = _on_circle(lower, theta)
        phi = _solve(upper, lbj, wishbone.l_upper, phi)
        ubj = _on_circle(upper, phi)
        design_axis = _unit(wishbone.ubj - wishbone.lbj)
        kingpin = _unit(ubj - lbj)
        rot0 = _rotation_between(design_axis, kingpin)
        arm = rot0 @ (wishbone.tro - wishbone.lbj)
        spin = _circle(lbj, kingpin, lbj + arm)
        psi = _solve(spin, wishbone.tri, wishbone.l_tie, psi)
        rotation = _rot(kingpin, psi) @ rot0
        axle = _unit(rotation @ (wishbone.wco - wishbone.wc))
        pose = {
            "theta": theta,
            "lbj": lbj,
            "ubj": ubj,
            "tro": _on_circle(spin, psi),
            "axle": axle,
            "camber": float(np.degrees(np.arcsin(np.clip(-axle[2], -1.0, 1.0)))),
        }
        return pose, phi, psi

    poses[anchor], phi, psi = step(anchor, 0.0, 0.0)
    for index in range(anchor + 1, len(thetas)):
        poses[index], phi, psi = step(index, phi, psi)
    phi, psi = 0.0, 0.0
    for index in range(anchor - 1, -1, -1):
        poses[index], phi, psi = step(index, phi, psi)
    return poses  # type: ignore[return-value]


def camber_curve(wishbone: Wishbone, thetas: np.ndarray) -> list[float]:
    """Camber in degrees at each driven angle."""
    return [pose["camber"] for pose in walk(wishbone, thetas)]


# -- geometry --------------------------------------------------------------------------


def _circle(centre: np.ndarray, axis: np.ndarray, point: np.ndarray):
    axis = _unit(axis)
    v = point - centre
    along = float(v @ axis)
    perp = v - along * axis
    return centre + along * axis, perp, np.cross(axis, perp)


def _on_circle(circle, angle: float) -> np.ndarray:
    centre, a, b = circle
    return centre + np.cos(angle) * a + np.sin(angle) * b


def _solve(circle, target: np.ndarray, length: float, near: float) -> float:
    centre, a, b = circle
    delta = centre - target
    aa = 2.0 * float(delta @ a)
    bb = 2.0 * float(delta @ b)
    cc = length**2 - float(delta @ delta) - float(a @ a)
    radius = float(np.hypot(aa, bb))
    if radius == 0.0:
        raise ValueError("degenerate circle: the arm has no radius")
    ratio = max(-1.0, min(1.0, cc / radius))
    base = float(np.arctan2(bb, aa))
    candidates = [base + np.arccos(ratio), base - np.arccos(ratio)]
    return min(candidates, key=lambda angle: abs(_wrap(angle - near)))


def _rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    axis = np.cross(a, b)
    sine = float(np.linalg.norm(axis))
    cosine = float(a @ b)
    if sine < 1e-15:
        return np.eye(3) if cosine > 0 else _rot(a, np.pi)
    return _rot(axis, float(np.arctan2(sine, cosine)))


def _rot(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = _unit(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def _unit(v: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    return v / norm if norm else v


def _wrap(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))
