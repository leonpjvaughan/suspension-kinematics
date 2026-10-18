"""Every number a model has, in one file. See docs/config-layer.md §7.

Coordinates are metres, x lateral (positive outboard for the corner modelled), y forward,
z up, at the design position. A body's local frame *is* the world frame at design, so a
point's local coordinate is the number written here.
"""

from __future__ import annotations

from typing import Annotated

from pydantic import BaseModel, ConfigDict, Field

Vec3 = Annotated[list[float], Field(min_length=3, max_length=3)]
Tensor6 = Annotated[list[float], Field(min_length=6, max_length=6)]


class JointNumbers(BaseModel):
    """Per-joint numbers, keyed by component where a component needs one."""

    model_config = ConfigDict(extra="forbid")

    #: `axial` kinds. Defaults to the design separation; giving it explicitly is a check.
    length: float | None = None
    #: `plane` kinds. Required, and must equal the design gap.
    gap: float | None = None
    #: `spring` components: a linear compliance, m/N (translation, `length`, `gap`) or
    #: rad/(N m) (rotation). Exactly one of `compliance` and `stiffness` per component.
    compliance: dict[str, float] = {}
    stiffness: dict[str, float] = {}
    #: Reserved for dynamics. Accepted and ignored by the kinematic solver.
    damping: dict[str, float] = {}


class BodyMass(BaseModel):
    model_config = ConfigDict(extra="forbid")

    mass: float
    #: A site name, or a raw [x, y, z].
    com: str | Vec3
    #: [ixx, iyy, izz, ixy, ixz, iyz] about the centre of mass, in the design frame.
    inertia: Tensor6


class DynamicsNumbers(BaseModel):
    """Read by the dynamics work, ignored by the kinematic solver."""

    model_config = ConfigDict(extra="forbid")

    gravity: Vec3 = [0.0, 0.0, -9.81]
    bodies: dict[str, BodyMass] = {}


class Geometry(BaseModel):
    model_config = ConfigDict(extra="forbid")

    version: int = 1
    points: dict[str, Vec3]
    #: Directions, not positions. Normalised on load.
    axes: dict[str, Vec3]
    joints: dict[str, JointNumbers] = {}
    scalars: dict[str, float] = {}
    dynamics: DynamicsNumbers | None = None
