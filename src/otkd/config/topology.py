"""The symbolic half of a model: bodies, sites, joints, actuators, roles.

No numbers live here. Every field is a name, a mode or a reference; the numbers they point
at are in `geometry.yaml`. See docs/config-layer.md §1–§5.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from .kinds import ConstraintKind, Mode


class BodySpec(BaseModel):
    model_config = ConfigDict(extra="forbid")

    fixed: bool = False
    #: Geometry points this body owns. Ownership decides how an output point moves; it does
    #: not affect the constraints, where a site's origin is shared by both bodies at design.
    points: list[str] = []


class Site(BaseModel):
    """A point with an optional orientation, in world coordinates at the design position.

    For the axis-aligned families the site's **z is the axis**: the hinge of a revolute, the
    slide of a prismatic, the normal of a planar joint or a plane gap.
    """

    model_config = ConfigDict(extra="forbid")

    origin: str
    x: str | None = None
    y: str | None = None
    z: str | None = None


class Joint(BaseModel):
    model_config = ConfigDict(extra="forbid", populate_by_name=True)

    name: str
    kind: str
    a: str
    b: str
    #: `frame` kinds: the one site the two bodies share at design.
    site: str | None = None
    #: `axial` and `plane` kinds: the two points that do *not* coincide at design.
    from_: str | None = Field(default=None, alias="from")
    to: str | None = None
    #: Per-joint override of the kind's modes, e.g. `{length: spring}`.
    modes: dict[str, Mode] = {}


class Actuator(BaseModel):
    """A device on one free component of one joint. No numbers — a study supplies the law."""

    model_config = ConfigDict(extra="forbid")

    name: str
    joint: str
    dof: str
    effort: Literal["position", "force", "torque"] = "position"


class Roles(BaseModel):
    """Which body outputs are measured against, and which body is the road."""

    model_config = ConfigDict(extra="forbid")

    reference: str | None = None
    road: str | None = None
    #: The two geometry points that define the wheel axle, e.g. [wc, wco]. Outputs need it.
    axle: list[str] | None = None


class Topology(BaseModel):
    model_config = ConfigDict(extra="forbid")

    version: int = 1
    id: str
    #: Path to the constraint library, relative to this file.
    constraints: str = "constraints.yaml"
    bodies: dict[str, BodySpec]
    roles: Roles = Roles()
    sites: dict[str, Site] = {}
    #: Local kinds, merged with the library. Names must be unique across the two.
    constraint_kinds: dict[str, ConstraintKind] = {}
    joints: list[Joint]
    actuators: list[Actuator] = []
