"""The question being asked of a model: which actuator moves, over what, under what load.

A study is not part of the mechanism. The same topology and geometry with a second study is
a second question, so the sweep lives here rather than in `geometry.yaml`. See
docs/config-layer.md §8.
"""

from __future__ import annotations

from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .geometry import Vec3


class Law(BaseModel):
    """How an actuator's coordinate (or a load) varies across the study."""

    model_config = ConfigDict(extra="forbid", populate_by_name=True)

    kind: Literal["constant", "linear", "table"] = "constant"
    value: float = 0.0
    start: float | None = Field(default=None, alias="from")
    stop: float | None = Field(default=None, alias="to")
    #: Inclusive count, so the grid has exactly this many frames.
    steps: int | None = None
    values: list[float] | None = None

    @model_validator(mode="after")
    def _complete(self) -> "Law":
        if self.kind == "linear":
            if self.start is None or self.stop is None:
                raise ValueError("linear law needs `from` and `to`")
            if self.steps is None or self.steps < 1:
                raise ValueError("linear law needs `steps` >= 1")
        if self.kind == "table" and not self.values:
            raise ValueError("table law needs a non-empty `values`")
        return self

    def grid(self) -> np.ndarray:
        if self.kind == "constant":
            return np.array([self.value], dtype=float)
        if self.kind == "linear":
            assert self.start is not None and self.stop is not None and self.steps is not None
            if self.steps == 1:
                return np.array([self.start], dtype=float)
            return np.linspace(self.start, self.stop, self.steps)
        assert self.values is not None
        return np.asarray(self.values, dtype=float)


class Axis(BaseModel):
    """A geometry number the study sweeps, outside the drivers.

    One law applied to every number named, because one bush is one stiffness: a joint whose three
    translation components deflect together is one axis, not three. Each value starts a fresh pass
    over the drivers, seeded from the design pose — a soft joint and a stiff joint are different
    assemblies, and branch continuity is a property of a pass, not of the study.
    """

    model_config = ConfigDict(extra="forbid")

    #: Dotted geometry names: `compliance.uca-up.tx`, `length.tie-rod`, `point.lcaf.2`.
    numbers: list[str]
    law: Law = Law()


class Driver(BaseModel):
    model_config = ConfigDict(extra="forbid")

    actuator: str
    law: Law = Law()


class Load(BaseModel):
    """A wrench applied at a site, in world axes. Input, never a function of the state."""

    model_config = ConfigDict(extra="forbid")

    site: str
    force: Vec3 = [0.0, 0.0, 0.0]
    torque: Vec3 | None = None
    law: Law = Law()


class Study(BaseModel):
    model_config = ConfigDict(extra="forbid")

    version: int = 1
    analysis: Literal["kinematics", "dynamics"] = "kinematics"
    drivers: list[Driver] = []
    #: Geometry numbers swept outside the drivers: one pass over the drivers per value.
    axes: list[Axis] = []
    loads: list[Load] = []
    #: A dynamics study's time span and output count. Required for `analysis: dynamics`.
    duration: float | None = None
    steps: int | None = None
    #: Row names to remove, e.g. `lca-front.tx`. Must have been redundant.
    drop: list[str] = []
    seed: Literal["previous", "design"] = "previous"
